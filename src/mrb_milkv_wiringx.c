#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include <pthread.h>

#include <mruby.h>
#include <mruby/array.h>
#include <mruby/hash.h>
#include <mruby/variable.h>
#include <mruby/value.h>
#include <wiringx.h>

// Set MILKV_DUO_VARIANT in mruby build config.
// Precompiler isn't treating strings in -D flags as strings, so do this nonsense.
#define _milkv_duo      "milkv_duo"
#define _milkv_duo256m  "milkv_duo256m"
#define _milkv_duos     "milkv_duos"

/*****************************************************************************/
/*                             TIMING HELPERS                                */
/*****************************************************************************/
static uint64_t nanoDiff(const struct timespec *event2, const struct timespec *event1) {
  uint64_t event2_ns = (uint64_t)event2->tv_sec * 1000000000LL + event2->tv_nsec;
  uint64_t event1_ns = (uint64_t)event1->tv_sec * 1000000000LL + event1->tv_nsec;
  return event2_ns - event1_ns;
}

static uint64_t nanosSince(const struct timespec *event) {
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  return nanoDiff(&now, event);
}

static void nanoDelay(uint64_t nanos) {
  struct timespec refTime;
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &refTime);
  now = refTime;
  while(nanoDiff(&now, &refTime) < nanos) {
    clock_gettime(CLOCK_MONOTONIC, &now);
  }
}

static void microDelay(uint64_t micros) {
  nanoDelay(micros * 1000);
}

static mrb_value
mrb_microDelay(mrb_state* mrb, mrb_value self) {
  mrb_int micros;
  mrb_get_args(mrb, "i", &micros);
  microDelay(micros);
  return mrb_nil_value();
}

/****************************************************************************/
/*                                 GPIO                                     */
/****************************************************************************/
static mrb_value
mrbWX_setup(mrb_state* mrb, mrb_value self) {
  int result = wiringXSetup(MILKV_DUO_VARIANT, NULL);
  if (result == -1) {
    wiringXGC();
    mrb_raise(mrb, E_RUNTIME_ERROR, "Wiring X setup failed");
  }
  return mrb_nil_value();
}

static mrb_value
mrbWX_valid_gpio(mrb_state* mrb, mrb_value self) {
  mrb_int pin, valid;
  mrb_get_args(mrb, "i", &pin);
  valid = wiringXValidGPIO(pin);
  return mrb_fixnum_value(valid);
}

static mrb_value
mrbWX_pin_mode(mrb_state* mrb, mrb_value self) {
  mrb_int pin, mode;
  mrb_get_args(mrb, "ii", &pin, &mode);
  pinMode(pin, mode);
  return mrb_nil_value();
}

static mrb_value
mrbWX_digital_write(mrb_state* mrb, mrb_value self) {
  mrb_int pin, state;
  mrb_get_args(mrb, "ii", &pin, &state);
  digitalWrite(pin, state);
  return mrb_nil_value();
}

static mrb_value
mrbWX_digital_read(mrb_state* mrb, mrb_value self) {
  mrb_int pin, state;
  mrb_get_args(mrb, "i", &pin);
  state = digitalRead(pin);
  return mrb_fixnum_value(state);
}

/****************************************************************************/
/*                          GPIO ALERTS / LISTENERS                         */
/****************************************************************************/
typedef struct
{
   // uint64_t timestamp;
   int pin;
   int level;
} wxGpioReport_t;

// Set up a queue for up to 2**16 GPIO reports.
#define QUEUE_LENGTH UINT16_MAX + 1
static wxGpioReport_t reportQueue[QUEUE_LENGTH];
static uint16_t qWritePos = 1;
static uint16_t qReadPos  = 0;

// Add a report to the queue
static void mrbWX_queue_report(uint32_t pin, uint32_t level) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  // reportQueue[qWritePos].timestamp  = (uint64_t)((ts.tv_sec * 1000000000ULL) + (ts.tv_nsec));
  reportQueue[qWritePos].pin        = pin;
  reportQueue[qWritePos].level      = level;

  // Update queue pointers
  qWritePos++;
  // qReadPos is the LAST report read. If passing by 1, increment it too. Lose oldest data first.
  if (qWritePos - qReadPos == 1) qReadPos++;
}

// Storage for up to 32 listeners
#define LISTENER_COUNT 32
typedef struct
{
  int active;
  int pin;
  int state;
  int changed;
} wxGpioListener_t;
static wxGpioListener_t listeners[LISTENER_COUNT];
static int lastActiveListener = -1;

// Poll in a separate thread roughly every 100 microseconds.
#define LISTEN_INTERVAL_NS 100000
static pthread_mutex_t queueLock;
static pthread_t listenThread;
static int runListenThread = 0;

void mrbWX_listen_thread(){
  struct timespec start, finish, sleep_time;
  sleep_time.tv_sec = 0;
  uint64_t time_taken;
  int readState;

  while(runListenThread){
    clock_gettime(CLOCK_MONOTONIC, &start);

    // Update all the listeners.
    pthread_mutex_lock(&queueLock);
    for(int i=0; i<=lastActiveListener; i++) {
      if (listeners[i].active == 1) {
        readState = digitalRead(listeners[i].pin);
        if (readState != listeners[i].state) {
          listeners[i].state   = readState;
          listeners[i].changed = 1;
        }
      }
    }

    // Generate alerts.
    for(int i=0; i<=lastActiveListener; i++) {
      if(listeners[i].changed == 1) {
         mrbWX_queue_report(listeners[i].pin, listeners[i].state);
         listeners[i].changed = 0;
      }
    }
    pthread_mutex_unlock(&queueLock);

    clock_gettime(CLOCK_MONOTONIC, &finish);

    // Sleep
    time_taken = nanoDiff(&finish, &start);
    if (time_taken < LISTEN_INTERVAL_NS) {
      sleep_time.tv_nsec = LISTEN_INTERVAL_NS - time_taken;
      nanosleep(&sleep_time, NULL);
    }
  }
}

static void
mrbWX_start_listen_thread(mrb_state* mrb) {
  if (runListenThread != 1){
    // Deactive all listeners
    pthread_mutex_lock(&queueLock);
    for(int i=0; i<LISTENER_COUNT; i++) {
      listeners[i].active = 0;
      listeners[i].pin    = -1;
    }
    pthread_mutex_unlock(&queueLock);

    // Start the thread
    runListenThread = 1;
    int err = pthread_create(&listenThread, NULL, mrbWX_listen_thread, NULL);
    if(err != 0) mrb_raise(mrb, E_TYPE_ERROR, "Could not start listen thread");
  }
}

static mrb_value
mrbWX_claim_alert(mrb_state* mrb, mrb_value self) {
  mrb_int pin;
  mrb_get_args(mrb, "i", &pin);

  // Start listen thread if needed;
  mrbWX_start_listen_thread(mrb);

  // Check for existing listener on this pin.
  pthread_mutex_lock(&queueLock);
  int pos = 0;
  while(pos < LISTENER_COUNT) {
    if (listeners[pos].pin == pin) break;
    pos++;
  }

  // Find lowest inactive position if not.
  if (pos == LISTENER_COUNT) {
    pos = 0;
    while(pos < LISTENER_COUNT) {
      if (listeners[pos].active == 0) break;
      pos++;
    }
  }

  // If available, set it up.
  if (pos < LISTENER_COUNT) {
    pinMode(pin, PINMODE_INPUT);
    listeners[pos].active  = 1;
    listeners[pos].pin     = pin;
    listeners[pos].state   = digitalRead(pin);
    listeners[pos].changed = 0;
    if (pos > lastActiveListener) lastActiveListener = pos;
  }
  pthread_mutex_unlock(&queueLock);

  return mrb_nil_value();
}

static mrb_value
mrbWX_stop_alert(mrb_state* mrb, mrb_value self) {
  mrb_int pin;
  mrb_get_args(mrb, "i", &pin);

  // Stop any listener with this pin.
  for(int i=0; i<LISTENER_COUNT; i++) {
    if (listeners[i].pin == pin) {
      listeners[i].active = 0;
    }
  }

  // If none active, let the listen thread stop.
  int oneActive = 0;
  for(int i=0; i<LISTENER_COUNT; i++) {
    if (listeners[i].active == 1) oneActive = 1;
  }
  if (oneActive == 0) runListenThread = 0;
}

static mrb_value
mrbWX_get_alert(mrb_state* mrb, mrb_value self) {
  mrb_value hash = mrb_hash_new(mrb);
  uint8_t popped = 0;

  pthread_mutex_lock(&queueLock);
  // qWritePos is where the NEXT report will go. Always trail it by 1.
  if (qWritePos - qReadPos != 1){
    qReadPos += 1;
    // mrb_hash_set(mrb, hash, mrb_symbol_value(mrb_intern_lit(mrb, "timestamp")), mrb_fixnum_value(reportQueue[qReadPos].timestamp));
    mrb_hash_set(mrb, hash, mrb_symbol_value(mrb_intern_lit(mrb, "pin")), mrb_fixnum_value(reportQueue[qReadPos].pin));
    mrb_hash_set(mrb, hash, mrb_symbol_value(mrb_intern_lit(mrb, "level")), mrb_fixnum_value(reportQueue[qReadPos].level));
    popped = 1;
  }
  pthread_mutex_unlock(&queueLock);

  return popped ? hash : mrb_nil_value();
}

/****************************************************************************/
/*                                   PWM                                    */
/****************************************************************************/
static mrb_value
mrbWX_pwm_enable(mrb_state* mrb, mrb_value self) {
  mrb_int pin, enabled;
  mrb_get_args(mrb, "ii", &pin, &enabled);
  wiringXPWMEnable(pin, enabled);
  return mrb_nil_value();
}

static mrb_value
mrbWX_pwm_set_polarity(mrb_state* mrb, mrb_value self) {
  mrb_int pin, polarity;
  mrb_get_args(mrb, "ii", &pin, &polarity);
  wiringXPWMSetPolarity(pin, polarity);
  return mrb_nil_value();
}

static mrb_value
mrbWX_pwm_set_period(mrb_state* mrb, mrb_value self) {
  mrb_int pin, period;
  mrb_get_args(mrb, "ii", &pin, &period);
  wiringXPWMSetPeriod(pin, period);
  return mrb_nil_value();
}

static mrb_value
mrbWX_pwm_set_duty(mrb_state* mrb, mrb_value self) {
  mrb_int pin, duty;
  mrb_get_args(mrb, "ii", &pin, &duty);
  wiringXPWMSetDuty(pin, duty);
  return mrb_nil_value();
}

static mrb_value
tx_wave_ook(mrb_state* mrb, mrb_value self) {
  mrb_int pin, duty;
  mrb_value txArray;
  mrb_get_args(mrb, "iiA", &pin, &duty, &txArray);
  if (!mrb_array_p(txArray)) mrb_raise(mrb, E_TYPE_ERROR, "OOK pulses must be given as Array");

  // Copy mrb array txArray into C array nanoPulses.
  mrb_int length = RARRAY_LEN(txArray);
  uint64_t nanoPulses[length];
  for (int i=0; i<length; i++) {
    mrb_value elem = mrb_ary_ref(mrb, txArray, i);
    if (!mrb_integer_p(elem)) mrb_raise(mrb, E_TYPE_ERROR, "Each I2C byte must be Integer");
    // Should validate +ve too?
    nanoPulses[i] = mrb_integer(elem) * 1000;
  }

  // Enable before
  wiringXPWMEnable(pin, 1);

  // Even numbered indices are on, odd numbered off.
  for (int i=0; i<length; i++) {
    if (i % 2 == 0) {
      wiringXPWMSetDuty(pin, duty);
    } else {
      wiringXPWMSetDuty(pin, 0);
    }
    // Wait for pulse time.
    nanoDelay(nanoPulses[i]);
  }

  // Disable after
  wiringXPWMEnable(pin, 0);
  return mrb_fixnum_value(length);
}

/****************************************************************************/
/*                                   I2C                                    */
/****************************************************************************/
static mrb_value
mrbWX_i2c_setup(mrb_state* mrb, mrb_value self) {
  // Args are Linux I2C dev index, and peripheral I2C address.
  mrb_int index, address;
  mrb_get_args(mrb, "ii", &index, &address);

  // Full I2C dev path.
  char i2c_dev[12];
  snprintf(i2c_dev, sizeof(i2c_dev), "/dev/i2c-%d", index);

  // Setup and return the file descriptor to mruby.
  int fd = wiringXI2CSetup(i2c_dev, address);
  return mrb_fixnum_value(fd);
}

static mrb_value
mrbWX_i2c_write(mrb_state* mrb, mrb_value self) {
  // Args are I2C file descriptor, and array of bytes to send.
  mrb_int fd;
  mrb_value txArray;
  mrb_get_args(mrb, "iA", &fd, &txArray);
  if (!mrb_array_p(txArray)) mrb_raise(mrb, E_TYPE_ERROR, "I2C bytes must be given as Array");

  // Copy mrb array txArray into C array txBuf.
  mrb_int length = RARRAY_LEN(txArray);
  uint8_t txBuf[length];
  for (int i=0; i<length; i++) {
    mrb_value elem = mrb_ary_ref(mrb, txArray, i);
    if (!mrb_integer_p(elem)) mrb_raise(mrb, E_TYPE_ERROR, "Each I2C byte must be Integer");
    txBuf[i] = mrb_integer(elem);
  }

  // Raw I2C write, returning number of bytes written.
  int result = write(fd, txBuf, sizeof(txBuf));
  return mrb_fixnum_value(result);
}

static mrb_value
mrbWX_i2c_read(mrb_state* mrb, mrb_value self) {
  // Args are I2C file descriptor, and number of bytes to read.
  mrb_int fd, length;
  mrb_get_args(mrb, "ii", &fd, &length);

  // Raw I2C read length bytes into C array.
  uint8_t rxBuf[length];
  read(fd, rxBuf, length);

  // Convert to mrb_ary and return.
  mrb_value rxArray = mrb_ary_new_capa(mrb, length);
  for (int i=0; i<length; i++) mrb_ary_push(mrb, rxArray, mrb_fixnum_value(rxBuf[i]));
  return rxArray;
}

/****************************************************************************/
/*                                   SPI                                    */
/****************************************************************************/
static mrb_value
mrbWX_spi_setup(mrb_state* mrb, mrb_value self) {
  // Args are Linux SPI dev index, and clock speed.
  mrb_int index, speed;
  mrb_get_args(mrb, "ii", &index, &speed);

  // Setup and return the file descriptor to mruby.
  int fd = wiringXSPISetup(index, speed);
  return mrb_fixnum_value(fd);
}

static mrb_value
mrbWX_spi_xfer(mrb_state* mrb, mrb_value self) {
  // Args are SPI index, array of bytes to write, length of bytes to read.
  mrb_int index, rxLength;
  mrb_value txArray;
  mrb_get_args(mrb, "iAi", &index, &txArray, &rxLength);
  if (!mrb_array_p(txArray)) mrb_raise(mrb, E_TYPE_ERROR, "SPI bytes must be given as Array");

  // Reading more than writing, or writing more than reading?
  mrb_int txLength = RARRAY_LEN(txArray);
  int length = (rxLength > txLength) ? rxLength : txLength;
  uint8_t rwBuf[length+1];

  // Copy bytes from txArray into rwBuffer.
  for (int i=0; i<txLength; i++) {
    mrb_value elem = mrb_ary_ref(mrb, txArray, i);
    if (!mrb_integer_p(elem)) mrb_raise(mrb, E_TYPE_ERROR, "Each SPI byte must be Integer");
    rwBuf[i] = mrb_integer(elem);
  }
  // Extend with 0s if needed.
  if (length > txLength) {
    for(int i=txLength; i<length; i++) rwBuf[i] = 0;
  }

  // Do the transfer.
  wiringXSPIDataRW(index, rwBuf, length);

  // Convert read bytes to mrb_ary and return.
  mrb_value rxArray = mrb_ary_new_capa(mrb, rxLength);
  for (int i=0; i<rxLength; i++) mrb_ary_push(mrb, rxArray, mrb_fixnum_value(rwBuf[i]));
  return rxArray;
}

static mrb_value
mrbWX_spi_ws2812_write(mrb_state* mrb, mrb_value self){
  // Args are SPI index, and array of pixels to write.
  mrb_int index;
  mrb_value pixelArray;
  mrb_get_args(mrb, "iA", &index, &pixelArray);
  if (!mrb_array_p(pixelArray)) mrb_raise(mrb, E_TYPE_ERROR, "WS2812 bytes must be given as Array");

  int count = RARRAY_LEN(pixelArray);
  int zeroesBefore = 1;  // 1/4 of lgpio rounded up, since uint32 instead of uint8.
  int zeroesAfter  = 23; // 1/4 of lgpio rounded up, since uint32 instead of uint8.
  int txBufLength  = zeroesBefore + count + zeroesAfter; // Don't 3x count, since 1:1 mapping of uint8 to uint32.
  uint32_t txBuf[txBufLength];
  for (int i=0; i<txBufLength; i++) { txBuf[i] = 0; }

  mrb_value mrb_currentByte;
  uint8_t   currentByte;
  uint8_t   currentBit;
  uint32_t  temp;

  for (int i=0; i<count; i++){
    mrb_currentByte = mrb_ary_ref(mrb, pixelArray, i);
    if (!mrb_integer_p(mrb_currentByte)) mrb_raise(mrb, E_TYPE_ERROR, "Each WS2812 byte must be Integer");
    currentByte = mrb_integer(mrb_currentByte);

    // 4 SPI bits per data bit, instead of 3 like other implementations.
    // The extra 0 holds the line low a bit longer, which is fine.
    // Needed because either aligned memory access, or SPI clock not 100% consistent. Not sure which.
    temp = 0;
    for (int j=0; j<8; j++) {
      currentBit  = (currentByte & (1 << j));
      temp = temp << 4;
      temp = (currentBit == 0) ? (temp | 0b100) : (temp | 0b110);
    }
    txBuf[i+zeroesBefore] = temp;
  }

  int result = wiringXSPIDataRW(index, (uint8_t *)txBuf, sizeof(txBuf));
  return mrb_fixnum_value(result);
}

/****************************************************************************/
/*                               GEM INIT                                   */
/****************************************************************************/
void
mrb_mruby_milkv_wiringx_gem_init(mrb_state* mrb) {
  // Module
  struct RClass *mrbWX = mrb_define_module(mrb, "WiringX");

  // Constants
  mrb_define_const(mrb, mrbWX, "PINMODE_NOT_SET",   mrb_fixnum_value(PINMODE_NOT_SET));
  mrb_define_const(mrb, mrbWX, "PINMODE_INPUT",     mrb_fixnum_value(PINMODE_INPUT));
  mrb_define_const(mrb, mrbWX, "PINMODE_OUTPUT",    mrb_fixnum_value(PINMODE_OUTPUT));
  mrb_define_const(mrb, mrbWX, "PINMODE_INTERRUPT", mrb_fixnum_value(PINMODE_INTERRUPT));
  mrb_define_const(mrb, mrbWX, "LOW",               mrb_fixnum_value(LOW));
  mrb_define_const(mrb, mrbWX, "HIGH",              mrb_fixnum_value(HIGH));

  // Class Methods
  mrbWX_setup(mrb, mrb_nil_value()); // Save user from calling WiringX.setup each script.
  mrb_define_method(mrb, mrbWX, "micro_delay",    mrb_microDelay,       MRB_ARGS_REQ(1));
  mrb_define_method(mrb, mrbWX, "setup",          mrbWX_setup,          MRB_ARGS_REQ(0));
  mrb_define_method(mrb, mrbWX, "valid_gpio",     mrbWX_valid_gpio,     MRB_ARGS_REQ(1));

  // Digital I/O
  mrb_define_method(mrb, mrbWX, "pin_mode",       mrbWX_pin_mode,       MRB_ARGS_REQ(2));
  mrb_define_method(mrb, mrbWX, "digital_write",  mrbWX_digital_write,  MRB_ARGS_REQ(2));
  mrb_define_method(mrb, mrbWX, "digital_read",   mrbWX_digital_read,   MRB_ARGS_REQ(1));

  // GPIO Alerts
  mrb_define_method(mrb, mrbWX, "claim_alert",    mrbWX_claim_alert,    MRB_ARGS_REQ(1));
  mrb_define_method(mrb, mrbWX, "stop_alert",     mrbWX_stop_alert,     MRB_ARGS_REQ(1));
  mrb_define_method(mrb, mrbWX, "get_alert",      mrbWX_get_alert,      MRB_ARGS_REQ(0));

  // PWM
  mrb_define_method(mrb, mrbWX, "pwm_enable",       mrbWX_pwm_enable,       MRB_ARGS_REQ(2));
  mrb_define_method(mrb, mrbWX, "pwm_set_polarity", mrbWX_pwm_set_polarity, MRB_ARGS_REQ(2));
  mrb_define_method(mrb, mrbWX, "pwm_set_period",   mrbWX_pwm_set_period,   MRB_ARGS_REQ(2));
  mrb_define_method(mrb, mrbWX, "pwm_set_duty",     mrbWX_pwm_set_duty,     MRB_ARGS_REQ(2));
  mrb_define_method(mrb, mrbWX, "tx_wave_ook",      tx_wave_ook,            MRB_ARGS_REQ(3));

  // I2C
  mrb_define_method(mrb, mrbWX, "i2c_setup",        mrbWX_i2c_setup,        MRB_ARGS_REQ(2));
  mrb_define_method(mrb, mrbWX, "i2c_write",        mrbWX_i2c_write,        MRB_ARGS_REQ(2));
  mrb_define_method(mrb, mrbWX, "i2c_read",         mrbWX_i2c_read,         MRB_ARGS_REQ(2));

  // SPI
  mrb_define_method(mrb, mrbWX, "spi_setup",        mrbWX_spi_setup,        MRB_ARGS_REQ(2));
  mrb_define_method(mrb, mrbWX, "spi_xfer",         mrbWX_spi_xfer,         MRB_ARGS_REQ(3));
  mrb_define_method(mrb, mrbWX, "spi_ws2812_write", mrbWX_spi_ws2812_write, MRB_ARGS_REQ(2));
}

void
mrb_mruby_milkv_wiringx_gem_final(mrb_state* mrb) {
}
