#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
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
mrb_micro_delay(mrb_state* mrb, mrb_value self) {
  mrb_int micros;
  mrb_get_args(mrb, "i", &micros);
  microDelay(micros);
  return mrb_nil_value();
}

/****************************************************************************/
/*                                 GPIO                                     */
/****************************************************************************/
static mrb_value
mrb_wx_setup(mrb_state* mrb, mrb_value self) {
  int result = wiringXSetup(MILKV_DUO_VARIANT, NULL);
  if (result == -1) {
    wiringXGC();
    mrb_raise(mrb, E_RUNTIME_ERROR, "Wiring X setup failed");
  }
  return mrb_nil_value();
}

static mrb_value
mrb_valid_gpio(mrb_state* mrb, mrb_value self) {
  mrb_int pin;
  mrb_get_args(mrb, "i", &pin);
  int invalid = wiringXValidGPIO(pin);
  return invalid ? mrb_false_value() : mrb_true_value();
}

static mrb_value
mrb_pin_mode(mrb_state* mrb, mrb_value self) {
  mrb_int pin, mode;
  mrb_get_args(mrb, "ii", &pin, &mode);
  pinMode(pin, mode);
  return mrb_nil_value();
}

static mrb_value
mrb_digital_write(mrb_state* mrb, mrb_value self) {
  mrb_int pin, state;
  mrb_get_args(mrb, "ii", &pin, &state);
  digitalWrite(pin, state);
  return mrb_nil_value();
}

static mrb_value
mrb_digital_read(mrb_state* mrb, mrb_value self) {
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
static pthread_mutex_t queueLock;
static uint16_t qWritePos = 1;
static uint16_t qReadPos  = 0;

// Add a report to the queue
static void queue_report(uint32_t pin, uint32_t level) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  // reportQueue[qWritePos].timestamp  = (uint64_t)((ts.tv_sec * 1000000000ULL) + (ts.tv_nsec));
  reportQueue[qWritePos].pin        = pin;
  reportQueue[qWritePos].level      = level;

  // Update queue pointers
  qWritePos++;
  // qReadPos is the LAST report read. If passing by 1, increment it too. Lose oldest data first.
  if (qWritePos - qReadPos == 1) {
    pthread_mutex_lock(&queueLock);
    if (qWritePos - qReadPos == 1) qReadPos++;
    pthread_mutex_unlock(&queueLock);
  }
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
static pthread_t listenThread;
static int runListenThread = 0;

static void listen(){
  struct timespec start, finish, sleep_time;
  sleep_time.tv_sec = 0;
  uint64_t time_taken;
  int readState;

  while(runListenThread){
    clock_gettime(CLOCK_MONOTONIC, &start);

    // Update all the listeners.
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
         queue_report(listeners[i].pin, listeners[i].state);
         listeners[i].changed = 0;
      }
    }

    clock_gettime(CLOCK_MONOTONIC, &finish);

    // Sleep
    time_taken = nanoDiff(&finish, &start);
    if (time_taken < LISTEN_INTERVAL_NS) {
      sleep_time.tv_nsec = LISTEN_INTERVAL_NS - time_taken;
      nanosleep(&sleep_time, NULL);
    }
  }
}

static void start_listen_thread(mrb_state* mrb) {
  if (runListenThread != 1){
    // Deactive all listeners
    for(int i=0; i<LISTENER_COUNT; i++) {
      listeners[i].active = 0;
      listeners[i].pin    = -1;
    }

    // Start the thread
    runListenThread = 1;
    int err = pthread_create(&listenThread, NULL, listen, NULL);
    if(err != 0) mrb_raise(mrb, E_TYPE_ERROR, "Could not start listen thread");
  }
}

static mrb_value
mrb_claim_alert(mrb_state* mrb, mrb_value self) {
  mrb_int pin;
  mrb_get_args(mrb, "i", &pin);

  // Start listen thread if needed;
  start_listen_thread(mrb);

  // Check for existing listener on this pin.
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

  return mrb_nil_value();
}

static mrb_value
mrb_stop_alert(mrb_state* mrb, mrb_value self) {
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
mrb_get_alert(mrb_state* mrb, mrb_value self) {
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
mrb_pwm_enable(mrb_state* mrb, mrb_value self) {
  mrb_int pin, enabled;
  mrb_get_args(mrb, "ii", &pin, &enabled);
  wiringXPWMEnable(pin, enabled);
  return mrb_nil_value();
}

static mrb_value
mrb_pwm_set_polarity(mrb_state* mrb, mrb_value self) {
  mrb_int pin, polarity;
  mrb_get_args(mrb, "ii", &pin, &polarity);
  wiringXPWMSetPolarity(pin, polarity);
  return mrb_nil_value();
}

static mrb_value
mrb_pwm_set_period(mrb_state* mrb, mrb_value self) {
  mrb_int pin, period;
  mrb_get_args(mrb, "ii", &pin, &period);
  wiringXPWMSetPeriod(pin, period);
  return mrb_nil_value();
}

static mrb_value
mrb_pwm_set_duty(mrb_state* mrb, mrb_value self) {
  mrb_int pin, duty;
  mrb_get_args(mrb, "ii", &pin, &duty);
  wiringXPWMSetDuty(pin, duty);
  return mrb_nil_value();
}

static mrb_value
mrb_tx_wave_ook(mrb_state* mrb, mrb_value self) {
  mrb_int pin, duty;
  mrb_value txArray;
  mrb_get_args(mrb, "iiA", &pin, &duty, &txArray);
  if (!mrb_array_p(txArray)) mrb_raise(mrb, E_TYPE_ERROR, "OOK pulses must be given as Array");

  // Copy mrb array txArray into C array nanoPulses.
  mrb_int length = RARRAY_LEN(txArray);
  uint64_t nanoPulses[length];
  for (int i=0; i<length; i++) {
    mrb_value elem = mrb_ary_ref(mrb, txArray, i);
    if (!mrb_integer_p(elem)) mrb_raise(mrb, E_TYPE_ERROR, "OOK pulses must be Integer");
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
/*                                   ADC                                    */
/****************************************************************************/
#define ADC_PATH "/sys/class/cvi-saradc/cvi-saradc0/device/cv_saradc"

static mrb_value
mrb_analog_read(mrb_state* mrb, mrb_value self) {
  mrb_int pin;
  mrb_get_args(mrb, "i", &pin);

  // Write ADC channel (corresponding to pin) to the ADC path.
  int fd = open(ADC_PATH, O_RDWR|O_NOCTTY|O_NDELAY);
  if (fd < 0) mrb_raise(mrb, E_RUNTIME_ERROR, "Could not open SARADC. Call Duo.saradc_initialize first");
  switch (pin) {
    case 26: write(fd, "1", 1); break;
    case 27: write(fd, "2", 1); break;
    default:
      close(fd);
      mrb_raise(mrb, E_ARGUMENT_ERROR, "Invalid GPIO for ADC. Only available on 26 and 27");
      break;
  }

  // Read ADC result from path.
  char buffer[8];
  int length = 0;
  int result = 0;
  lseek(fd, 0, SEEK_SET);
  length = read(fd, buffer, sizeof(buffer) -1);
  close(fd);

  // Convert to integer and return.
	if(length > 0) {
		result = atoi(buffer);
	} else {
	  return mrb_nil_value();
	}
	return mrb_fixnum_value(result);
}

/****************************************************************************/
/*                                   I2C                                    */
/****************************************************************************/
static mrb_value
mrb_i2c_setup(mrb_state* mrb, mrb_value self) {
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
mrb_i2c_write(mrb_state* mrb, mrb_value self) {
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
mrb_i2c_read(mrb_state* mrb, mrb_value self) {
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
mrb_spi_setup(mrb_state* mrb, mrb_value self) {
  // Args are Linux SPI dev index, and clock speed.
  mrb_int index, speed;
  mrb_get_args(mrb, "ii", &index, &speed);

  // Setup and return the file descriptor to mruby.
  int fd = wiringXSPISetup(index, speed);
  return mrb_fixnum_value(fd);
}

static mrb_value
mrb_spi_xfer(mrb_state* mrb, mrb_value self) {
  // Args are SPI index, array of bytes to write, length of bytes to read.
  mrb_int index, rxLength;
  mrb_value txArray;
  mrb_get_args(mrb, "iAi", &index, &txArray, &rxLength);
  if (!mrb_array_p(txArray)) mrb_raise(mrb, E_TYPE_ERROR, "SPI bytes must be given as Array");

  // Reading more than writing, or writing more than reading?
  mrb_int txLength = RARRAY_LEN(txArray);
  int length = (rxLength > txLength) ? rxLength : txLength;
  uint8_t rwBuf[length];

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
mrb_spi_ws2812_write(mrb_state* mrb, mrb_value self){
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

/*****************************************************************************/
/*                           BIT-BANG PULSE INPUT                            */
/*****************************************************************************/
static mrb_value
mrb_read_ultrasonic(mrb_state* mrb, mrb_value self) {
  mrb_int trigger, echo, triggerTime;
  mrb_get_args(mrb, "iii", &trigger, &echo, &triggerTime);

  struct timespec start;
  struct timespec now;
  uint8_t echoSeen = 0;

  // Pull down avoids false readings if disconnected.
  pinMode(echo, PINMODE_INPUT);

  // Initial pulse on the triger pin.
  pinMode(trigger, PINMODE_OUTPUT);
  digitalWrite(trigger, 0);
  microDelay(5);
  digitalWrite(trigger, 1);
  microDelay(triggerTime);
  digitalWrite(trigger, 0);

  clock_gettime(CLOCK_MONOTONIC, &start);
  now = start;

  // Wait for echo to go high, up to 25,000 us after trigger.
  while(nanoDiff(&now, &start) < 25000000){
    clock_gettime(CLOCK_MONOTONIC, &now);
    if (digitalRead(echo) == 1) {
      echoSeen = 1;
      start = now;
      break;
    }
  }
  if (!echoSeen) return mrb_nil_value();

  // Wait for echo to go low again, up to 25,000 us after echo start.
  while(nanoDiff(&now, &start) < 25000000){
    clock_gettime(CLOCK_MONOTONIC, &now);
    if (digitalRead(echo) == 0) break;
  }

  // High pulse time in microseconds.
  return mrb_fixnum_value(round(nanoDiff(&now, &start) / 1000.0));
}

static mrb_value
mrb_read_pulses_us(mrb_state* mrb, mrb_value self) {
  mrb_int gpio, reset_us, resetLevel, limit, timeout_ms;
  mrb_get_args(mrb, "iiiii", &gpio, &reset_us, &resetLevel, &limit, &timeout_ms);
  uint64_t timeout_ns = timeout_ms * 1000000;

  // State setup
  uint64_t pulses_ns[limit];
  uint32_t pulseIndex = 0;
  int      gpioState;
  struct timespec start;
  struct timespec lastPulse;
  struct timespec now;

  // Perform reset
  if (reset_us > 0) {
    pinMode(gpio, PINMODE_OUTPUT);
    digitalWrite(gpio, resetLevel);
    microDelay(reset_us);
  }

  // Initialize timing
  clock_gettime(CLOCK_MONOTONIC, &start);
  lastPulse = start;
  now       = start;

  // Switch to input and read initial state
  pinMode(gpio, PINMODE_INPUT);
  gpioState = digitalRead(gpio);

  // Read pulses in nanoseconds
  while ((nanoDiff(&now, &start) < timeout_ns) && (pulseIndex < limit)) {
    clock_gettime(CLOCK_MONOTONIC, &now);
    if (digitalRead(gpio) != gpioState) {
      pulses_ns[pulseIndex] = nanoDiff(&now, &lastPulse);
      lastPulse = now;
      gpioState = gpioState ^ 0b1;
      pulseIndex++;
    }
  }

  // Return mrb array of pulse as microseconds
  if (pulseIndex == 0) return mrb_nil_value();
  mrb_value retArray = mrb_ary_new_capa(mrb, pulseIndex);
  for(uint32_t i=0; i<pulseIndex; i++){
    uint32_t pulse_us = round(pulses_ns[i] / 1000.0);
    mrb_ary_push(mrb, retArray, mrb_fixnum_value(pulse_us));
  }
  return retArray;
}

/*****************************************************************************/
/*                       BIT BANG 1-WIRE HEPERS                              */
/*****************************************************************************/
static mrb_value
mrb_one_wire_bit_read(mrb_state* mrb, mrb_value self) {
  mrb_int pin;
  mrb_get_args(mrb, "i", &pin);

  uint8_t bit = 1;
  struct timespec start;
  struct timespec now;

  // Start the read slot.
  pinMode(pin, PINMODE_OUTPUT);
  digitalWrite(pin, 0);
  microDelay(1);
  pinMode(pin, PINMODE_INPUT);

  // Poll for 60us to see if pin goes low.
  clock_gettime(CLOCK_MONOTONIC, &start);
  now = start;
  while(nanoDiff(&now, &start) < 60000){
    if (digitalRead(pin) == 0) bit = 0;
    clock_gettime(CLOCK_MONOTONIC, &now);
  }
  return mrb_fixnum_value(bit);
}

static mrb_value
mrb_one_wire_bit_write(mrb_state* mrb, mrb_value self) {
  mrb_int pin, bit;
  mrb_get_args(mrb, "ii", &pin, &bit);

  // Write slot starts by going low for at least 1us.
  pinMode(pin, PINMODE_OUTPUT);
  digitalWrite(pin, 0);
  microDelay(1);

  // If 0, keep it low for the rest of the 60us write slot, then release.
  if (bit == 0) {
    microDelay(59);
    digitalWrite(pin, 1);
  // If 1, release first, then wait the rest of the 60us slot.
  } else {
    digitalWrite(pin, 1);
    microDelay(59);
  }
  pinMode(pin, PINMODE_INPUT);

  // Minimum 1us recovery time after each slot.
  microDelay(1);
  return mrb_nil_value();
}

static mrb_value
mrb_one_wire_reset(mrb_state* mrb, mrb_value self) {
  mrb_int pin;
  mrb_get_args(mrb, "i", &pin);

  struct timespec start;
  struct timespec now;
  uint8_t presence = 1;

  // Hold low for 500us to reset, then go high.
  pinMode(pin, PINMODE_OUTPUT);
  digitalWrite(pin, 0);
  microDelay(500);
  pinMode(pin, PINMODE_INPUT);

  // Poll for 250us. If a device pulls the line low, return 0 (device present).
  clock_gettime(CLOCK_MONOTONIC, &start);
  now = start;
  while(nanoDiff(&now, &start) < 250000){
    if (digitalRead(pin) == 0) presence = 0;
    clock_gettime(CLOCK_MONOTONIC, &now);
  }

  return mrb_fixnum_value(presence);
}

/****************************************************************************/
/*                            BIT-BANG I2C                                  */
/****************************************************************************/
static uint8_t bitReadU8(uint8_t* b, uint8_t i) {
  return (*b >> i) & 0b1;
}

static void bitWriteU8(uint8_t* b, uint8_t i, uint8_t v) {
  if (v == 0) {
    *b &= ~(1 << i);
  } else {
    *b |=  (1 << i);
  }
}

static uint8_t i2c_bb_sdaState = 2;
static uint8_t i2c_bb_slow = 1;

static void i2c_bb_sda_write(int sda, uint8_t bit) {
  if (i2c_bb_sdaState == bit) return;
  if ((i2c_bb_sdaState != 0) && (i2c_bb_sdaState != 1)) pinMode(sda, PINMODE_OUTPUT);
  digitalWrite(sda, bit);
  i2c_bb_sdaState = bit;
}

static void i2c_bb_sda_read(int sda) {
  if (i2c_bb_sdaState == 2) return;
  pinMode(sda, PINMODE_OUTPUT);
  digitalWrite(sda, 1);
  pinMode(sda, PINMODE_INPUT);
  i2c_bb_sdaState = 2;
}

static void i2c_bb_scl_write(int scl, uint8_t bit) {
  digitalWrite(scl, bit);
  // C906 goes too fast for some I2C peripherals, but calling nanoDelay
  // after each clock edge, even with 0ns, slows into the ~400 KHz range.
  if (i2c_bb_slow) nanoDelay(0);
}

// Start condition is SDA then SCL going low, from both high.
static void i2c_bb_start(int scl, int sda) {
  i2c_bb_sda_write(sda, 0);
  i2c_bb_scl_write(scl, 0);
}

// Stop condition is SDA going high, while SCL is also high.
static void i2c_bb_stop(int scl, int sda) {
  i2c_bb_sda_write(sda, 0);
  i2c_bb_scl_write(scl, 1);
  i2c_bb_sda_read(sda);
}

static uint8_t i2c_bb_read_bit(int scl, int sda) {
  uint8_t bit;
  // Ensure SDA high before pulling SCL high.
  i2c_bb_sda_read(sda);
  i2c_bb_scl_write(scl, 1);
  bit = digitalRead(sda);
  i2c_bb_scl_write(scl, 0);
  return bit;
}

static void i2c_bb_write_bit(int scl, int sda, uint8_t bit) {
  // Set SDA while SCL is low.
  i2c_bb_sda_write(sda, bit);
  i2c_bb_scl_write(scl, 1);
  i2c_bb_scl_write(scl, 0);
}

static uint8_t i2c_bb_read_byte(int scl, int sda, uint8_t ack) {
  uint8_t b;
  // Receive MSB first.
  for (int i=7; i>=0; i--) bitWriteU8(&b, i, i2c_bb_read_bit(scl, sda));
  // Send ACK or NACK and return byte.
  i2c_bb_write_bit(scl, sda, ack ^ 0b1);
  return b;
}

static int i2c_bb_write_byte(int scl, int sda, uint8_t b) {
  // Send MSB first.
  for (int i=7; i>=0; i--) i2c_bb_write_bit(scl, sda, bitReadU8(&b, i));
  // Return -1 for NACK, 0 for ACK.
  return (i2c_bb_read_bit(scl, sda) == 0) ? 0 : -1;
}

static void i2c_bb_reset(int scl, int sda) {
  // SCL is a driven output. SDA simulates an open drain with pullup enabled.
  // Simulate stop condition, but make sure modes and sda state set properly.
  i2c_bb_sda_read(sda);
  pinMode(scl, PINMODE_OUTPUT);
  digitalWrite(scl, 1);
  i2c_bb_slow = 1;
}

static mrb_value
mrb_i2c_bb_setup(mrb_state* mrb, mrb_value self) {
  mrb_int scl, sda;
  mrb_get_args(mrb, "ii", &scl, &sda);
  i2c_bb_reset(scl, sda);
}

static mrb_value
mrb_i2c_bb_search(mrb_state* mrb, mrb_value self) {
  mrb_int scl, sda;
  mrb_get_args(mrb, "ii", &scl, &sda);

  int ack;
  uint8_t present[128];
  for(int i=0; i<128; i++) present[i] = 0;
  uint8_t presentCount = 0;

  i2c_bb_reset(scl, sda);
  // Only addresses from 0x08 to 0x77 are usable (8 to 127).
  for (uint8_t addr = 0x08; addr < 0x78;  addr++) {
    i2c_bb_start(scl, sda);
    ack = i2c_bb_write_byte(scl, sda, ((addr << 1) & 0b11111110));
    i2c_bb_stop(scl, sda);
    if (ack == 0){
      present[addr] = 1;
      presentCount++;
    } else {
      present[addr] = 0;
    }
  }

  if (presentCount == 0) return mrb_nil_value();
  mrb_value retArray = mrb_ary_new_capa(mrb, presentCount);
  for (uint8_t addr = 0x08; addr < 0x78;  addr++) {
    if (present[addr] == 1) mrb_ary_push(mrb, retArray, mrb_fixnum_value(addr));
  }
  return retArray;
}

static mrb_value
mrb_i2c_bb_read(mrb_state* mrb, mrb_value self) {
  mrb_int scl, sda, address, count;
  mrb_get_args(mrb, "iiii", &scl, &sda, &address, &count);

  uint8_t rxBuf[count];
  uint8_t readAddress  = (address << 1) | 0b00000001;
  i2c_bb_reset(scl, sda);

  i2c_bb_start(scl, sda);
  int ack = i2c_bb_write_byte(scl, sda, readAddress);
  // Device with this address not present on the bus.
  if (ack != 0) return mrb_nil_value();
  // Read and ACK for all but the last byte.
  int pos = 0;
  while(pos < count-1) {
    rxBuf[pos] = i2c_bb_read_byte(scl, sda, 1);
    pos++;
  }
  rxBuf[pos] = i2c_bb_read_byte(scl, sda, 0);
  i2c_bb_stop(scl, sda);

  mrb_value retArray = mrb_ary_new_capa(mrb, count);
  for(int i=0; i<count; i++) mrb_ary_push(mrb, retArray, mrb_fixnum_value(rxBuf[i]));
  return retArray;
}

static mrb_value
mrb_i2c_bb_write(mrb_state* mrb, mrb_value self) {
  mrb_int scl, sda, address;
  mrb_value txArray, options;
  mrb_get_args(mrb, "iiiA|H", &scl, &sda, &address, &txArray, &options);
  if (!mrb_array_p(txArray)) mrb_raise(mrb, E_TYPE_ERROR, "I2C bytes must be given as Array");

  // Copy mrb array txArray into C array txBuf.
  mrb_int length = RARRAY_LEN(txArray);
  uint8_t txBuf[length];
  for (int i=0; i<length; i++) {
    mrb_value elem = mrb_ary_ref(mrb, txArray, i);
    if (!mrb_integer_p(elem)) mrb_raise(mrb, E_TYPE_ERROR, "Each I2C byte must be Integer");
    txBuf[i] = mrb_integer(elem);
  }

  uint8_t writeAddress = (address << 1);

  // Reset, then apply fast: true from the options hash, if applicable.
  i2c_bb_reset(scl, sda);
  if (mrb_hash_p(options)) {
    mrb_value fast = mrb_hash_get(mrb, options, mrb_symbol_value(mrb_intern_lit(mrb, "fast")));
    if (mrb_bool(fast)) i2c_bb_slow = 0;
  }

  i2c_bb_start(scl, sda);
  i2c_bb_write_byte(scl, sda, writeAddress);
  for (int i=0; i<length; i++) i2c_bb_write_byte(scl, sda, txBuf[i]);
  i2c_bb_stop(scl, sda);
}

/****************************************************************************/
/*                            BIT-BANG SPI                                  */
/****************************************************************************/
uint8_t spi_bb_xfer_byte(int sck, int sdo, int sdi, int mode, int bitOrder, uint8_t data) {
  uint8_t b = 0x00;
  uint8_t bitPos;

  for (int i=0; i<8; i++) {
    // 0 is LSBFIRST, anything else is MSBFIRST.
    bitPos = (bitOrder == 0) ? i : 7-i;

    // SPI MODE 0
    if (mode == 0){
      if (sdo >= 0) digitalWrite(sdo, (data >> bitPos) & 0b1);
      digitalWrite(sck, HIGH);
      if (sdi >= 0) b |= (digitalRead(sdi) << bitPos);
      digitalWrite(sck, LOW);
    }

    // SPI MODE 1
    if (mode == 1){
      digitalWrite(sck, HIGH);
      if (sdo >= 0) digitalWrite(sdo, (data >> bitPos) & 0b1);
      digitalWrite(sck, LOW);
      if (sdi >= 0) b |= (digitalRead(sdi) << bitPos);
    }

    // SPI MODE 2
    if (mode == 2){
      if (sdo >= 0) digitalWrite(sdo, (data >> bitPos) & 0b1);
      digitalWrite(sck, LOW);
      if (sdi >= 0) b |= (digitalRead(sdi) << bitPos);
      digitalWrite(sck, HIGH);
    }

    // SPI MODE 3
    if (mode == 3){
      digitalWrite(sck, LOW);
      if (sdo >= 0) digitalWrite(sdo, (data >> bitPos) & 0b1);
      digitalWrite(sck, HIGH);
      if (sdi >= 0) b |= (digitalRead(sdi) << bitPos);
    }
  }
  return b;
}

mrb_value
mrb_spi_bb_xfer(mrb_state* mrb, mrb_value self) {
  mrb_int sck, sdo, sdi, cs, mode, bitOrder, rxLength;
  mrb_value txArray;
  mrb_get_args(mrb, "iiiiiiiA", &sck, &sdo, &sdi, &cs, &mode, &bitOrder, &rxLength, &txArray);
  if (!mrb_array_p(txArray)) mrb_raise(mrb, E_TYPE_ERROR, "SPI bytes must be given as Array");
  if ((mode < 0) || (mode > 3)) mrb_raise(mrb, E_TYPE_ERROR, "Invalid SPI mode. Must be in range 0..3");
  if ((cs >= 0) && (!wiringXValidGPIO(cs))) mrb_raise(mrb, E_TYPE_ERROR, "Invalid GPIO given for SPI chip select");

  // Reading more than writing, or writing more than reading?
  mrb_int txLength = RARRAY_LEN(txArray);
  int length = (rxLength > txLength) ? rxLength : txLength;
  uint8_t txBuf[length];
  uint8_t rxBuf[length];

  // Copy bytes from txArray into txBuf.
  for (int i=0; i<txLength; i++) {
    mrb_value elem = mrb_ary_ref(mrb, txArray, i);
    if (!mrb_integer_p(elem)) mrb_raise(mrb, E_TYPE_ERROR, "Each SPI byte must be Integer");
    txBuf[i] = mrb_integer(elem);
  }
  // Extend with 0s if needed.
  if (length > txLength) {
    for(int i=txLength; i<length; i++) txBuf[i] = 0;
  }

  // Pin Setup
  pinMode(sck, PINMODE_OUTPUT);
  if ((mode == 0)||(mode == 1)) digitalWrite(sck, LOW);
  if ((mode == 2)||(mode == 3)) digitalWrite(sck, HIGH);
  if (sdo >= 0) pinMode(sdo, PINMODE_OUTPUT);
  if (sdi >= 0) pinMode(sdi, PINMODE_INPUT);

  // Pull select low if needed.
  if (cs >= 0) {
    pinMode(cs, PINMODE_OUTPUT);
    digitalWrite(cs, LOW);
  }

  // Do the transfer.
  for (int i=0; i<length; i++) {
    rxBuf[i] = spi_bb_xfer_byte(sck, sdo, sdi, mode, bitOrder, txBuf[i]);
  }

  // Leave select high if needed.
  if (cs >= 0) digitalWrite(cs, HIGH);

  // Convert read bytes to mrb_ary and return.
  mrb_value rxArray = mrb_ary_new_capa(mrb, rxLength);
  for (int i=0; i<rxLength; i++) mrb_ary_push(mrb, rxArray, mrb_fixnum_value(rxBuf[i]));
  return rxArray;
}

/****************************************************************************/
/*                               GEM INIT                                   */
/****************************************************************************/
void
mrb_mruby_milkv_duo_gem_init(mrb_state* mrb) {
  // Module
  struct RClass *topMod = mrb_define_module(mrb, "Duo");

  // Constants
  mrb_define_const(mrb, topMod, "PINMODE_NOT_SET",   mrb_fixnum_value(PINMODE_NOT_SET));
  mrb_define_const(mrb, topMod, "PINMODE_INPUT",     mrb_fixnum_value(PINMODE_INPUT));
  mrb_define_const(mrb, topMod, "PINMODE_OUTPUT",    mrb_fixnum_value(PINMODE_OUTPUT));
  mrb_define_const(mrb, topMod, "PINMODE_INTERRUPT", mrb_fixnum_value(PINMODE_INTERRUPT));
  mrb_define_const(mrb, topMod, "LOW",               mrb_fixnum_value(LOW));
  mrb_define_const(mrb, topMod, "HIGH",              mrb_fixnum_value(HIGH));

  // Class Methods
  mrb_wx_setup(mrb, mrb_nil_value()); // Save user from calling Duo.setup each script.
  mrb_define_module_function(mrb, topMod, "micro_delay",         mrb_micro_delay,        MRB_ARGS_REQ(1));
  mrb_define_module_function(mrb, topMod, "setup",               mrb_wx_setup,           MRB_ARGS_REQ(0));

  // Digital I/O
  mrb_define_module_function(mrb, topMod, "valid_gpio",          mrb_valid_gpio,         MRB_ARGS_REQ(1));
  mrb_define_module_function(mrb, topMod, "pin_mode",            mrb_pin_mode,           MRB_ARGS_REQ(2));
  mrb_define_module_function(mrb, topMod, "digital_write",       mrb_digital_write,      MRB_ARGS_REQ(2));
  mrb_define_module_function(mrb, topMod, "digital_read",        mrb_digital_read,       MRB_ARGS_REQ(1));

  // GPIO Alerts
  mrb_define_module_function(mrb, topMod, "claim_alert",         mrb_claim_alert,        MRB_ARGS_REQ(1));
  mrb_define_module_function(mrb, topMod, "stop_alert",          mrb_stop_alert,         MRB_ARGS_REQ(1));
  mrb_define_module_function(mrb, topMod, "get_alert",           mrb_get_alert,          MRB_ARGS_REQ(0));

  // PWM
  mrb_define_module_function(mrb, topMod, "pwm_enable",          mrb_pwm_enable,         MRB_ARGS_REQ(2));
  mrb_define_module_function(mrb, topMod, "pwm_set_polarity",    mrb_pwm_set_polarity,   MRB_ARGS_REQ(2));
  mrb_define_module_function(mrb, topMod, "pwm_set_period",      mrb_pwm_set_period,     MRB_ARGS_REQ(2));
  mrb_define_module_function(mrb, topMod, "pwm_set_duty",        mrb_pwm_set_duty,       MRB_ARGS_REQ(2));
  mrb_define_module_function(mrb, topMod, "tx_wave_ook",         mrb_tx_wave_ook,        MRB_ARGS_REQ(3));

  // SARADC
  mrb_define_module_function(mrb, topMod, "analog_read",         mrb_analog_read,        MRB_ARGS_REQ(1));

  // I2C
  mrb_define_module_function(mrb, topMod, "i2c_setup",           mrb_i2c_setup,          MRB_ARGS_REQ(2));
  mrb_define_module_function(mrb, topMod, "i2c_write",           mrb_i2c_write,          MRB_ARGS_REQ(2));
  mrb_define_module_function(mrb, topMod, "i2c_read",            mrb_i2c_read,           MRB_ARGS_REQ(2));

  // SPI
  mrb_define_module_function(mrb, topMod, "spi_setup",           mrb_spi_setup,          MRB_ARGS_REQ(2));
  mrb_define_module_function(mrb, topMod, "spi_xfer",            mrb_spi_xfer,           MRB_ARGS_REQ(3));
  mrb_define_module_function(mrb, topMod, "spi_ws2812_write",    mrb_spi_ws2812_write,   MRB_ARGS_REQ(2));

  // Bit-Bang Pulse Input
  mrb_define_module_function(mrb, topMod, "read_ultrasonic",     mrb_read_ultrasonic,    MRB_ARGS_REQ(3));
  mrb_define_module_function(mrb, topMod, "read_pulses_us",      mrb_read_pulses_us,     MRB_ARGS_REQ(3));

  // Bit-bang 1-Wire Helpers
  mrb_define_module_function(mrb, topMod, "one_wire_bit_read",   mrb_one_wire_bit_read,  MRB_ARGS_REQ(1));
  mrb_define_module_function(mrb, topMod, "one_wire_bit_write",  mrb_one_wire_bit_write, MRB_ARGS_REQ(2));
  mrb_define_module_function(mrb, topMod, "one_wire_reset",      mrb_one_wire_reset,     MRB_ARGS_REQ(1));

  // Bit-bang I2C
  mrb_define_module_function(mrb, topMod, "i2c_bb_setup",        mrb_i2c_bb_setup,       MRB_ARGS_REQ(2));
  mrb_define_module_function(mrb, topMod, "i2c_bb_search",       mrb_i2c_bb_search,      MRB_ARGS_REQ(2));
  mrb_define_module_function(mrb, topMod, "i2c_bb_read",         mrb_i2c_bb_read,        MRB_ARGS_REQ(4));
  mrb_define_module_function(mrb, topMod, "i2c_bb_write",        mrb_i2c_bb_write,       MRB_ARGS_REQ(4));

  // Bit-bang SPI
  mrb_define_module_function(mrb, topMod, "spi_bb_xfer",         mrb_spi_bb_xfer,        MRB_ARGS_REQ(8));
}

void
mrb_mruby_milkv_duo_gem_final(mrb_state* mrb) {
}
