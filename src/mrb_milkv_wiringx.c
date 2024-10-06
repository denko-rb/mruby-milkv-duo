#include <mruby.h>
#include <mruby/value.h>
#include <wiringx.h>

// Set MILKV_DUO_VARIANT in mruby build config.
// Prep isn't treating strings in -D flags as strings, so do this nonsense.
#define _milkv_duo      "milkv_duo"
#define _milkv_duo256m  "milkv_duo256m"
#define _milkv_duos     "milkv_duos"

static mrb_value
mrb_wiringx_setup(mrb_state* mrb, mrb_value self) {
  int result = wiringXSetup(MILKV_DUO_VARIANT, NULL);
  if (result == -1) {
    wiringXGC();
    mrb_raise(mrb, E_RUNTIME_ERROR, "Wiring X setup failed");
  }
  return mrb_nil_value();
}

static mrb_value
mrb_wiringx_pin_mode(mrb_state* mrb, mrb_value self) {
  mrb_int pin, mode;
  mrb_get_args(mrb, "ii", &pin, &mode);
  pinMode(pin, mode);
  return mrb_nil_value();
}

static mrb_value
mrb_wiringx_digital_write(mrb_state* mrb, mrb_value self) {
  mrb_int pin, state;
  mrb_get_args(mrb, "ii", &pin, &state);
  digitalWrite(pin, state);
  return mrb_nil_value();
}

static mrb_value
mrb_wiringx_digital_read(mrb_state* mrb, mrb_value self) {
  mrb_int pin, state;
  mrb_get_args(mrb, "i", &pin);
  state = digitalRead(pin);
  return mrb_fixnum_value(state);
}

void
mrb_mruby_milkv_wiringx_gem_init(mrb_state* mrb) {
  // Module
  struct RClass *mrb_WiringX = mrb_define_module(mrb, "WiringX");

  // Constants
  mrb_define_const(mrb, mrb_WiringX, "PINMODE_NOT_SET",   mrb_fixnum_value(PINMODE_NOT_SET));
  mrb_define_const(mrb, mrb_WiringX, "PINMODE_INPUT",     mrb_fixnum_value(PINMODE_INPUT));
  mrb_define_const(mrb, mrb_WiringX, "PINMODE_OUTPUT",    mrb_fixnum_value(PINMODE_OUTPUT));
  mrb_define_const(mrb, mrb_WiringX, "PINMODE_INTERRUPT", mrb_fixnum_value(PINMODE_INTERRUPT));
  mrb_define_const(mrb, mrb_WiringX, "PINMODE_INTERRUPT", mrb_fixnum_value(PINMODE_INTERRUPT));
  mrb_define_const(mrb, mrb_WiringX, "PINMODE_INTERRUPT", mrb_fixnum_value(PINMODE_INTERRUPT));
  mrb_define_const(mrb, mrb_WiringX, "LOW",               mrb_fixnum_value(LOW));
  mrb_define_const(mrb, mrb_WiringX, "HIGH",              mrb_fixnum_value(HIGH));

  // Class Methods
  mrb_define_method(mrb, mrb_WiringX, "wiringx_setup",  mrb_wiringx_setup,          MRB_ARGS_REQ(0));
  mrb_define_method(mrb, mrb_WiringX, "pin_mode",       mrb_wiringx_pin_mode,       MRB_ARGS_REQ(2));
  mrb_define_method(mrb, mrb_WiringX, "digital_write",  mrb_wiringx_digital_write,  MRB_ARGS_REQ(2));
  mrb_define_method(mrb, mrb_WiringX, "digital_read",   mrb_wiringx_digital_read,   MRB_ARGS_REQ(1));
}

void
mrb_mruby_milkv_wiringx_gem_final(mrb_state* mrb) {
}
