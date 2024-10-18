# Top-level include WiringX, to call its class methods directly.
include WiringX

# Connect external LED to GPIO2
PIN = 2
raise "Errror, pin: #{PIN} is not a valid GPIO" unless valid_gpio(PIN) == 0

steps = (0..100).to_a + (1..99).to_a.reverse

# Period and duty are set in nanoseconds.
pwm_set_period(PIN, 1000)
pwm_set_duty(PIN, 0)
pwm_set_polarity(PIN, 0)
pwm_enable(PIN, 1)

# Fade the LED up and down.
steps.cycle do |value|
  pwm_set_duty(PIN, 10*value)
  sleep 0.02
end
