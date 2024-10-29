# Top-level include Duo, to call its class methods directly.
include Duo

PIN = 15
raise "Error: pin #{PIN} is not a valid GPIO" unless valid_gpio(PIN)
pin_mode(PIN, PINMODE_INPUT)

old_state = digital_read(PIN)
old_state = old_state ^ 0b1

loop do
  state = digital_read(PIN)
  if (state != old_state)
    print "\rGPIO #{PIN} state: #{state}"
    old_state = state
  end
  sleep 0.001
end
