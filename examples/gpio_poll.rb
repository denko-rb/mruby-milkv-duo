# Call WiringX.wiringx_setup before all GPIO access.
include WiringX
wiringx_setup

PIN = 15

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
