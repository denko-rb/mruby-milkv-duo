#
# This is a bad example, recreating blink.rb with a 500k us delay,
# instead of a 0.5s sleep. DON'T do this. It's more CPU intensive.
# Only use Wiringx.micro_delay if more precise timing is required.
#
include WiringX

PIN     = 25
pin_mode(PIN, PINMODE_OUTPUT)

loop do
  digital_write(PIN, 1)
  micro_delay(500_000)
  digital_write(PIN, 0)
  micro_delay(500_000)
end
