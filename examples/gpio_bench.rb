# Top-level include Duo, to call its class methods directly.
include Duo

TOGGLES = 1_000_000

# LED pin on duo and duo_256M. Change to 0 for duos.
PIN = 25
raise "Error: pin #{PIN} is not a valid GPIO" unless valid_gpio(PIN)
pin_mode(PIN, PINMODE_OUTPUT)

t1 = Time.now
TOGGLES.times do
  digital_write(PIN, 1)
  digital_write(PIN, 0)
end
t2 = Time.now

puts "Toggles per second: #{(TOGGLES/(t2-t1)).round}"
