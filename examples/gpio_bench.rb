# Call WiringX.wiringx_setup before all GPIO access.
include WiringX
wiringx_setup

# LED pin on duo and duo_256M. Change to 0 for duos.
PIN     = 25
TOGGLES = 1_000_000

pin_mode(PIN, PINMODE_OUTPUT)

t1 = Time.now
TOGGLES.times do
  digital_write(PIN, 1)
  digital_write(PIN, 0)
end
t2 = Time.now

puts "Toggles per second: #{(TOGGLES/(t2-t1)).round}"
