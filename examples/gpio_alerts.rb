# Top-level include WiringX, to call its class methods directly.
include WiringX

# Poll these 8 pins, in a separate thread, every ~100us.
PINS = (14..21).to_a

PINS.each do |pin|
  raise "Errror, pin: #{pin} is not a valid GPIO" unless valid_gpio(pin) == 0
  pin_mode(pin, PINMODE_INPUT)
  claim_alert(pin)
end

# Keep reading alerts from the queue and printing them.
loop do
  alert = get_alert
  alert ? puts(alert) : sleep(0.010)
end
