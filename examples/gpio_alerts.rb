# Top-level include Duo, to call its class methods directly.
include Duo

# Poll these 8 pins, in a separate thread, every ~100us.
PINS = (14..21).to_a

PINS.each do |pin|
  raise "Error: pin #{pin} is not a valid GPIO" unless valid_gpio(pin)
  pin_mode(pin, PINMODE_INPUT)
  claim_alert(pin)
end

# Keep reading alerts from the queue and printing them.
loop do
  alert = get_alert
  alert ? puts(alert) : sleep(0.010)
end
