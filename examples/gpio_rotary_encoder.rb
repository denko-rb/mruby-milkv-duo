#
# Demo of a simple 30-detent rotary encoder.
# PIN_A = CLK/CLOCK, PIN_B = DT/DATA, PIN_SW = SWITCH
#
include WiringX

PIN_A     = 14
PIN_B     = 15
PIN_SW    = 16

[PIN_A, PIN_B, PIN_SW].each do |pin|
  raise "Errror, pin: #{pin} is not a valid GPIO" unless valid_gpio(pin) == 0
  pin_mode(pin, PINMODE_INPUT)
  claim_alert(pin)
end

# Encoder state
position = 0
state_a  = 0
state_b  = 0

# Get alerts to update state.
loop do
  alert = WiringX.get_alert
  if alert
    if alert[:pin] == PIN_A
      # Half quadrature, so we count every detent.
      state_a = alert[:level]
    elsif alert[:pin] == PIN_B
      delta = (alert[:level] == state_a) ? -1 : 1
      position += delta
      state_b = alert[:level]
      puts "Position: #{position}"
    elsif alert[:pin] == PIN_SW
      position = 0
      puts "Position: 0"
    end
  else
    sleep(0.005)
  end
end
