#
# Example showing an HC-S04 ultrasonic distance sensor.
#
# NOTE: Some versions of this sensor require 5V power to function properly.
# If using one of these, use a 5V to 3.3V level shifter between your board and
# the sensor, at least on the echo pin.
#
include Duo

TRIGGER_PIN     = 18
ECHO_PIN        = 19
SPEED_OF_SOUND  = 343.0

loop do
  # Arguments in order are:
  #   trigger pin, echo pin, trigger time (us)
  #
  # HC-SR04 uses 10 microseconds for trigger. Some others use 20us.
  #
  microseconds = Duo.read_ultrasonic(TRIGGER_PIN, ECHO_PIN, 10)

  if microseconds
    mm = (microseconds / 2000.0) * SPEED_OF_SOUND
    puts "Distance: #{mm.round} mm"
  else
    puts "Cound not read HC-SR04 sensor"
  end
  sleep 0.5
end
