include Duo

PIN = 10

servo = Duo::PositionalServo.new(PIN, 500, 2500, 0, 180)

angles = [0, 30, 60, 90, 120, 150, 180, 150, 120, 90, 60, 30]

angles.cycle do |angle|
  servo.angle = angle
  sleep 1
end
