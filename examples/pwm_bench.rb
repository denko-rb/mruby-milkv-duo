include Duo

PIN = 10
pwm_out = Duo::HardwarePWM.new(PIN, period: 20_000_000)

RUNS = 25_000
start = Time.now
RUNS.times do
  pwm_out.duty_us = 1000
end
finish = Time.now

wps = RUNS / (finish - start)
puts "Hardware PWM writes per second: #{wps.round(2)}"
