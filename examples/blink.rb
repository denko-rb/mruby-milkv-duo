# Top-level include Duo, to call its class methods directly.
include Duo

# 25 is the on-board LED.
PIN   = 25
DELAY = 0.5
pin_mode(PIN, PINMODE_OUTPUT)

loop do
  digital_write(PIN, 1)
  sleep(DELAY)
  digital_write(PIN, 0)
  sleep(DELAY)
end
