# Top-level include Duo, to call its class methods directly.
include Duo

INIT_ARRAY  = [0, 168, 63, 211, 0, 64, 161, 200, 218, 18, 164, 166, 213, 128, 219, 32, 217, 241, 141, 20, 32, 0, 175]
START_ARRAY = [0, 33, 0, 127, 34, 0, 7]
PATTERN_1   = [64] + Array.new(1024) { 0b00110011 }
PATTERN_2   = [64] + Array.new(1024) { 0b11001100 }

SCL = 16
SDA = 17
ADDRESS = 0x3C

i2c_bb_setup(SCL, SDA)
i2c_bb_write(SCL, SDA, ADDRESS, INIT_ARRAY)

FRAME_COUNT = 400

start = Time.now
(FRAME_COUNT / 2).times do
  i2c_bb_write(SCL, SDA, ADDRESS, START_ARRAY, fast: true)
  i2c_bb_write(SCL, SDA, ADDRESS, PATTERN_1, fast: true)
  i2c_bb_write(SCL, SDA, ADDRESS, START_ARRAY, fast: true)
  i2c_bb_write(SCL, SDA, ADDRESS, PATTERN_2, fast: true)
end
finish = Time.now

fps = FRAME_COUNT / (finish - start)
puts "SSD1306 benchmark result: #{fps.round(2)} fps"
