include WiringX

I2C_DEV     = 1
ADDRESS     = 0x3C
INIT_ARRAY  = [0, 168, 63, 211, 0, 64, 161, 200, 218, 18, 164, 166, 213, 128, 219, 32, 217, 241, 141, 20, 32, 0, 175]
START_ARRAY = [0, 33, 0, 127, 34, 0, 7]
PATTERN_1   = [64] + Array.new(1024) { 0b00110011 }
PATTERN_2   = [64] + Array.new(1024) { 0b11001100 }

# Open handle and setup.
ssd1306_handle = WiringX.i2c_setup(I2C_DEV, ADDRESS)
WiringX.i2c_write(ssd1306_handle, INIT_ARRAY)

FRAME_COUNT = 100

start = Time.now
(FRAME_COUNT / 2).times do
  WiringX.i2c_write(ssd1306_handle, START_ARRAY)
  WiringX.i2c_write(ssd1306_handle, PATTERN_1)
  WiringX.i2c_write(ssd1306_handle, START_ARRAY)
  WiringX.i2c_write(ssd1306_handle, PATTERN_2)
end
finish = Time.now

fps = FRAME_COUNT / (finish - start)
puts "SSD1306 benchmark result: #{fps.round(2)} fps"
