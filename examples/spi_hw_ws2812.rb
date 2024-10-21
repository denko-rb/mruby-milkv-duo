#
# 2.4 MHz SPI method for writing WS2812 addressable LEDs.
# Based on: https://learn.adafruit.com/dma-driven-neopixels/overview
#
include WiringX

# SPI config
SPI_DEV  = 2
SPI_BAUD = 2_400_000

PIXEL_COUNT = 8
COLORS = [
  [255, 255, 255],
  [0, 255, 0],
  [255, 0, 0],
  [0, 0, 255],
  [255, 255, 0],
  [255, 0, 255],
  [0, 255, 255]
]

# Move along the strip and back, one pixel at a time.
POSITIONS = (0..PIXEL_COUNT-1).to_a + (1..PIXEL_COUNT-2).to_a.reverse

pixels = Array.new(PIXEL_COUNT) { [0, 0, 0] }
spi_handle = WiringX.spi_setup(SPI_DEV, SPI_BAUD)

loop do
  COLORS.each do |color|
    POSITIONS.each do |index|
      # Clear and write.
      (0..PIXEL_COUNT-1).each { |i| pixels[i] = [0, 0, 0] }
      WiringX.spi_ws2812_write(SPI_DEV, pixels.flatten)

      # Set one pixel and write.
      pixels[index] = color
      WiringX.spi_ws2812_write(SPI_DEV, pixels.flatten)

      sleep 0.025
    end
  end
end
