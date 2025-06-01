include Duo

SPI_DEV  = 0
SPI_BAUD = 1_000_000

tx_bytes = [0, 1, 2, 3, 4, 5, 6, 7]
puts "TX bytes: #{tx_bytes.inspect}"

# rx_bytes == tx_bytes if MOSI looped back to MISO.
# rx_byte all 255 when MOSI and MISO not connected.
rx_bytes = Duo.spi_xfer(SPI_DEV, SPI_BAUD, tx_bytes, tx_bytes.length)
puts "RX bytes: #{rx_bytes.inspect}"
