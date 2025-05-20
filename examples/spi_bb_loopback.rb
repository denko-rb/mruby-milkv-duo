include Duo

SCK = 16
SDO = 14
SDI = 15

MODES       = [0, 1, 2, 3]
# 0 is LSBFIRST. 1 is MSBFIRST (default).
ORDERS      = [0, 1]
TX_BYTES    = [0, 1, 2, 3, 4, 5, 6, 7]

puts "TX bytes => #{TX_BYTES.inspect}"

# Connect (loop back) SDI to SDO to see received bytes.
ORDERS.each do |order|
  MODES.each do |mode|
    # Arg order: sck, sdo, sdi, cs, mode, bit_order, write_bytes, read_length
    # sdo, sdi, and cs may all be unused. Give -1 in each case.
    rx_bytes = spi_bb_xfer(SCK, SDO, SDI, -1, mode, order, TX_BYTES, TX_BYTES.length)
    puts "RX (order: #{(order == 0) ? 'lsbfirst' : 'msbfirst'}, mode: #{mode}) => #{rx_bytes.inspect}"
  end
end
