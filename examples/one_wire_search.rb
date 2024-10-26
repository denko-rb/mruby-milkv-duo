include WiringX

PIN = 15
one_wire = WiringX::OneWire.new(PIN)
one_wire.search

puts; puts "Found these 1-wire addresss (HEX) on the bus:"; puts

one_wire.found_addresses.each do |address|
  puts address.to_s(16)
end
puts
