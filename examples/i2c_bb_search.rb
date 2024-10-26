include WiringX

SCL = 16
SDA = 17

i2c_bb_setup(SCL, SDA)
devices = i2c_bb_search(SCL, SDA)

if devices.empty?
  puts "No devices found on I2C bus"
else
  puts "I2C device addresses found:"
  devices.each do |address|
    # Print as hexadecimal.
    puts "0x#{address.to_s(16).upcase}"
  end
end
