include WiringX

a1 = WiringX::AnalogInput.new(26)
a2 = WiringX::AnalogInput.new(27)

loop do
  puts "GPIO26: #{a1.read}"
  puts "GPIO27: #{a2.read}"
  sleep 1
end
