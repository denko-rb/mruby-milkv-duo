include Duo

a1 = Duo::AnalogInput.new(26)
a2 = Duo::AnalogInput.new(27)

loop do
  puts "GPIO26: #{a1.read}"
  puts "GPIO27: #{a2.read}"
  sleep 1
end
