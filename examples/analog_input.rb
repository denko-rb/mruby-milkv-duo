include Duo

Duo.saradc_initialize

loop do
  puts "GPIO26: #{Duo.analog_read(26)}"
  puts "GPIO27: #{Duo.analog_read(27)}"
  sleep 1
end
