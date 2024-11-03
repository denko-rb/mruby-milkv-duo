# Top-level include Duo, to call its class methods directly.
include Duo

saradc_initialize

loop do
  puts "GPIO26: #{analog_read(26)}"
  puts "GPIO27: #{analog_read(27)}"
  sleep 1
end
