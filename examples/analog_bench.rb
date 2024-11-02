include Duo

PIN   = 26
READS = 100_000

Duo.saradc_initialize

t1 = Time.now
READS.times do
  Duo.analog_read(PIN)
end
t2 = Time.now

puts "Reads per second: #{(READS/(t2-t1)).round}"
