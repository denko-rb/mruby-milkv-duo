include WiringX

I2C_DEV             = 1
ADDRESS             = 0x38
POWER_ON_DELAY      = 0.100
RESET_DELAY         = 0.020
COMMAND_DELAY       = 0.010
MEASURE_DELAY       = 0.080
DATA_LENGTH         = 6
SOFT_RESET          = [0xBA]
INIT_AND_CALIBRATE  = [0xE1, 0x08, 0x00]
START_MEASUREMENT   = [0xAC, 0x33, 0x00]

aht10_handle = WiringX.i2c_setup(I2C_DEV, ADDRESS)

# Startup sequence
sleep(POWER_ON_DELAY)
WiringX.i2c_write(aht10_handle, SOFT_RESET)
sleep(RESET_DELAY)
WiringX.i2c_write(aht10_handle, INIT_AND_CALIBRATE)
sleep(COMMAND_DELAY)

# Read and close
WiringX.i2c_write(aht10_handle, START_MEASUREMENT)
sleep(MEASURE_DELAY)
bytes = WiringX.i2c_read(aht10_handle, DATA_LENGTH)

# Humidity uses the upper 4 bits of the shared byte as its lowest 4 bits.
h_raw = ((bytes[1] << 16) | (bytes[2] << 8) | (bytes[3])) >> 4
humidity = (h_raw.to_f / 2**20) * 100

# Temperature uses the lower 4 bits of the shared byte as its highest 4 bits.
t_raw = ((bytes[3] & 0x0F) << 16) | (bytes[4] << 8) | bytes[5]
temperature = (t_raw.to_f / 2**20) * 200 - 50

puts "Temperature: #{temperature.round(2)} \xC2\xB0C | Humidity: #{humidity.round(2)} %"
