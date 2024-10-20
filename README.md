# mruby-milkv-wiringx

This mrbgem is for the [Milk-V Duo](https://milkv.io/duo) series of single board computers. It uses the wiringX C library (from the Milk-V SDK) to provide GPIO functionality in mruby. Instead of directly mapping the WiringX functions, it aims to provide an interface similar to [lgpio](https://github.com/denko-rb/lgpio) for CRuby on Linux.

## Features

- [x] GPIO Mode/Read/Write
- [ ] GPIO Alerts
- [x] PWM Output
- [x] Hardware I2C, but simplified:
  - `.i2c_setup` works just like the corresponding C function
  - `.i2c_write` doesn't match any C function. Raw write of any length byte array.
  - `.i2c_read` doesn't match any C function. Raw read of any length byte array.
- [x] Hardware SPI
  - Main method is `.spi_xfer` instead of expected `.spi_data_rw`, matching the `lgpio` CRuby gem.
- [ ] WS2812 addressable LEDs over Hardware SPI
- [ ] Bit Bang I2C
- [ ] Bit Bang SPI
- [ ] Bit Bang 1-Wire


**Note:** Use `duo-pinmux` to enable alternate functions (PWM/I2C/SPI etc.) on specific pins BEFORE they are used in mruby. See [official docs](https://milkv.io/docs/duo/application-development/pinmux) for more info.

## Build Instructions
- Unless running Ubuntu, set up a virtual machine with Ubuntu 24.04, then install:

```console
sudo apt install wget git make gcc
```

- Clone mruby and duo-sdk, then enter the mruby directory:

```console
git clone https://github.com/mruby/mruby
git clone https://github.com/milkv-duo/duo-sdk
cd mruby
```

- In `mruby/build_config/milkv_duo.rb`:
  - Set `MILKV_DUO_VARIANT` to the string that matches your board
  - Uncomment `conf.gem :github => 'denko-rb/mruby-milkv-wiringx'` (last line), to enable this gem

- Cross-compile mruby with: `rake MRUBY_CONFIG=build_config/milkv_duo.rb`
- Connect your board (with SD card running the official image), and passthrough to your virtual machine if needed
- Copy the compiled Milk-V binaries to the SD card, using scp:

```console
scp -O build/milkv_duo/bin/* root@192.168.42.1:/usr/local/bin
```

- SSH into the board: `ssh root@192.168.42.1` (default password: milkv)
- Try the `mirb` shell, or try the examples included with this gem:

```console
mirb

# Copy these from examples folder first
mruby gpio_bench.rb
mruby gpio_poll.rb
```
