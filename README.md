# mruby-milkv-duo

This mruby gem provides GPIO, ADC, PWM, I2C and SPI functionality for the [Milk-V Duo](https://milkv.io/duo) series of single board computers. It uses the wiringX C library (included in the Milk-V SDK), and some Linux `/dev` and `/sys` access. The interface aims to be similar to the [lgpio](https://github.com/denko-rb/lgpio) CRuby gem.

## Features

### `Duo` class methods

- [x] GPIO Mode/Read/Write
- [x] GPIO Input Alerts
  - Alerts are generated by a separate thread, and can be read from a queue in mruby.
  - **Does not** use interrupts, but polls pins every ~100 microseconds instead
- [x] PWM Output
- [x] Hardware I2C
- [x] Ultrasonic Read (HC-SR04)
- [x] Pulse Sequence Read (DHT enviro sensors)
- [x] Bit Bang I2C
- [ ] Bit Bang SPI
- [x] Bit Bang 1-Wire
- [x] WS2812 addressable LEDs over Hardware SPI
  - Only outputs on SPI2 MOSI pin
  - SPI clock must be set to 2.4 MHz
- [x] On-off Keying (OOK) Modulated Waves (based on Hardware PWM)
  - Carrier generated by hardware PWM. Software modulated with monotonic clock timing.

### Classes

- [x] `Duo::HardwarePWM` (convenience wrapper for PWM methods)
- [x] `Duo::PositionalServo` (based on Hardware PWM)
- [x] `Duo::Infrared` (based on OOK waves)
  - Default frequency: 38 kHz. Default duty cycle: 33.33%
- [X] `Duo::AnalogInput` (based on sysfs SARADC interface)
  - On GPIO 26 and 27 only
  - Always 12-bit resolution (0-4095 range)
  - Full scale voltage is only 1.8V, **not** 3.3V.

### Pinmux

- Some features of the Duo are mulltiplexed onto the same pins
- Use `duo-pinmux` to set them up BEFORE using them. See [opficial docs](https://milkv.io/docs/duo/application-development/pinmux) for more info.
- Run `mruby pinmux_custom.rb` from the examples folder, to get the pinmux layout used in all examples

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
  - Uncomment `conf.gem :github => 'denko-rb/mruby-milkv-duo'` (last line), to enable this gem

- Cross-compile mruby with: `rake MRUBY_CONFIG=build_config/milkv_duo.rb`
- Connect your board (with SD card running the official image), and passthrough to your virtual machine if needed
- Copy the compiled Milk-V binaries to the SD card, using scp:

```console
scp -O build/milkv_duo/bin/* root@192.168.42.1:/usr/local/bin
```

- SSH into the board: `ssh root@192.168.42.1` (default password: milkv)
- Try the `mirb` shell, or check out the the examples in [this](examples) folder.
