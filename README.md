# mruby-milkv-wiringx

**WARNING: INCOMPLETE AND EXPERIMENTAL**

This mrbgem is for the [Milk-V Duo](https://milkv.io/duo) series of single board computers. It uses the wiringX C library (from the Milk-V SDK) to provide GPIO functionality in mruby. So far, **only** these features have been implemented:

- Pin Mode
- Digital Write
- Digital Read

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
