### INSTALLATION
```
git clone --recurse-submodules git@github.com:2ni/attiny-boilerplate.git
pyenv virtualenv 3.10.2 <projectname>
pyenv local <projectname>
pip install -r requirements.txt
make patch # patch pymcuprg for our serial / writer

# install pymcuprog
cd src_python/pymcuprog
pip install -r requirements.txt
pip install -e src_python/pymcuprog  # to have proper paths

# install microchip toolchain
# see "install microchip toolchain" below
```

#### Manually update existing submodules:
Only needed if not using `-recurse-submodules` when cloning.
```
git submodule init
git submodule update
pip install -e src_python/pymcuprog # to have prope paths
```

#### INSTALLATION MICROCHIP TOOLCHAIN
- download the newest [toolchain](https://www.microchip.com/mplab/avr-support/avr-and-arm-toolchains-c-compilers) "AVR-8-bit Toolchain 3.7.0 (OSX)"from micro
chip
- download the newest [pack](http://packs.download.atmel.com/) from atmel (search for eg attiny3217 to get the correct pack)
```
mkdir toolchain_microchip
tar xfz avr8-gnu-toolchain-osx-3.7.0.518-darwin.any.x86_64.tar.gz -C toolchain_microchip
cd toolchain_microchip; mv avr8-gnu-toolchain-darwin_x86_64/* .; rm -rf avr8-gnu-toolchain-darwin_x86_64/
cd ..
mkdir toolchain_microchip/pack
unzip Atmel.ATtiny_DFP.2.0.368.atpack -d toolchain_microchip/pack/
ln -s toolchain_microchip/pack/include/avr/iotn3217.h .
```

### RUN
```
make check                                # check if you have connection to the device
./activate examples/blink                 # select the project you want to compile/flash 
make flash # compile and upload           # upload project to the device
make port=1 mcu=attiny3217 clk=10000000   # some options, mcu must be installed in the toolchain (appropriate pack)
```
