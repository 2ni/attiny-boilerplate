## About the project
This is a template to work with the ATtiny series programmed by updi, such as ATtiny3217, ATtiny1604 or ATtiny1627. With this template you can work with any editor you want, I personnally use vim.

The [SETUP](SETUP.md) explains how to set up a new project including the toolchain from microchip to be able to compile your code for the mcu's. The advantage (or disadvantage) is, that you don't need to use the IDE from microchip (eg [Microchip Studio](https://www.microchip.com/en-us/tools-resources/develop/microchip-studio)).

I work mostly with the ATtiny3217, but also did some projects with ATtiny1604 and ATtiny1627. For this purpose I designed a [developer board](https://github.com/2ni/attiny3217-dev-board) and a programmer which is also a serial terminal. It's possible to flash the chip via the programmer and also to communicate via serial (tx and rx) by using only one usb port.

## Getting started
Have a look at the [SETUP](SETUP.md), but basically you can do the following:
- download the latest [toolchain](https://www.microchip.com/mplab/avr-support/avr-and-arm-toolchains-c-compilers), eg "AVR-8-bit Toolchain 3.7.0 (OSX) from microchip
- download the latest [pack](http://packs.download.atmel.com/) from Atmel, eg search for "attiny3217"
- create a new repo, eg my-awesome-attiny-project
- run the following commands to merge the template into your project:
```
mkdir my-awesome-attiny-project
cd my-awesome-attiny-project
git init
git remote add template https://github.com/2ni/attiny-boilerplate
git fetch --all
git diff ...template/main                                          # only if you 1st want to see the diffs
git merge template/main [--allow-unrelated-histories]              # allow-unrelated-histories is only needed on the 1st merge
git diff origin/main..HEAD                                         # only to see the changes before pushing
git submodule init                                                 # load submodule pymcuprog from microchip
git submodule update

pyenv virtualenv 3.10.2 my-awesome-attiny-project
pyenv local my-awesome-attiny-project
make patch                                                         # patch pymcuprog to use it with our programmer/serial board
pip install -r requirements                                        # install python dependencies, mainly for the serial terminal
pip install -r src_python/pymcuprog/requirements.txt               # install python dependencies for pymcuprog
pip install -e src_python/pymcuprog                                # install pymcuprog into our env to have proper paths

mkdir toolchain_microchip
tar xfz avr8-gnu-toolchain-osx-3.7.0.518-darwin.any.x86_64.tar.gz -C toolchain_microchip
cd toolchain_microchip; mv avr8-gnu-toolchain-darwin_x86_64/* .; rm -rf avr8-gnu-toolchain-darwin_x86_64/
cd ..
mkdir toolchain_microchip/pack
unzip Atmel.ATtiny_DFP.2.0.368.atpack -d toolchain_microchip/pack/
ln -s toolchain_microchip/pack/include/avr/iotn3217.h .
```

## Usage
You can now start coding. You can put your files directly into the folder src/ or you can load an example as follows:
```
./activate examples/blink                                         # loads a project into src/
make clean                                                        # when activating a new project, you need to have a clean setup
make [port=1] [mcu=attiny3217] [clk=10000000] flash               # if port not given, it tries to get it, mcu defaults to attiny3217
make [mcu=attiny3217] reset                                       # resets mcu
make [mcu=attiny3217] check                                       # checks the connection
```

## Some example projects
- [Blink](/examples/blink)
A simple led blinking. You can load it with ./activate examples/blink
- [DCC simulator](/examples/dcc-simulator)
Simulator to test eg turnouts/trackswitches or decoders for model railroads
- [Stepper](/examples/stepper)
Control a 4-wire stepper motor
