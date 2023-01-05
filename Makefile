# inspired by 
# - https://gist.github.com/holachek/3304890
# - https://gist.github.com/mcous/5920089
#
# .elf needed on host for debugging
# .hex is code for programming to flash
#
# export PATH=/www/forgetmenot/toolchain/bin:$PATH
#
# run "make FLAGS=" to not run in debug mode
#
# example:
# make mcu=attiny1627 common="uart aes" port=3 clk=5000000
#
.PHONY: all clean tests

PRJ        = main
DEVICE     = $(shell if [ ! -z $(mcu) ]; then echo $(mcu); else echo attiny3217; fi)
# default prescaler is 6 -> 3.3MHz
# max frequency is 13.3MHz for 3.3v (see p.554)
# set to 10MHz with cmd: _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PDIV_2X_gc);
CLK        = $(shell if [ ! -z $(clk) ]; then echo $(clk); else echo 10000000; fi)

DEBUG      = $(shell if [ ! -z $(nodebug) ]; then echo ""; else echo "-DDEBUG"; fi)

SRC        = ./src
EXT        =
COMMON     = ./common
EXCLUDE    = timer.cpp

FLAGS      = $(DEBUG) -std=c++11 # avoid warning "non-static data member initializers"
CPPFLAGS   =

# toolchain_microchip
# -Wl,-gc-sections is used to not include unused code in binary
# https://stackoverflow.com/questions/14737641/have-linker-remove-unused-object-files-for-avr-gcc
# https://www.mikrocontroller.net/articles/GCC:_unbenutzte_Funktionen_entfernen

BIN        = ./toolchain_microchip/bin/
LIB        = ./toolchain_microchip/avr/include/
CFLAGS     = -Wall -Wl,-gc-sections -ffunction-sections -fdata-sections -Os -DF_CPU=$(CLK) -mmcu=$(DEVICE) -B toolchain_microchip/pack/gcc/dev/$(DEVICE)/ -I toolchain_microchip/pack/include/ -I $(LIB) $(FLAGS) -I $(COMMON)

# ********************************************************

# executables
#
# PORT takes 1st in the list if not given
# enforce port with <cmd> port=<1|2|3|4>
#
PORT       = $(shell if [ ! -z $(port) ]; then ls /dev/cu.usbserial* |grep $(port)0; else ls -t /dev/cu.usbserial*|head -1; fi)
PYPRG      = pymcuprog -t uart -d $(DEVICE) -u $(PORT) -b 230400
OBJCOPY    = $(BIN)avr-objcopy
OBJDUMP    = $(BIN)avr-objdump
SIZE       = $(BIN)avr-objdump -Pmem-usage
CC         = $(BIN)avr-gcc
MEMORY     = $(BIN)avr-size

# objects
# CFILES     = $(wildcard $(SRC)/*.c)
CFILES    := $(foreach dir, $(COMMON) $(SRC), $(wildcard $(dir)/*.c))
EXTC      := $(foreach dir, $(EXT), $(wildcard $(dir)/*.c))
CPPFILES  := $(foreach dir, $(SRC), $(wildcard $(dir)/*.cpp))
CPPFILES  := $(CPPFILES) $(shell if [ ! -z "$(common)" ]; then echo $(foreach c, $(common), $(COMMON)/$(c).cpp); else echo $(foreach dir, $(COMMON), $(filter-out $(foreach file, $(EXCLUDE), $(dir)/$(file)), $(wildcard $(dir)/*.cpp))); fi)
# CPPFILES  := $(foreach dir, $(COMMON) $(SRC), $(filter-out $(foreach file, $(EXCLUDE), $(dir)/$(file)), $(wildcard $(dir)/*.cpp)))
EXTCPP    := $(foreach dir, $(EXT), $(wildcard $(dir)/*.cpp))
OBJ        = $(CFILES:.c=.c.o) $(EXTC:.c=.c.o) $(CPPFILES:.cpp=.cpp.o) $(EXTCPP:.cpp=.cpp.o)
DEPENDS   := $(CFILES:.c=.d) $(EXTC:.c=.d) $(CPPFILES:.cpp=.cpp.d) $(EXTCPP:.cpp=.cpp.d)

# user target
# compile all files
all: 	$(PRJ).hex tests

clean:
	rm -f *.hex *.elf
	@# rm -f $(OBJ) $(DEPENDS)
	rm -f $(SRC)/*.{d,o}
	rm -f $(COMMON)/*.{d,o}

# objects from c files
%.c.o: %.c Makefile
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

# objects from c++ files
%.cpp.o: %.cpp Makefile
	$(CC) $(CFLAGS) $(CPPFLAGS) -MMD -MP -c $< -o $@

# linking, create elf
$(PRJ).elf: $(OBJ) Makefile
	$(CC) $(CFLAGS) -o $(PRJ).elf $(OBJ)

# create hex
$(PRJ).hex: $(PRJ).elf Makefile
	$(OBJCOPY) -j .text -j .data -j .rodata -O ihex $(PRJ).elf $(PRJ).hex
	@echo ""
	@echo "***********************************************************"
	@$(SIZE) $(PRJ).elf | egrep "Program|Data|Device"
	@echo "***********************************************************"

-include $(DEPENDS)

# start debugging terminal
# make serial port=1 or port=/dev/cu.usbserial1410
serial:
	@./serialterminal.py -dp $(if $(port),$(port),$(PORT))

# show available uart ports for flashing and debugging terminal
ports:
	@echo "available ports:"
	@unset CLICOLOR &&  ls -1t /dev/cu.usbserial* && export CLICOLOR=1

# check programmer connectivity
check:
	$(PYPRG) ping

# flash hex to mcu
# port can be overruled with: make flash port=1
# mcu can be overruled with: make flash mcu=attiny1604
flash: all
	@echo "flashing to $(PORT). Pls wait..."
	@$(PYPRG) --erase write -f $(PRJ).hex && echo "done." && echo "\a" && $(MAKE) serial  port=$(PORT) # && afplay /System/Library/Sounds/Ping.aiff -v 10

# reset device
reset:
	@$(PYPRG) reset && echo "done" && echo "\a" && $(MAKE) serial port=$(PORT) # && afplay /System/Library/Sounds/Ping.aiff -v 20

resetonly:
	@$(PYPRG) reset

# generate disassembly files for debugging
disasm: $(PRJ).elf
	#$(OBJDUMP) -d $(PRJ).elf
	$(OBJDUMP) -S $(PRJ).elf > $(PRJ).asm
	$(OBJDUMP) -t $(PRJ).elf > $(PRJ).sym
	$(OBJDUMP) -t $(PRJ).elf |grep 00 |sort  > $(PRJ)-sorted.sym

tests:
	@$(MAKE) -C tests

# show detailed memory usage
memory: $(PRJ).elf
	$(MEMORY) -A main.elf
patch:
	@cd ./src_python/pymcuprog/ && git apply ../../pymcuprog.patch
	@echo "DONE. Don't forget to run pip install -e src_python/pymcuprog"
