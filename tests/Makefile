# gcc -DF_CPU=10000000 -D__AVR_ATtiny3217__=attiny3217 -DTEST -DDEBUG -std=c++11 -Wno-format  -I ./mocks -I ../common -o test  ../common/uart.cpp  ./delay.cpp ./main.cpp ./misc.cpp ./test.cpp ./tests_uart.cpp && ./test

PRJ        = test
CLK        = 10000000

SRC        = .
COMMONDIR  = ../common
COMMON     = uart.cpp
MOCKS      = ./mocks

FLAGS      = -DTEST -DDEBUG -std=c++11 -Wno-format # ignore lu warnings as different sizes
CPPFLAGS   =

CFLAGS     = -DF_CPU=$(CLK) -D__AVR_ATtiny3217__=attiny3217 $(FLAGS) -I $(MOCKS) -I $(COMMONDIR)

# ********************************************************

CC         = gcc

# CFILES    := $(foreach dir, $(COMMON) $(SRC), $(wildcard $(dir)/*.c))
# CPPFILES  := $(foreach dir, $(COMMON) $(SRC), $(wildcard $(dir)/*.cpp))
CPPFILES  := $(foreach dir, $(SRC), $(wildcard $(dir)/*.cpp))

# objects from c files
all:
	@$(CC) $(CFLAGS) -o $(PRJ) $(patsubst %, ../common/%, $(COMMON)) $(CPPFILES) && ./$(PRJ)
