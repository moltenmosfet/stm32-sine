CC		= gcc
CPP	= g++
LD		= g++
CP		= cp
CFLAGS    = -std=c99 -ggdb -DSTM32F1 -I../include -I../libopeninv/include -I../libopencm3/include
CPPFLAGS    = -ggdb -DSTM32F1 -DCTRL_FOC=1 -DCTRL_SINE=0 -DCTRL=CTRL_FOC -I../include -I../libopeninv/include -I../libopencm3/include
LDFLAGS     = -g
BINARY		= test_sine
# Every test_*.cpp in this directory links automatically -- a hand-kept list
# here silently drops any test file someone forgets to add.
TESTOBJS	= $(patsubst %.cpp,%.o,$(wildcard test_*.cpp))
OBJS		= $(TESTOBJS) fu.o my_fp.o my_string.o params.o vehiclecontrol.o \
			  throttle.o sine_core.o temp_meas.o stub_canhardware.o \
			  stub_libopencm3.o picontroller.o foc.o \
			  canfilterpack.o stm32scheduler.o

# A static test function that is defined but never listed in REGISTER_TEST
# compiles fine and never runs; make that a build error (in test code only).
$(TESTOBJS): CPPFLAGS += -Wunused-function -Werror=unused-function
VPATH = ../src ../libopeninv/src

# Check if the variable GITHUB_RUN_NUMBER exists. When running on the github actions running, this
# variable is automatically available.
# Create a compiler define with the content of the variable. Or, if it does not exist, use replacement value 99999.
CPPFLAGS += $(shell \
    if [ -z "$$GITHUB_RUN_NUMBER" ]; then echo "-DGITHUB_RUN_NUMBER=0"; else echo "-DGITHUB_RUN_NUMBER=$$GITHUB_RUN_NUMBER"; fi )

all: $(BINARY)

$(BINARY): $(OBJS)
	$(LD) $(LDFLAGS) -o $(BINARY) $(OBJS)

%.o: ../%.cpp
	$(CPP) $(CPPFLAGS) -o $@ -c $<

%.o: ../%.c
	$(CC) $(CFLAGS) -o $@ -c $<

clean:
	rm -f $(OBJS) $(BINARY)
