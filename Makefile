CXX      = g++
CXXFLAGS = -std=c++17 -O2 -Wall -Wextra
INC      = -Iinc -Iexternal/mavlink

# ARCH=host (default, x86, native g++) or ARCH=rpi (cross-compile for RPi Zero 2W)
ARCH    ?= host
ifeq ($(ARCH),rpi)
  CXX      = aarch64-linux-gnu-g++
  CXXFLAGS += -march=armv8-a
  ifdef SYSROOT
    CXXFLAGS += --sysroot=$(SYSROOT)
    LDFLAGS  += --sysroot=$(SYSROOT)
  endif
endif

# DRIVER=stub (default, no deps) or DRIVER=tb6612 (requires libgpiod on target)
DRIVER  ?= stub
ifeq ($(DRIVER),tb6612)
  CXXFLAGS += -DDRIVER_TB6612
  LDFLAGS  += -lgpiod
endif

SRCS    = src/main.cpp
HEADERS = $(wildcard inc/*.hpp)
TARGET  = build/ground_rover_daemon

all: build $(TARGET)

build:
	mkdir -p build

$(TARGET): $(SRCS) $(HEADERS)
	@echo "  CXX  $@  [ARCH=$(ARCH) DRIVER=$(DRIVER)]"
	$(CXX) $(CXXFLAGS) $(SRCS) -o $(TARGET) $(INC) $(LDFLAGS)

rebuild: clean all

# Deploy to RPi: make deploy [RPI=pi@pi-rover.lan] [SYSROOT=/opt/rpi-sysroot]
RPI     ?= pi@pi-rover.lan
SYSROOT ?= /opt/rpi-sysroot
deploy:
	$(MAKE) rebuild ARCH=rpi DRIVER=tb6612 SYSROOT=$(SYSROOT)
	scp $(TARGET) $(RPI):/home/pi/.

clean:
	rm -rf build
