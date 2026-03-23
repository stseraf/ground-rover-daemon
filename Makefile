CXX      = g++
CXXFLAGS = -std=c++17 -O2 -Wall -Wextra
INC      = -Isrc -Iinclude -Iexternal/mavlink

# ARCH=host (default, x86, native g++) or ARCH=rpi (cross-compile for RPi Zero 2W)
ARCH    ?= host
ifeq ($(ARCH),rpi)
  CXX      = aarch64-linux-gnu-g++
  CXXFLAGS += -march=armv8-a
endif

# DRIVER=stub (default, no deps) or DRIVER=tb6612 (sysfs GPIO + PWM, no external libs)
DRIVER  ?= stub
ifeq ($(DRIVER),tb6612)
  CXXFLAGS += -DDRIVER_TB6612
endif

SRCS = src/main.cpp \
       src/mavlink/mav_sender.cpp \
       src/mavlink/command_handlers.cpp \
       src/mavlink/camera_handlers.cpp

ifeq ($(DRIVER),tb6612)
  SRCS += src/motor/tb6612_driver.cpp
endif

HEADERS = $(wildcard include/*.hpp) \
          $(wildcard src/*.hpp) \
          $(wildcard src/mavlink/*.hpp) \
          $(wildcard src/drive/*.hpp) \
          $(wildcard src/motor/*.hpp)

TARGET = build/ground_rover_daemon

all: build $(TARGET)

build:
	mkdir -p build

$(TARGET): $(SRCS) $(HEADERS)
	@echo "  CXX  $@  [ARCH=$(ARCH) DRIVER=$(DRIVER)]"
	$(CXX) $(CXXFLAGS) $(SRCS) -o $(TARGET) $(INC) $(LDFLAGS)

rebuild: clean all

# Deploy to RPi: make deploy [RPI=pi@pi-rover.lan]
RPI ?= pi@pi-rover.lan
deploy:
	$(MAKE) rebuild ARCH=rpi DRIVER=tb6612
	scp $(TARGET) $(RPI):/home/pi/.

clean:
	rm -rf build
