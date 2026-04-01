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

# GIMBAL=stub (default, no deps) or GIMBAL=i2c (Linux i2c-dev, no external libs)
GIMBAL  ?= stub
ifeq ($(GIMBAL),i2c)
  CXXFLAGS += -DGIMBAL_I2C
endif

# GPS=stub (default, no deps) or GPS=nmea (UART NMEA parser for GY-GPS6MV2 / NEO-6M)
GPS     ?= stub
ifeq ($(GPS),nmea)
  CXXFLAGS += -DGPS_NMEA
endif

SRCS = src/main.cpp \
       src/mavlink/mav_sender.cpp \
       src/mavlink/param_store.cpp \
       src/mavlink/command_handlers.cpp \
       src/mavlink/camera_handlers.cpp

ifeq ($(DRIVER),tb6612)
  SRCS += src/motor/tb6612_driver.cpp
endif

ifeq ($(GIMBAL),i2c)
  SRCS += src/gimbal/i2c_gimbal_controller.cpp
endif

ifeq ($(GPS),nmea)
  SRCS += src/gps/nmea_gps_provider.cpp
endif

HEADERS = $(wildcard include/*.hpp) \
          $(wildcard src/*.hpp) \
          $(wildcard src/mavlink/*.hpp) \
          $(wildcard src/drive/*.hpp) \
          $(wildcard src/motor/*.hpp) \
          $(wildcard src/gimbal/*.hpp) \
          $(wildcard src/gps/*.hpp) \
          $(wildcard include/gps_fix.hpp)

TARGET = build/ground_rover_daemon

all: build $(TARGET)

build:
	mkdir -p build

$(TARGET): $(SRCS) $(HEADERS)
	@echo "  CXX  $@  [ARCH=$(ARCH) DRIVER=$(DRIVER) GIMBAL=$(GIMBAL) GPS=$(GPS)]"
	$(CXX) $(CXXFLAGS) $(SRCS) -o $(TARGET) $(INC) $(LDFLAGS)

rebuild: clean all

# Deploy to RPi: make deploy [RPI=pi@pi-rover.lan]
RPI ?= pi@pi-rover.lan
deploy:
	$(MAKE) rebuild ARCH=rpi DRIVER=tb6612 GPS=nmea
	ssh $(RPI) "mkdir -p /home/pi/ground-rover-daemon && sudo systemctl stop ground-rover-daemon || true"
	scp $(TARGET) $(RPI):/home/pi/ground-rover-daemon/
	scp deploy/ground-rover-daemon.service $(RPI):/tmp/
	ssh $(RPI) "sudo mv /tmp/ground-rover-daemon.service /etc/systemd/system/ && sudo systemctl daemon-reload && sudo systemctl enable --now ground-rover-daemon"

clean:
	rm -rf build

.PHONY: all build rebuild deploy clean
