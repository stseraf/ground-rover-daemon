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

# LTE=stub (default, no deps) or LTE=usb (USB modem via sysfs + HTTP API, no external libs)
LTE     ?= stub
ifeq ($(LTE),usb)
  CXXFLAGS += -DLTE_USB
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

ifeq ($(LTE),usb)
  SRCS += src/lte/usb_lte_monitor.cpp
endif

HEADERS = $(wildcard include/*.hpp) \
          $(wildcard src/*.hpp) \
          $(wildcard src/mavlink/*.hpp) \
          $(wildcard src/drive/*.hpp) \
          $(wildcard src/motor/*.hpp) \
          $(wildcard src/gimbal/*.hpp) \
          $(wildcard src/gps/*.hpp) \
          $(wildcard src/lte/*.hpp) \
          $(wildcard include/gps_fix.hpp) \
          $(wildcard include/lte_status.hpp)

TARGET = build/ground_rover_daemon

all: build $(TARGET)

build:
	mkdir -p build

$(TARGET): $(SRCS) $(HEADERS)
	@echo "  CXX  $@  [ARCH=$(ARCH) DRIVER=$(DRIVER) GIMBAL=$(GIMBAL) GPS=$(GPS) LTE=$(LTE)]"
	$(CXX) $(CXXFLAGS) $(SRCS) -o $(TARGET) $(INC) $(LDFLAGS)

rebuild: clean all

# Deploy to RPi: make deploy [RPI=pi@pi-rover.lan]
RPI ?= pi@pi-rover.lan
deploy:
	$(MAKE) rebuild ARCH=rpi DRIVER=tb6612 GPS=nmea LTE=usb
	ssh $(RPI) "mkdir -p /home/pi/ground-rover-daemon && sudo systemctl stop ground-rover-daemon || true"
	scp $(TARGET) $(RPI):/home/pi/ground-rover-daemon/
	scp deploy/ground-rover-daemon.service $(RPI):/tmp/
	ssh $(RPI) "sudo mv /tmp/ground-rover-daemon.service /etc/systemd/system/ && sudo systemctl daemon-reload && sudo systemctl enable --now ground-rover-daemon"

# Deploy modem config to a fresh/replacement UZ801 dongle via ADB.
# Copies scripts to the Pi and runs deploy-modem.sh there (adb is on the Pi).
# Usage: make deploy-modem [RPI=pi@pi-rover.lan]
deploy-modem:
	ssh $(RPI) "mkdir -p /tmp/modem-deploy"
	scp deploy/modem/nat_forward.sh deploy/modem/led_status.sh deploy/modem/deploy-modem.sh \
	    $(RPI):/tmp/modem-deploy/
	ssh $(RPI) "bash /tmp/modem-deploy/deploy-modem.sh"

clean:
	rm -rf build

.PHONY: all build rebuild deploy deploy-modem clean
