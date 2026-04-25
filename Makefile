CXX      = g++
CXXFLAGS = -std=c++17 -O2 -Wall -Wextra
INC      = -Isrc -Iinclude -Iexternal/mavlink

# Git version info — surfaced to QGC UI via STATUSTEXT banner on first connect.
GIT_BRANCH := $(shell git rev-parse --abbrev-ref HEAD 2>/dev/null || echo unknown)
GIT_SHA    := $(shell git rev-parse --short HEAD 2>/dev/null || echo unknown)
CXXFLAGS  += -DGIT_BRANCH=\"$(GIT_BRANCH)\" -DGIT_SHA=\"$(GIT_SHA)\"

# ARCH=host (default, x86, native g++) or ARCH=rpi (cross-compile for RPi Zero 2W)
ARCH    ?= host
ifeq ($(ARCH),rpi)
  CXX      = aarch64-linux-gnu-g++
  CXXFLAGS += -march=armv8-a
  # Inline the env on every pkg-config invocation — `export` in Makefile
  # scope doesn't always reach $(shell …) at parse time. Paths come from
  # libgstrtspserver-1.0-dev:arm64 + libgstreamer1.0-dev:arm64 inside the
  # xbuild container.
  PKG_CFG := PKG_CONFIG_PATH=/usr/lib/aarch64-linux-gnu/pkgconfig \
             PKG_CONFIG_LIBDIR=/usr/lib/aarch64-linux-gnu/pkgconfig \
             PKG_CONFIG_SYSROOT_DIR=/ \
             pkg-config
else
  PKG_CFG := pkg-config
endif

# VIDEO=rtsp (default, embedded RTSP server; needs libgstrtspserver-1.0-dev)
# VIDEO=none (no video; for local sanity builds without gst deps)
VIDEO   ?= rtsp
ifeq ($(VIDEO),rtsp)
  # Swallow stderr: the host parses this Makefile when running `make xbuild`
  # even though only the container will use the values. Missing arm64 gst
  # deps on the host are expected and not an error. If compilation actually
  # needs these flags and they're missing, g++ fails with a clear "gst.h
  # not found" anyway.
  GST_CFLAGS := $(shell $(PKG_CFG) --cflags gstreamer-1.0 gstreamer-rtsp-server-1.0 2>/dev/null)
  GST_LIBS   := $(shell $(PKG_CFG) --libs   gstreamer-1.0 gstreamer-rtsp-server-1.0 2>/dev/null)
  CXXFLAGS   += $(GST_CFLAGS)
  LDLIBS     += $(GST_LIBS)
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
       src/mavlink/camera_handlers.cpp \
       src/camera/camera_discovery.cpp \
       src/video/rtsp_backend.cpp \
       src/video/udp_backend.cpp

ifeq ($(VIDEO),rtsp)
  SRCS += src/video/rtsp_server.cpp
else ifeq ($(VIDEO),none)
  SRCS += src/video/rtsp_server_stub.cpp
else
  $(error Unknown VIDEO=$(VIDEO). Use VIDEO=rtsp or VIDEO=none)
endif

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
  SRCS += src/lte/link_switcher.cpp
endif

HEADERS = $(wildcard include/*.hpp) \
          $(wildcard src/*.hpp) \
          $(wildcard src/mavlink/*.hpp) \
          $(wildcard src/camera/*.hpp) \
          $(wildcard src/video/*.hpp) \
          $(wildcard src/drive/*.hpp) \
          $(wildcard src/motor/*.hpp) \
          $(wildcard src/gimbal/*.hpp) \
          $(wildcard src/gps/*.hpp) \
          $(wildcard src/lte/*.hpp)

TARGET = build/ground_rover_daemon

all: build $(TARGET)

build:
	mkdir -p build

$(TARGET): $(SRCS) $(HEADERS)
	@echo "  CXX  $@  [ARCH=$(ARCH) DRIVER=$(DRIVER) GIMBAL=$(GIMBAL) GPS=$(GPS) LTE=$(LTE) VIDEO=$(VIDEO)]"
	$(CXX) $(CXXFLAGS) $(SRCS) -o $(TARGET) $(INC) $(LDFLAGS) $(LDLIBS)

rebuild: clean all

# Cross-compile for the Pi inside a throwaway Debian container — keeps arm64
# dev libs off the host (Ubuntu multiarch + these specific gst packages will
# nuke your desktop, learned the hard way). Pi OS Bookworm matches the
# container ABI, so the binary runs unchanged on the Pi.
# First build takes ~60 s (pulls base image + apt); subsequent builds are
# instant (Docker caches the apt layer).
# Usage: make xbuild [DRIVER=tb6612 GIMBAL=i2c GPS=nmea LTE=usb VIDEO=rtsp]
XBUILD_IMAGE := rover-xbuild
xbuild:
	@docker build -q -t $(XBUILD_IMAGE) deploy/xbuild/ >/dev/null
	docker run --rm -v $(CURDIR):/src -u $$(id -u):$$(id -g) $(XBUILD_IMAGE) \
	    make rebuild ARCH=rpi DRIVER=$(DRIVER) GIMBAL=$(GIMBAL) GPS=$(GPS) LTE=$(LTE) VIDEO=$(VIDEO)

# Deploy to RPi: make deploy [RPI=pi@pi-rover.lan]
# Build step runs inside the xbuild container; ssh/scp run on the host so
# your keys / ssh-agent / ~/.ssh/config work normally.
RPI ?= pi@pi-rover.lan
deploy:
	$(MAKE) xbuild DRIVER=tb6612 GPS=nmea LTE=usb VIDEO=rtsp
	ssh $(RPI) "mkdir -p /home/pi/ground-rover-daemon && sudo systemctl stop ground-rover-daemon || true"
	scp $(TARGET) $(RPI):/home/pi/ground-rover-daemon/
	scp deploy/ground-rover-daemon.service $(RPI):/tmp/
	ssh $(RPI) "sudo mv /tmp/ground-rover-daemon.service /etc/systemd/system/ && sudo systemctl daemon-reload && sudo systemctl enable --now ground-rover-daemon"

# Deploy modem config to a fresh/replacement UZ801 dongle via ADB.
# Copies scripts to the Pi and runs deploy-modem.sh there (adb is on the Pi).
# Usage: make deploy-modem [RPI=pi@pi-rover.lan]
deploy-modem: _deploy-modem-push verify-modem

_deploy-modem-push:
	ssh $(RPI) "mkdir -p /tmp/modem-deploy"
	scp deploy/modem/nat_forward.sh deploy/modem/led_status.sh \
	    deploy/modem/lte_status_srv.sh deploy/modem/link_switch_srv.sh \
	    deploy/modem/deploy-modem.sh \
	    deploy/modem/verify-modem.sh \
	    $(RPI):/tmp/modem-deploy/
	ssh -o ServerAliveInterval=3 -o ServerAliveCountMax=1 $(RPI) "bash /tmp/modem-deploy/deploy-modem.sh" 2>/dev/null; \
	echo ""; \
	echo "SSH dropped — modem is rebooting, waiting for it to come back..."

# Verify modem health after deploy/reboot.
# SSH may drop during modem reboot if Pi routes traffic through the modem,
# so verification polls until the Pi is reachable again.
# Usage: make verify-modem [RPI=pi@pi-rover.lan]
verify-modem:
	@echo "Waiting for Pi to be reachable (up to 90s)..."; \
	for i in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18; do \
	    ssh -o ConnectTimeout=5 -o BatchMode=yes $(RPI) true 2>/dev/null \
	        && echo "  [OK] Pi reachable" && break; \
	    echo "  ...$$((i * 5))s elapsed, retrying..."; \
	    sleep 5; \
	done; \
	ssh -o ConnectTimeout=1 -o BatchMode=yes $(RPI) true 2>/dev/null \
	    || (echo "  [FAIL] Cannot reach $(RPI) — is the modem up?"; exit 1)
	scp deploy/modem/verify-modem.sh $(RPI):/tmp/modem-deploy/verify-modem.sh
	ssh $(RPI) "bash /tmp/modem-deploy/verify-modem.sh"

# Configure static IP on Pi's RNDIS USB interface (usb0) for LTE modem tethering.
# Replaces dnsmasq DHCP — Pi uses 192.168.100.100/24 with GW 192.168.100.1.
# Persistent across reboots via NetworkManager or dhcpcd.
# Usage: make setup-pi-usb [RPI=pi@pi-rover.lan]
setup-pi-usb:
	scp deploy/pi/setup-usb-static.sh $(RPI):/tmp/setup-usb-static.sh
	ssh $(RPI) "bash /tmp/setup-usb-static.sh"

# Configure WiFi client uplink on the modem.
# Writes wpa_supplicant config so nat_forward.sh uses home WiFi instead of LTE.
# Usage: make setup-modem-wifi WIFI_SSID=S_HOME WIFI_PSK=password [RPI=pi@pi-rover.lan]
WIFI_SSID ?=
WIFI_PSK  ?=
setup-modem-wifi:
	scp deploy/modem/setup-wifi-client.sh $(RPI):/tmp/setup-wifi-client.sh
	ssh $(RPI) "WIFI_SSID='$(WIFI_SSID)' WIFI_PSK='$(WIFI_PSK)' bash /tmp/setup-wifi-client.sh"

# Remove WiFi client config from modem (reverts to LTE-only uplink).
# Usage: make remove-modem-wifi [RPI=pi@pi-rover.lan]
remove-modem-wifi:
	ssh $(RPI) "adb shell rm -f /data/misc/wifi/rover_wpa.conf && echo removed"

# Measure CPU usage on the UZ801 modem via ADB (runs through the Pi).
# Usage: make modem-cpu [RPI=pi@pi-rover.lan] [INTERVAL=5]
INTERVAL ?= 5
modem-cpu:
	scp deploy/modem/measure-cpu.sh deploy/modem/measure-cpu-inner.sh $(RPI):/tmp/
	ssh $(RPI) "bash /tmp/measure-cpu.sh $(INTERVAL)"

# One-time setup for the WireGuard tunnel to the MikroTik hAP (back-to-home-vpn).
# Get the hAP public key first: /interface wireguard print (on RouterOS)
# Usage: make setup-wireguard HAP_PUBKEY="<key>" [RPI=pi@pi-rover.lan]
#        [HAP_ENDPOINT=46.33.34.161:23392] [PI_WG_ADDR=192.168.216.6/24] [PI_WG_ADDR6=fc00:0:0:216::6/128]
HAP_PUBKEY   ?=
HAP_ENDPOINT ?= 46.33.34.161:23392
PI_WG_ADDR   ?= 192.168.216.6/24
PI_WG_ADDR6  ?= fc00:0:0:216::6/128
setup-wireguard:
	scp deploy/pi/setup-wireguard.sh $(RPI):/tmp/setup-wireguard.sh
	ssh $(RPI) "sudo HAP_PUBKEY='$(HAP_PUBKEY)' HAP_ENDPOINT='$(HAP_ENDPOINT)' \
	            PI_WG_ADDR='$(PI_WG_ADDR)' PI_WG_ADDR6='$(PI_WG_ADDR6)' \
	            bash /tmp/setup-wireguard.sh"

clean:
	rm -rf build

.PHONY: all build rebuild xbuild deploy deploy-modem _deploy-modem-push verify-modem setup-pi-usb setup-modem-wifi remove-modem-wifi modem-cpu setup-wireguard clean
