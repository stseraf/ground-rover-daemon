CXX      = g++
CXXFLAGS = -std=c++17 -O2 -Wall -Wextra
INC      = -Iinc -Iexternal/mavlink

SRCS    = src/main.cpp
HEADERS = $(wildcard inc/*.hpp)
TARGET  = build/ground_rover_daemon

all: build $(TARGET)

build:
	mkdir -p build

$(TARGET): $(SRCS) $(HEADERS)
	$(CXX) $(CXXFLAGS) $(SRCS) -o $(TARGET) $(INC)

clean:
	rm -rf build
