CXX      = g++
CXXFLAGS = -std=c++17 -O2 -Wall -Wextra
INC      = -Iinc -Iexternal/mavlink

SRCS   = src/main.cpp
TARGET = build/ground_rover_daemon

all: build $(TARGET)

build:
	mkdir -p build

$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) $(SRCS) -o $(TARGET) $(INC)

clean:
	rm -rf build
