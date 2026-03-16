CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall -Wextra
INC = -Iexternal/mavlink

all:
	$(CXX) $(CXXFLAGS) main.cpp -o ground_rover_daemon $(INC)

clean:
	rm -f ground_rover_daemon