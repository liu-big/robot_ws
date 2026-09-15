CXX      := g++
CXXFLAGS := -O2 -Wall -Wextra -std=c++17
LDFLAGS  := -pthread -lrt

TARGET   := servo_rt
SRCS     := servo_rt.cpp

.PHONY: all clean run

all: $(TARGET)

$(TARGET): $(SRCS) shm_interface.h
	$(CXX) $(CXXFLAGS) -o $@ $(SRCS) $(LDFLAGS)

run: $(TARGET)
	sudo chrt -f 90 taskset -c 3 ./$(TARGET)

clean:
	rm -f $(TARGET)
