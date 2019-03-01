RM=rm -f
RBD_BENCHMARKS_PATH = $(RBD_BENCHMARKS)
RESULTS_PATH = $(RESULTS)

CXXFLAGS += -DRBD_BENCHMARKS_DIR=\"$(RBD_BENCHMARKS_PATH)\" -DRESULTS_DIR=\"$(RESULTS_PATH)\"
CXXFLAGS += $(shell pkg-config --cflags pinocchio)
CXXFLAGS += -pedantic -Wno-long-long -Wall -Wextra -Wcast-align -Wcast-qual -Wformat -Wwrite-strings -Wconversion
CXXFLAGS += -O3 -DNDEBUG -std=c++11

LDFLAGS += $(shell pkg-config --libs pinocchio)
LDFLAGS += -rdynamic

$(TARGET): $(TARGET).o
	$(CXX) $(TARGET).o  -o $(TARGET) $(LDFLAGS)

$(TARGET).o: $(TARGET).cc
	$(CXX) $(CXXFLAGS) -o $(TARGET).o -c $(TARGET).cc

clean:
	$(RM) *.o $(TARGET)

