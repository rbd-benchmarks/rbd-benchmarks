RM = rm -f
JULIA ?= julia
JULIA_PATH := $(shell $(JULIA) -e 'println(Sys.BINDIR)')/..
RBD_BENCHMARKS_PATH = $(RBD_BENCHMARKS)
RBDJL_PATH = $(RBD_BENCHMARKS_PATH)/libs/rbdjl/install
RESULTS_PATH = $(RESULTS)

LDFLAGS += -L$(JULIA_PATH)/lib -L$(RBDJL_PATH)
LDFLAGS += -lstaticrbdjl -ljulia -lrt
LDFLAGS += -Wl,--export-dynamic -Wl,-rpath,$(JULIA_PATH)/lib -Wl,-rpath,$(JULIA_PATH)/lib/julia '-Wl,-rpath,$(RBDJL_PATH)/'
CXXFLAGS += -DRBD_BENCHMARKS_DIR=\"$(RBD_BENCHMARKS_PATH)\" -DRESULTS_DIR=\"$(RESULTS_PATH)\"
CXXFLAGS += -DJULIAC_PROGRAM_LIBNAME=\"$(RBDJL_PATH)/libstaticrbdjl.so\"
CXXFLAGS += -I$(JULIA_PATH)/include/julia -DJULIA_ENABLE_THREADING=1 -fPIC
CXXFLAGS += -O3 -DNDEBUG -std=c++11

.PHONY: all
all: $(TARGET)

$(TARGET): $(TARGET).o
	$(CXX) $(TARGET).o -o $(TARGET) $(LDFLAGS)

$(TARGET).o: $(TARGET).cc
	$(CXX) $(CXXFLAGS) -o $(TARGET).o -c $(TARGET).cc

clean:
	$(RM) *.o $(TARGET)
