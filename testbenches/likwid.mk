# EVENTS
ifeq ($(EVENTS),branch)
CXXFLAGS += -DBRANCH_EVENTS
else ifeq ($(EVENTS),l2cache)
CXXFLAGS += -DL2CACHE_EVENTS
else ifeq ($(EVENTS),l3cache)
CXXFLAGS += -DL3CACHE_EVENTS
else ifeq ($(EVENTS),energy)
CXXFLAGS += -DENERGY_EVENTS
else ifeq ($(EVENTS),mix)
CXXFLAGS += -DMIX_EVENTS
else ifeq ($(EVENTS),stall)
CXXFLAGS += -DSTALL_EVENTS
else ifeq ($(EVENTS),mstall)
CXXFLAGS += -DMSTALL_EVENTS
else ifeq ($(EVENTS),flopsdp)
CXXFLAGS += -DFLOPSDP_EVENTS
else ifeq ($(EVENTS),flopssp)
CXXFLAGS += -DFLOPSSP_EVENTS
else ifeq ($(EVENTS),uops)
CXXFLAGS += -DUOPS_EVENTS
endif

LIKWID_DIR = $(RBD_BENCHMARKS)/likwid/install

CXXFLAGS += -I$(LIKWID_DIR)/include

LDFLAGS += -L$(LIKWID_DIR)/lib
LDFLAGS += -Wl,-rpath,$(LIKWID_DIR)/lib
LDFLAGS += -llikwid
