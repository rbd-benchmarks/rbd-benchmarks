INSTALL_DIR ?= install
JULIA ?= julia

JULIA_BINDIR := $(shell $(JULIA) -e 'println(Sys.BINDIR)')
JULIA_PATH := $(JULIA_BINDIR)/..
OS_NAME := $(shell uname -s)
SCALAR_TYPE ?= 1 # 1: Float64, 2: Float32
DEPOT_PATH := $(abspath packages)
SRC_DIR := static-rbdjl
CXXFLAGS += -I$(JULIA_PATH)/include/julia -DJULIA_ENABLE_THREADING=1 -fPIC -L$(JULIA_PATH)/lib -Wl,-rpath,$(JULIA_PATH)/lib -Wl,-rpath,$(JULIA_PATH)/lib/julia -ljulia -m64 -O3 '-Wl,-rpath,$$ORIGIN'

ifeq ($(OS_NAME),Linux)
	LIBEXT := so
	CXXFLAGS += -Wl,--export-dynamic
endif
ifeq ($(OS_NAME),Darwin)
	LIBEXT := dylib
endif

.DEFAULT_GOAL := all

.PHONY: all clean clean-driver

all: $(INSTALL_DIR)/driver

$(INSTALL_DIR)/driver: $(SRC_DIR)/driver.c $(INSTALL_DIR)/libstaticrbdjl.$(LIBEXT)
	$(CXX) -DSCALAR_TYPE=$(SCALAR_TYPE) -DJULIAC_PROGRAM_LIBNAME=\"libstaticrbdjl.$(LIBEXT)\" -o $(INSTALL_DIR)/driver $(SRC_DIR)/driver.c $(INSTALL_DIR)/libstaticrbdjl.$(LIBEXT) $(CXXFLAGS)

$(INSTALL_DIR)/libstaticrbdjl.$(LIBEXT): $(SRC_DIR)/build.jl $(SRC_DIR)/snoopfile.jl $(SRC_DIR)/staticrbd.jl $(SRC_DIR)/Manifest.toml $(SRC_DIR)/Project.toml
	JULIA_DEPOT_PATH=$(DEPOT_PATH) $(JULIA) --project=$(SRC_DIR) --color=yes --startup-file=no -e 'import Pkg; Pkg.instantiate(); Pkg.build(); include("$(SRC_DIR)/build.jl")' $(INSTALL_DIR)

clean:
	rm -rf $(INSTALL_DIR)

clean-driver:
	rm -f $(INSTALL_DIR)/driver
