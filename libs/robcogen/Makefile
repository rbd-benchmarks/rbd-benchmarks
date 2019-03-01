GEN_DIR ?= gen
BUILD_DIR ?= build
INSTALL_DIR ?= install

ifeq ($(OS_NAME),Linux)
	LIBEXT := so
	CXXFLAGS += -Wl,--export-dynamic
endif
ifeq ($(OS_NAME),Darwin)
	LIBEXT := dylib
endif

ROBCOGEN_ZIP_URL := https://bitbucket.org/robcogenteam/robcogen-downloads/downloads/robcogen-0.5.1.zip
ROBCOGEN_ZIP_LOCAL := $(notdir $(ROBCOGEN_ZIP_URL))
ROBCOGEN_BASENAME := $(basename $(ROBCOGEN_ZIP_LOCAL) .zip)
ROBCOGEN_MD5 := robcogen.md5
ROBCOGEN_DEST_DIR := robcogen
ROBCOGEN_TMP_GEN_DIR := /tmp/gen
KINDSL_DIR := $(abspath ../../description/kindsl)
KINDSL_FILES := $(wildcard $(KINDSL_DIR)/*.kindsl)
ROBOT_NAMES := $(basename $(notdir $(KINDSL_FILES)))
ROBOT_GEN_DIRS := $(addprefix $(GEN_DIR)/,$(ROBOT_NAMES))
ROBOT_LIBS := $(addsuffix .$(LIBEXT),$(addprefix $(INSTALL_DIR)/lib/libiitgen,$(ROBOT_NAMES)))

EIGEN_TARBALL_URL = http://bitbucket.org/eigen/eigen/get/3.3.7.tar.gz
include ../Eigen.mk

.DEFAULT_GOAL := all

.PHONY: all all-gen all-libs clean iit-headers
all: all-libs iit-headers

$(ROBCOGEN_ZIP_LOCAL):
	wget -q $(ROBCOGEN_ZIP_URL) -O $(ROBCOGEN_ZIP_LOCAL)
	md5sum --status -c $(ROBCOGEN_MD5)

$(ROBCOGEN_DEST_DIR): $(ROBCOGEN_ZIP_LOCAL)
	unzip -q -o $(ROBCOGEN_ZIP_LOCAL)
	mv -f $(ROBCOGEN_BASENAME) $(ROBCOGEN_DEST_DIR)
	patch -s -p0 < robcogen_robot_subdir.patch
	patch -s -p0 < robcogen_zero_gravity.patch
	touch $(ROBCOGEN_DEST_DIR)

# Generate C++ code for .kindsl files in $(KINDSL_DIR)
# example: `make gen/iiwa`
$(GEN_DIR)/% : $(KINDSL_DIR)/%.kindsl $(ROBCOGEN_DEST_DIR)
	mkdir -p $(GEN_DIR)
	# Piping "1\n4\n28\n" to robcogen:
	# option 1: all Maxima targets
	# option 4: all C++ targets
	# option 28: exit
	cd $(ROBCOGEN_DEST_DIR)/exe && echo "1\n4\n28\n" | ./robcogen.sh $< > /dev/null
	# RobCoGen's output is in a temp directory. Move it to $(GEN_DIR):
	mkdir -p $@
	mv -f $(ROBCOGEN_TMP_GEN_DIR)/$*/cpp/* $@
	# Patch the CMakeLists.txt to have CMake pick up the IIT headers in the iit subdirectory
	# This avoids requiring users to install these headers into /usr/ and requiring sudo as a result.
	# Also turn off invalid contexpr warnings (turned into errors), needed to build with OSX.
	patch $@/CMakeLists.txt robcogen_cmakelists.patch
	# Patch to stop aligning statically, which fixes segfaults related 256 bit wide AVX instructions.
	patch $@/CMakeLists.txt robcogen_cmakelists_dont_align_statically.patch
	touch $@

$(INSTALL_DIR)/lib/libiitgen%.$(LIBEXT) : $(GEN_DIR)/% $(EIGEN_INCLUDE_DIR)
	{ \
	set -e ;\
	mkdir -p $(BUILD_DIR)/$* ;\
	cd $(BUILD_DIR)/$* ;\
	cmake -DEIGEN_ROOT=$(abspath $(EIGEN_INCLUDE_DIR)) -DHEADER_INSTALL_ROOT=$(abspath $(INSTALL_DIR))/include -DLIB_INSTALL_ROOT=$(abspath $(INSTALL_DIR))/lib -DCMAKE_CXX_FLAGS=-Wno-invalid-constexpr $(abspath $<) ;\
	$(MAKE) install ;\
	}

all-gen: $(ROBOT_GEN_DIRS)

all-libs: $(ROBOT_LIBS)

iit-headers: $(ROBCOGEN_DEST_DIR)
	mkdir -p $(INSTALL_DIR)/include
	cp -r $(ROBCOGEN_DEST_DIR)/etc/cpp-iitrbd/iit $(INSTALL_DIR)/include

clean: clean-eigen
	-rm -rf $(GEN_DIR)
	-rm -f $(ROBCOGEN_ZIP_LOCAL)
	-rm -rf $(ROBCOGEN_DEST_DIR)
	-rm -rf $(BUILD_DIR)
	-rm -rf $(INSTALL_DIR)
