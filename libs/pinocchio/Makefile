BUILD_DIR ?= build
INSTALL_DIR ?= install

PINOCCHIO_TARBALL_URL := https://github.com/stack-of-tasks/pinocchio/releases/download/v2.0.0/pinocchio-2.0.0.tar.gz
PINOCCHIO_TARBALL_LOCAL := $(notdir $(PINOCCHIO_TARBALL_URL))
PINOCCHIO_MD5 := pinocchio.md5
PINOCCHIO_SRC_DIR := pinocchio-src

OS_NAME := $(shell uname -s)

ifeq ($(OS_NAME),Linux)
	TARFLAGS += --warning=no-unknown-keyword
endif

EIGEN_TARBALL_URL = http://bitbucket.org/eigen/eigen/get/3.3.7.tar.gz
include ../Eigen.mk

.DEFAULT_GOAL := all

.PHONY: all clean
all: $(INSTALL_DIR)

$(PINOCCHIO_TARBALL_LOCAL):
	wget -q $(PINOCCHIO_TARBALL_URL) -O $(PINOCCHIO_TARBALL_LOCAL)
	md5sum --status -c $(PINOCCHIO_MD5)

$(PINOCCHIO_SRC_DIR): $(PINOCCHIO_TARBALL_LOCAL)
	tar $(TARFLAGS) -zxf $(PINOCCHIO_TARBALL_LOCAL)
	mv $(basename $(basename $(PINOCCHIO_TARBALL_LOCAL))) $(PINOCCHIO_SRC_DIR)
	touch $(PINOCCHIO_SRC_DIR)

$(INSTALL_DIR): $(PINOCCHIO_SRC_DIR) $(EIGEN_INCLUDE_DIR)
	{ \
	mkdir -p $(BUILD_DIR); \
	cd $(BUILD_DIR); \
	PKG_CONFIG_PATH=$(abspath $(INSTALL_DIR))/share/pkgconfig cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$(abspath $(INSTALL_DIR)) -DBUILD_UNIT_TESTS=OFF -DBUILD_PYTHON_INTERFACE=OFF $(abspath $(PINOCCHIO_SRC_DIR)); \
	$(MAKE) install; \
	}

clean: clean-eigen
	-rm -f $(PINOCCHIO_TARBALL_LOCAL)
	-rm -rf $(BUILD_DIR)
	-rm -rf $(PINOCCHIO_SRC_DIR)
	-rm -rf $(INSTALL_DIR)
