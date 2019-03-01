EIGEN_MD5 ?= eigen.md5
EIGEN_TARBALL_LOCAL ?= eigen.tar.gz
EIGEN_BUILD_DIR ?= $(BUILD_DIR)/eigen
EIGEN_INCLUDE_DIR := $(INSTALL_DIR)/include/eigen3

ifeq ($(OS_NAME),Linux)
	EIGEN_TARFLAGS += --warning=no-unknown-keyword
endif

$(EIGEN_TARBALL_LOCAL):
	wget -q $(EIGEN_TARBALL_URL) -O $(EIGEN_TARBALL_LOCAL)
	md5sum --status -c $(EIGEN_MD5)

$(EIGEN_INCLUDE_DIR): $(EIGEN_TARBALL_LOCAL)
	tar $(EIGEN_TARFLAGS) -zxf $(EIGEN_TARBALL_LOCAL)
	mv -f eigen-eigen-* eigen-src
	{ \
	set -e; \
	mkdir -p $(EIGEN_BUILD_DIR); \
	cd $(EIGEN_BUILD_DIR); \
	cmake -DCMAKE_INSTALL_PREFIX=$(abspath $(INSTALL_DIR)) $(abspath eigen-src); \
	$(MAKE) install; \
	}

clean-eigen:
	rm -rf eigen-src
	rm -rf $(EIGEN_TARBALL_LOCAL)
	rm -rf $(EIGEN_INCLUDE_DIR)
