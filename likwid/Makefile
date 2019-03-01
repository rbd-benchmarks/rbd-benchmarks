INSTALL_DIR := install

LIKWID_TARBALL_URL := https://github.com/RRZE-HPC/likwid/archive/4.3.2.tar.gz
LIKWID_TARBALL_LOCAL := likwid.tar.gz
LIKWID_MD5 := likwid.md5
LIKWID_SRC_DIR := likwid-src

ifeq ($(OS_NAME),Linux)
	LIKWID_TARFLAGS += --warning=no-unknown-keyword
endif

.PHONY: all build clean install

.DEFAULT_GOAL := all

$(LIKWID_TARBALL_LOCAL):
	wget -q $(LIKWID_TARBALL_URL) -O $(LIKWID_TARBALL_LOCAL)
	md5sum --status -c $(LIKWID_MD5)

$(LIKWID_SRC_DIR): $(LIKWID_TARBALL_LOCAL)
	tar $(LIKWID_TARFLAGS) -zxf $(LIKWID_TARBALL_LOCAL)
	mv likwid-* $(LIKWID_SRC_DIR)
	patch $(LIKWID_SRC_DIR)/config.mk likwid_config.patch
	touch $(LIKWID_SRC_DIR)

all: $(LIKWID_SRC_DIR)
	{ \
	set -e ;\
	cd $(LIKWID_SRC_DIR); \
	$(MAKE); \
	} \

install: build
	{ \
	set -e ;\
	cd $(LIKWID_SRC_DIR); \
	$(MAKE) install; \
	} \

clean:
	-rm -f $(LIKWID_TARBALL_LOCAL)
	-rm -rf $(LIKWID_SRC_DIR)
	-rm -rf $(INSTALL_DIR)
