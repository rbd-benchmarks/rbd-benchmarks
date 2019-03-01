# Approach from https://www.oreilly.com/library/view/managing-projects-with/0596006101/ch06.html.

LIBS := $(patsubst %/,%,$(wildcard */))
CLEAN_TARGETS := $(addprefix clean-,$(LIBS))

.PHONY: all clean $(LIBS)

all: $(LIBS)

clean:
	$(MAKE) TARGET=clean

$(LIBS):
	echo $(MAKE)
	$(MAKE) --directory=$@ $(TARGET)
