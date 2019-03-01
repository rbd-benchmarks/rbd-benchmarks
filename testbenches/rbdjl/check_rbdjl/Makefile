# Usage: make JULIA=? RBD_BENCHMARKS=? ALG=? ROB=?
TARGET = check_rbdjl

ifeq ($(ROB), iiwa)
FLOATING_BASE=0
else
FLOATING_BASE=1
endif

# ROBOT MODEL and FLOATING
CXXFLAGS+=-D ROBOT_MODEL=\"$(ROB)\" -D FLOATING=$(FLOATING_BASE)

include ../../alg.mk
include ../rbdjl.mk
