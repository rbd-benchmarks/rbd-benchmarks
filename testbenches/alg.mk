ifeq ($(ALG),rnea)
CXXFLAGS += -DRNEA_ALG
else ifeq ($(ALG),crba)
CXXFLAGS += -DCRBA_ALG
else ifeq ($(ALG),aba)
CXXFLAGS += -DABA_ALG
endif
