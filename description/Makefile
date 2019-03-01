JULIA ?= julia

.PHONY: all

all:
	cd ../libs/robcogen && $(MAKE) robcogen
	$(JULIA) --project=generation --color=yes --startup-file=no -e 'import Pkg; Pkg.instantiate()'
	$(JULIA) --project=generation --color=yes --startup-file=no generation/generate_description_files.jl
