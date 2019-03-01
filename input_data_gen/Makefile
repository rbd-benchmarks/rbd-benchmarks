JULIA ?= julia
CSVDIR = ../csv

.PHONY: all
all: $(CSVDIR)

$(CSVDIR):
	$(JULIA) --project=. --color=yes --startup-file=no -e 'import Pkg; Pkg.instantiate()'
	$(JULIA) --project=. --color=yes --startup-file=no -e 'using BenchmarkInputDataGen; write_csv_files(num_samples=100_000)'

.PHONY: clean
clean:
	rm -rf $(CSVDIR)

.PHONY: test
test:
	$(JULIA) --project=. -e 'using Pkg; Pkg.test()'
