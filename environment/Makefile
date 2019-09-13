CONDA_ENV = network_flow_env
CONDA_CONFIG = conda_env.yml
SOLVERS_DIR = solvers

create_environment:
	conda env create --file $(CONDA_CONFIG) --prefix ./$(CONDA_ENV)
add_solvers:
	cp -a ./$(SOLVERS_DIR)/* ./$(CONDA_ENV)/bin/
remove_environment:
	rm -fr ./$(CONDA_ENV)
.PHONY: create_environment add_solvers remove_environment
