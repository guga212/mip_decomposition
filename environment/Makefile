CONDA_ENV = network_flow_env
CONDA_CONFIG = conda_env.yml

create_environment:
	conda env create --file $(CONDA_CONFIG) --prefix ./$(CONDA_ENV)
remove_environment:
	rm -fr ./$(CONDA_ENV)
.PHONY: create_environment remove_environment