ROOT_MAKEFILE_DIR=../..
# Get source directory name
PROJECT_SRC_DIR=$(shell basename $$(pwd))
export PROJECT_SRC_DIR
# Write ouput files in output/$(PROJECT_SRC_DIR)
OUTPUT_SUB_PATH=$(PROJECT_SRC_DIR)
export OUTPUT_SUB_PATH

.PHONY: all program

all:
	$(MAKE) -C $(ROOT_MAKEFILE_DIR)

program:
	$(MAKE) -C $(ROOT_MAKEFILE_DIR) program

%:
	$(MAKE) -C $(ROOT_MAKEFILE_DIR) $@
