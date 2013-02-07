include $(shell rospack find mk)/cmake.mk

MAIN_DIR=$(shell pwd)
LIB_DIR=$(MAIN_DIR)/lib/
BIN_DIR=$(MAIN_DIR)/bin/
BUILD_DIR=$(MAIN_DIR)/build/
MODEL_DIR=$(MAIN_DIR)/models/

all: inst

checkedout:
	wget https://dl.dropbox.com/u/51957850/cad_models.zip
	touch checkedout
 
inst: wiped checkedout
	unzip cad_models.zip
	rm cad_models.zip
	touch inst
# 
wiped: Makefile
	make wipe
	touch wiped
 
wipe:
	rm -rf $(BIN_DIR) $(LIB_DIR) $(MODEL_DIR)
	rm -f checkedout wiped inst *.zip

clean: wipe
