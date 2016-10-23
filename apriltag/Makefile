#
# Makefile for apriltag mex interface code
#

SOURCEDIR=src

CC = gcc

MEX := mex
MKOCTFILE := mkoctfile

MEX_EXISTS := $(shell command -v $(MEX) 2> /dev/null)
MKOCT_EXISTS := $(shell command -v $(MKOCTFILE) 2> /dev/null)

CFLAGS = -fpic -Wall -Wno-unused-parameter \
	-Wno-unused-function -pthread \
	-I$(SOURCEDIR) -I$(SOURCEDIR)/common -O4
LDFLAGS = -lpthread -lm

#
# mex files need to be relocatable, so april tag objects need to be recompiled with -fpic
#

SOURCES := $(shell find $(SOURCEDIR) -name '*.c')
OBJECTS := $(SOURCES:%.c=%.o)

all: mex

mex: $(OBJECTS) find_apriltags.c
ifdef MEX_EXISTS
	@$(MEX) CFLAGS='$(CFLAGS)' $(OBJECTS) find_apriltags.c
endif
ifdef MKOCT_EXISTS
	@$(MKOCTFILE) --mex $(CFLAGS) find_apriltags.c $(OBJECTS)
endif


%.o: %.c
	@echo "   $@"
	@$(CC) -o $@ -c $< $(CFLAGS)

.PHONY: clean
clean:
	rm -rf find_apriltags.o
	@rm -rf $(OBJECTS) *.mex
