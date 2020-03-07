####################################################################
#
#  Name:         Makefile
#  Created by:   Stefan Ritt
#
#  Contents:     Makefile for the v1720 frontend
#
#  $Id: Makefile 3655 2007-03-21 20:51:28Z amaudruz $
#
#####################################################################
#

# Path to gcc 4.8.1 binaries (needed to use new C++ stuff)
PATH := /home/deap/packages/newgcc/bin:$(PATH)

USE_SYSTEM_BUFFER=0
HWLOGDIR=\"$(DEAPDIR)/FrontEnd/v1720\"

# Hardware setup
NBLINKSPERA3818=4 # Number of optical links used per A3818
NBLINKSPERFE=4 # Number of optical links controlled by each frontend
NB1720PERLINK=1 # Number of daisy-chained v1720s per optical link
NBV1720TOTAL=4 # Number of v1720 boards in total
NBCORES=8

HWFLAGS = -DUSE_SYSTEM_BUFFER=$(USE_SYSTEM_BUFFER) \
-DNv1720=$(Nv1720) -DNBLINKSPERA3818=$(NBLINKSPERA3818) -DNBLINKSPERFE=$(NBLINKSPERFE) \
-DNB1720PERLINK=$(NB1720PERLINK) -DNBV1720TOTAL=$(NBV1720TOTAL) -DNBCORES=$(NBCORES) \
-DHWLOGDIR=$(HWLOGDIR)

#--------------------------------------------------------------------
# The MIDASSYS should be defined prior the use of this Makefile
ifndef MIDASSYS
missmidas::
	@echo "...";
	@echo "Missing definition of environment variable 'MIDASSYS' !";
	@echo "...";
endif

#--------------------------------------------------------------------
# The following lines contain specific switches for different UNIX
# systems. Find the one which matches your OS and outcomment the 
# lines below.
#
# get OS type from shell
OSTYPE = $(shell uname)

#-----------------------------------------
# This is for Linux
ifeq ($(OSTYPE),Linux)
OSTYPE = linux
endif

ifeq ($(OSTYPE),linux)
#OS_DIR = linux-m64
OS_DIR = linux
OSFLAGS = -DOS_LINUX -DLINUX
CFLAGS = -g -Wall -pthread $(HWFLAGS)
#For backtrace
#CFLAGS = -g -Wall -pthread $(HWFLAGS) -rdynamic -fno-omit-frame-pointer -fno-inline -fno-inline-functions
LDFLAGS = -g -lm -lz -lutil -lnsl -lpthread -lrt -lc 
endif

#-----------------------------------------
# optimize?

CFLAGS += -O2

#-----------------------------------------
# ROOT flags and libs
#
ifdef ROOTSYS
ROOTCFLAGS := $(shell  $(ROOTSYS)/bin/root-config --cflags)
ROOTCFLAGS += -DHAVE_ROOT -DUSE_ROOT
ROOTLIBS   := $(shell  $(ROOTSYS)/bin/root-config --libs) -Wl,-rpath,$(ROOTSYS)/lib
ROOTLIBS   += -lThread
else
missroot:
	@echo "...";
	@echo "Missing definition of environment variable 'ROOTSYS' !";
	@echo "...";
endif
#-------------------------------------------------------------------
#-------------------------------------------------------------------
# The following lines define directories. Adjust if necessary
#
# Expect the CAENCOMM and CAENVME to be installed system-wide
# using the libCAENComm.so, libCAENVME.so
#
# CONET2_DIR   = $(HOME)/packages/CONET2
# CAENCOMM_DIR = $(CONET2_DIR)/CAENComm-1.02
# CAENCOMM_LIB = $(CAENCOMM_DIR)/lib/x64
# CAENVME_DIR  = $(CONET2_DIR)/CAENVMELib-2.30.2
# CAENVME_DIR  = $(CONET2_DIR)/CAENVMELib-2.41
# CAENVME_LIB  = $(CAENVME_DIR)/lib/x64
MIDAS_INC    = $(MIDASSYS)/include
MIDAS_LIB    = $(MIDASSYS)/$(OS_DIR)/lib
MIDAS_SRC    = $(MIDASSYS)/src
MIDAS_DRV    = $(MIDASSYS)/drivers/vme
ROOTANA      = $(HOME)/packages/rootana

####################################################################
# Lines below here should not be edited
####################################################################
#
# compiler
CC   = gcc # -std=c99
#CXX  = g++ -std=c++11 -v
CXX  = g++ -std=c++11
#
# MIDAS library
LIBMIDAS=-L$(MIDAS_LIB) -lmidas
#
# CAENComm
#LIBCAENCOMM=-L$(CAENCOMM_LIB) -lCAENComm
LIBCAENCOMM=-lCAENComm
#
# CAENVME
#LIBCAENVME=-L$(CAENVME_LIB) -lCAENVME
LIBCAENVME=-lCAENVME
#
# All includes
# INCS = -I. -I./include -I$(MIDAS_INC) -I$(MIDAS_DRV) -I$(CAENVME_DIR)/include -I$(CAENCOMM_DIR)/include
INCS = -I. -I./include -I$(MIDAS_INC) -I$(MIDAS_DRV)

####################################################################
# General commands
####################################################################

all: fe
	@echo "***** Finished"
	@echo "***** Use 'make doc' to build documentation"

fe : feoV1720mt.exe

doc ::
	doxygen
	@echo "***** Use firefox --no-remote doc/html/index.html to view if outside gateway"

####################################################################
# Libraries/shared stuff
####################################################################

ov1720.o : $(MIDAS_DRV)/ov1720.c
	$(CC) -c $(CFLAGS) $(INCS) $< -o $@ 

####################################################################
# Single-thread frontend
####################################################################

feoV1720mt.exe: $(MIDAS_LIB)/mfe.o  feoV1720.o ov1720.o v1720CONET2.o
	$(CXX) $(OSFLAGS) feoV1720.o v1720CONET2.o ov1720.o $(MIDAS_LIB)/mfe.o $(LIBMIDAS) $(LIBCAENCOMM) $(LIBCAENVME) -o $@ $(LDFLAGS)

feoV1720.o : feoV1720.cxx v1720CONET2.o
	$(CXX) $(CFLAGS) $(OSFLAGS) $(INCS) -Ife -c $< -o $@

v1720CONET2.o : v1720CONET2.cxx
	$(CXX) $(CFLAGS) $(OSFLAGS) $(INCS) -Ife -c $< -o $@

$(MIDAS_LIB)/mfe.o:
	@cd $(MIDASSYS) && make
####################################################################
# Clean
####################################################################

clean:
	rm -f *.o *.exe
	rm -f *~
	rm -rf html
	rm -rf stress

####################################################################
# Stress test program
####################################################################
stress: stress_test.c
	$(CC) $(CFLAGS) $(INCS) -o $@ $(LDFLAGS) $< $(LIBCAENCOMM) $(LIBCAENVME)

setcards: setcards.cxx
	$(CXX) $(CFLAGS) $(INCS) -o $@ $(LDFLAGS) $< $(LIBCAENCOMM) $(LIBCAENVME)

#end file

