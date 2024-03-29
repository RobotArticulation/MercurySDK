##################################################
# PROJECT: Mercury sync_read_write Example Makefile
##################################################

##################################################
# Note - this is a modified version of the Dynamixel SDK sync_read_write example
# Original AUTHOR : ROBOTIS Ltd.
##################################################

#---------------------------------------------------------------------
# Makefile template for projects using MCY SDK
#
# Please make sure to follow these instructions when setting up your
# own copy of this file:
#
#   1- Enter the name of the target (the TARGET variable)
#   2- Add additional source files to the SOURCES variable
#   3- Add additional static library objects to the OBJECTS variable
#      if necessary
#   4- Ensure that compiler flags, INCLUDES, and LIBRARIES are
#      appropriate to your needs
#
#
# This makefile will link against several libraries, not all of which
# are necessarily needed for your project.  Please feel free to
# remove libaries you do not need.
#---------------------------------------------------------------------

# *** ENTER THE TARGET NAME HERE ***
TARGET      = sync_read_write

# important directories used by assorted rules and other variables
DIR_MCY    = ../../..
DIR_OBJS   = .objects

# compiler options
CC          = gcc
CX          = g++
# CCFLAGS     = -O0 -O3 -DLINUX -std=c++20 -D_GNU_SOURCE -Wall $(INCLUDES) $(FORMAT) -g
# CXFLAGS     = -O0 -O3 -DLINUX -std=c++20 -D_GNU_SOURCE -Wall $(INCLUDES) $(FORMAT) -g

CCFLAGS     = -O0 -DLINUX -std=c++20 -D_GNU_SOURCE -Wall $(INCLUDES) $(FORMAT) -g
CXFLAGS     = -O0 -DLINUX -std=c++20 -D_GNU_SOURCE -Wall $(INCLUDES) $(FORMAT) -g


CXXFLAGS 	= -O0 -std=c++20 -Wall -O0
LNKCC       = $(CX)
LNKFLAGS    = $(CXFLAGS) #-Wl,-rpath,$(DIR_THOR)/lib
FORMAT      = -m64

#---------------------------------------------------------------------
# Core components (all of these are likely going to be needed)
#---------------------------------------------------------------------
INCLUDES   += -I$(DIR_MCY)/include/mercury_sdk
LIBRARIES  += -lmercury_sdk_x64_cpp
LIBRARIES  += -lrt

#---------------------------------------------------------------------
# Files
#---------------------------------------------------------------------
SOURCES = ../sync_read_write.cpp \
    # *** OTHER SOURCES GO HERE ***

OBJECTS  = $(addsuffix .o,$(addprefix $(DIR_OBJS)/,$(basename $(notdir $(SOURCES)))))
#OBJETCS += *** ADDITIONAL STATIC LIBRARIES GO HERE ***


#---------------------------------------------------------------------
# Compiling Rules
#---------------------------------------------------------------------
$(TARGET): make_directory $(OBJECTS)
	$(LNKCC) $(LNKFLAGS) $(OBJECTS) -o $(TARGET) $(LIBRARIES)

all: $(TARGET)

clean:
	rm -rf $(TARGET) $(DIR_OBJS) core *~ *.a *.so *.lo

make_directory:
	mkdir -p $(DIR_OBJS)/

$(DIR_OBJS)/%.o: ../%.c
	$(CC) $(CCFLAGS) -c $? -o $@

$(DIR_OBJS)/%.o: ../%.cpp
	$(CX) $(CXFLAGS) -c $? -o $@

#---------------------------------------------------------------------
# End of Makefile
#---------------------------------------------------------------------
