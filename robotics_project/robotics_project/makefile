
#This is a template to build your own project with the e-puck2_main-processor folder as a library.
#Simply adapt the lines below to be able to compile

# Define project name here
PROJECT = robotics_project

#Define path to the e-puck2_main-processor folder
GLOBAL_PATH = ../../lib/e-puck2_main-processor

#Source files to include
CSRC += ./main.c \
		./communications.c \
		./process_image.c

#Header folders to include
INCDIR += 

#Compiler options
USE_OPT = -O3 -ggdb -fomit-frame-pointer -falign-functions=16 -fno-strict-aliasing -fstack-protector-all -L .

#Jump to the main Makefile
include $(GLOBAL_PATH)/Makefile
