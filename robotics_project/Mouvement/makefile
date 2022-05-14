
#This is a template to build your own project with the e-puck2_main-processor folder as a library.
#Simply adapt the lines below to be able to compile

# Define project name here
PROJECT = Movement

#Define path to the e-puck2_main-processor folder
GLOBAL_PATH = ../../lib/e-puck2_main-processor

#Source files to include
CSRC += ./main.c \
		./communications.c \
		./move.c \
		./odometrie.c \
		./listen.c \
		./process_image.c \

#Header folders to include
INCDIR +=

# Debugging options
USE_OPT = -O2 -g -ggdb -fomit-frame-pointer -falign-functions=16 -fno-strict-aliasing -fstack-protector-all -L .

# Warnings
USE_COPT = -Wfatal-errors -Wall -Wextra -Wshadow -Wunused -Wnull-dereference -Wdouble-promotion \
-Wmisleading-indentation -Wduplicated-cond -Wduplicated-branches -Wlogical-op

#Jump to the main Makefile
include $(GLOBAL_PATH)/Makefile
