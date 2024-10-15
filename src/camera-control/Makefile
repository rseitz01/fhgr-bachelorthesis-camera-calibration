#### Start of system configuration section. ####

TTY_AUTO := 1

ifeq ($(TTY_AUTO),1)
TTY_FLAGS := -D TTY_AUTO_ENABLED=1
TTY_LIBS  := -lncurses -ltinfo
else
TTY_FLAGS := -D TTY_AUTO_ENABLED=0
TTY_LIBS  :=
endif

VERSION := 1.00
CC      := g++ -O
CFLAGS  := -Wall -c `pkg-config opencv4 --cflags` -I"C:\Program Files\IDS\uEye\develop\include" $(TTY_FLAGS)
LDFLAGS := $(TTY_LIBS)
LDLIBS	:= `pkg-config opencv4 --cflags --libs --static` -L/usr/lib64 -lueye_api
CSUFFIX	:= .cpp
HSUFFIX	:= .hpp

#### End of system configuration section. ####

APPNAME = a

# Path for important files
# .c and .h files
SRC_DIR = src
# .o files
OBJ_DIR = obj

.PHONY: all clean


# Files to compile
TARGET  := $(APPNAME)
C_FILES := $(wildcard $(SRC_DIR)/*$(CSUFFIX))
O_FILES := $(addprefix $(OBJ_DIR)/,$(notdir $(C_FILES:$(CSUFFIX)=.o)))
D_FILES := $(addprefix $(OBJ_DIR)/,$(notdir $(C_FILES:$(CSUFFIX)=.d)))

all: $(TARGET)

#link all .o files
$(TARGET): $(O_FILES)
	@echo link    : $^
	@$(CC) $(LDFLAGS) -o $@ $^ $(LDLIBS)

# depend include files
-include $(D_FILES)

#compile all .c Files
$(OBJ_DIR)/%.o: $(SRC_DIR)/%$(CSUFFIX) Makefile | $(OBJ_DIR)
	@echo compile : $<
	@$(CC) $(CFLAGS) -MMD -MP -o $@ $<

# create directories if they don't exist
# .o dir
$(OBJ_DIR):
	@mkdir $@

#### CLEANING ####

ifeq ($(OS),Windows_NT)
# Cleaning rules for Windows OS
clean:
	@del /q $(OBJ_DIR)
	@rmdir $(OBJ_DIR)
else
# Cleaning rules for Unix-based OS (no clue if this works!)
clean:
	@rm -rf $(OBJ_DIR)
endif

