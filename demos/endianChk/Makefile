#
TARGET = endchk

#
ALT_DEVICE_FAMILY ?= soc_cv_av

HWLIBS_ROOT = $(SOCEDS_DEST_ROOT)/ip/altera/hps/altera_hps/hwlib

# CFLAGS = -g -Wall -Werror -I$(HWLIBS_ROOT)/include -I$(HWLIBS_ROOT)/include/$(ALT_DEVICE_FAMILY) -D$(ALT_DEVICE_FAMILY)
# CFLAGS = -g -Wall -Werror -I/home/matsushi/include -ULINUX_TASK_MODE
# CFLAGS = -g -Wall -Werror -I/home/matsushi/include
CFLAGS = -g -Wall -Werror
LDFLAGS = -g -Wall -Werror

CROSS_COMPILE = arm-linux-gnueabihf-
# CC = $(CROSS_COMPILE)gcc
# LD = $(CROSS_COMPILE)gcc
CC = gcc
LD = gcc
ARCH = arm

.PHONY: build
build: $(TARGET)

$(TARGET): main.o
	$(LD) $(LDFLAGS)   $^ -o $@

%.o : %.c
	$(CC) $(CFLAGS) -c $< -o $@

.PHONY: clean
clean:
	rm -f $(TARGET) *.a *.o *~
