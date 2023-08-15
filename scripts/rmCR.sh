#!/bin/sh -f

HOME="/home/matsushi"
SRC=" ./include/hwlib.h ./include/socal/hps.h ./include/socal/socal.h ./include/socal/alt_gpio.h ./demos/hps_gpio/._main.c ./demos/hps_gpio/Makefile ./demos/hps_gpio/main.c"

for s in $SRC
do
  f="$HOME/$s"
  echo $f
  sed -e s/// $f > ${f}.mod
done

