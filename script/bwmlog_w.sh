#!/bin/bash

# $1:output file name string
# S2:unloged entity name string
# $3:bwm-ng sampling period in msec (int,default 500)
#clear
fileIn=$1
> $fileIn
chmod a+rwx "$fileIn"
#while [ -z "$(<$fileIn)" ]; do
bwm-ng -u bits -o csv -t $3 -I %$2 -W $fileIn &
#done
#
echo "bwm-ng start logging with -W mode"
