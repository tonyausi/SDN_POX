#!/bin/bash

# $1:output file name string
# S2:unloged entity name string
# $3:bwm-ng sampling period in msec (int,default 500)
# $4:wait time in seconds (int)
#clear
fff=$1
> $fff
chmod a+rwx "$fff"
while [ -z "$(<$fff)" ]; do  # -z means null
  #bwm-ng -o csv -t 1 -I %lo -F $fff & #> /dev/null
  #bwm-ng -u bits -o csv -t 500 -I %$2 -F $fff &
  bwm-ng -u bits -o csv -t $3 -I %$2 -F $fff &
  #sleep 5
  sleep $4
  pkill bwm-ng #> /dev/null
done
#
echo "finished"
#
#cat $fff
