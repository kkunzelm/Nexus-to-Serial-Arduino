#!/bin/bash

#
# This file captures data from an attached microcontroller
# The data have to be formated in the influxdb format 
# The data will be transmitted at regular intervals
#
# copy this file as root to: /usr/bin 
# make it executable
#
# Edit to your needs
# 
#   USBPORT
#   MQTTTOPIC
#   MQTTBROKER
#
# 1.3.2020 Karl-Heinz Kunzelmann


# USB port
USBPORT="/dev/ttyUSB0"

# MQTT topic
MQTTTOPIC="sensors"

# MQTT broker url
MQTTBROKER="xxx.yyy.zzz"

# no need to edit below this line

stty -F /dev/ttyUSB0 115200 cs8 -cstopb -parenb -icanon min 1 time 1
while true; do
        read LINE < /dev/ttyUSB0
        # remove CR at the end of $LINE
        TEXT=$(echo "$LINE $( date "+%s%N" )" | tr -d '\r')
        echo "$TEXT" | /usr/bin/mosquitto_pub -l -t "$MQTTTOPIC" -h "$MQTTBROKER"
done

