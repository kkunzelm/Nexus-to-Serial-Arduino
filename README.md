# Nexus to MQTT via USB

This project reads weather station sensor data which are encoded in Nexus format with a cheep RF receiver (RFM83C for 5V MCU) and sends them to a MQTT broker. In this example I used mosquitto as broker and from mosquitto_clients mosquitto_pub to publish the data.

# What you need

The project consists of three parts:

- Arduino/RFM83C connected via USB to a linux computer. The Arduino code is here: src/433-Nexus-to-Serial-Arduino/433-Nexus-to-Serial-Arduino.ino
- A script which reads the data coming via USB from the Arduino: src/Linux/tty2mqtt.sh
- A script to daemonize the data capture script: src/Linux/tty2mqttDaemonPart.sh

# Daemonize the computer part

The daemon part is adapted according to the suggestions here: 

	https://unix.stackexchange.com/questions/426862/proper-way-to-run-shell-script-as-a-daemon

Copy the script tty2mqtt.sh to /usr/bin and make it executable.

Create the file /etc/systemd/system/tty2mqttDaemonPart.sh and make it executable.

	[Unit]
	Description=ttymqtt bridge (requires Arduino on /dev/ttyUSB0)

	[Service]
	ExecStart=/usr/bin/tty2mqtt.sh
	Restart=on-failure

	[Install]
	WantedBy=multi-user.target

To start the daemon you run

	systemctl start tty2mqttDaemonPart.service 

To start at boot you enable it

	systemctl enable tty2mqttDaemonPart.service

# Additional notes

I used an Arduino Nano. But any microcontroller can do the job. Just make sure you choose the right receiver module. There are modules for 5V logic MCUs (i.e. RFM83C) and other models made for 3.3 V MCUs (i.e. RFM210C-433S1).

I could not use an Wemos D1-mini or a similar MCU with WIFI because I could not connect the ESP8266 I tested to Eduroam (should work for ESP32 however). Other WIFI networks were not available. But a linux computer was always on and connected to the internet. 


