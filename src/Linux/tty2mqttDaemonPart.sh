[Unit]
Description=ttymqtt bridge (requires Arduino on /dev/ttyUSB0)

[Service]
ExecStart=/usr/bin/tty2mqtt.sh
Restart=on-failure

[Install]
WantedBy=multi-user.target 
