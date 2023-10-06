#!/bin/bash

systemctl --user stop murin_websocket.service
systemctl --user disable murin_websocket.service
sudo rm /home/nhattm/.config/systemd/user/murin_websocket.service
systemctl --user daemon-reload