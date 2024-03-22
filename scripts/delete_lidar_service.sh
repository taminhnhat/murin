#!/bin/bash

systemctl --user stop murin_lidar.service
systemctl --user disable murin_lidar.service
sudo rm /home/nhattm/.config/systemd/user/murin_lidar.service
systemctl --user daemon-reload