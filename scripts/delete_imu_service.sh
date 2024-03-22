#!/bin/bash

systemctl --user stop murin_imu.service
systemctl --user disable murin_imu.service
sudo rm /home/nhattm/.config/systemd/user/murin_imu.service
systemctl --user daemon-reload