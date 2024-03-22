#!/bin/bash

sudo cp murin_imu.service /home/nhattm/.config/systemd/user
systemctl --user daemon-reload
systemctl --user enable murin_imu.service
systemctl --user start murin_imu.service
