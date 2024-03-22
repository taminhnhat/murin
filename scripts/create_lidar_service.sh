#!/bin/bash

sudo cp murin_lidar.service /home/nhattm/.config/systemd/user
systemctl --user daemon-reload
systemctl --user enable murin_lidar.service
systemctl --user start murin_lidar.service
