#!/bin/bash

sudo cp murin_bringup.service /home/nhattm/.config/systemd/user
systemctl --user daemon-reload
systemctl --user enable murin_bringup.service
systemctl --user start murin_bringup.service
