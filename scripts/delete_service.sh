#!/bin/bash

systemctl --user stop murin_bringup.service
systemctl --user disable murin_bringup.service
systemctl --user daemon-reload
sudo rm /home/nhattm/.config/systemd/user/murin_bringup.service