#!/bin/bash

systemctl --user stop murin_stream.service
systemctl --user disable murin_stream.service
sudo rm /home/nhattm/.config/systemd/user/murin_stream.service
systemctl --user daemon-reload