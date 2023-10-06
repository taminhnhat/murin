#!/bin/bash

sudo cp murin_websocket.service /home/nhattm/.config/systemd/user
systemctl --user daemon-reload
systemctl --user enable murin_websocket.service
systemctl --user start murin_websocket.service
