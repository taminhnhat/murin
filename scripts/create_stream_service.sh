#!/bin/bash

sudo cp murin_stream.service /home/nhattm/.config/systemd/user
systemctl --user daemon-reload
systemctl --user enable murin_stream.service
systemctl --user start murin_stream.service
