#!/usr/bin/env bash
# One-time setup of the PX4 SITL toolchain in WSL (Ubuntu 22.04). Run it yourself: it asks for your
# Linux password (sudo). Installs the PX4 build dependencies without the NuttX (hardware) toolchain and
# without Gazebo: the SIH simulator built into PX4 is enough for the flight-controller tests.
set -e
PX4_TAG=v1.17.0
sudo apt-get update
sudo apt-get install -y git python3-pip
cd ~
if [ ! -d PX4-Autopilot ]; then
  git clone --branch "$PX4_TAG" --depth 1 --recursive --shallow-submodules \
      https://github.com/PX4/PX4-Autopilot.git
fi
bash PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools
python3 -m pip install --user pyulog
echo "PX4 SITL toolchain ready in ~/PX4-Autopilot"
