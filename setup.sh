#!/usr/bin/env bash

set -e

VENV_NAME="venv"

echo "========================================"
echo " ASV Setup (ROS 2 Humble + MAVROS + VENV)"
echo "========================================"

# ---------- Update system ----------
echo "[1/8] Updating system..."
sudo apt update && sudo apt upgrade -y

# ---------- Basic system deps ----------
echo "[2/8] Installing system dependencies..."
sudo apt install -y \
    git \
    curl \
    build-essential \
    python3-dev \
    python3-venv \
    python3-pip \
    python3-opencv \
    python3-wxgtk4.0 \
    python3-lxml \
    python3-pygame

# ---------- ROS 2 Humble ----------
echo "[3/8] Installing ROS 2 Humble..."
sudo apt install -y ros-humble-desktop

# ---------- MAVROS ----------
echo "[4/8] Installing MAVROS..."
sudo apt install -y \
    ros-humble-mavros \
    ros-humble-mavros-extras

# ---------- GeographicLib ----------
echo "[5/8] Installing GeographicLib datasets..."
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

# ---------- Create Python VENV ----------
echo "[6/8] Creating Python virtual environment..."
if [ ! -d "$VENV_NAME" ]; then
    python3 -m venv $VENV_NAME
fi

source $VENV_NAME/bin/activate

# ---------- Python packages in VENV ----------
echo "[7/8] Installing Python packages into VENV..."
pip install --upgrade pip
pip install \
    numpy \
    scipy \
    pyserial \
    opencv-python \
    matplotlib

deactivate

# ---------- ROS environment ----------
echo "[8/8] Configuring ROS environment..."
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi

echo "========================================"
echo " Setup selesai ✅"
echo " Aktifkan venv: source venv/bin/activate"
echo " Cek MAVROS: ros2 pkg list | grep mavros"
echo "========================================"
