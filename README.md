# ENG103_TASK3_AV079

Install dependencies

sudo apt-get update
sudo apt-get install -y python3-pip python3-rpi.gpio i2c-tools
pip3 install max30102 RPi.GPIO smbus2 numpy heartpy RPLCD

# Optional: verify devices on the I²C bus
sudo i2cdetect -y 1
