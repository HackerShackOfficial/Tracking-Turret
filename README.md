# Tracking-Turret
A motion tracking turret for https://www.youtube.com/watch?v=HoRPWUl_sF8

## Install Guide

Make sure pip is installed. 
```bash
sudo apt-get install python pip
```

Setup I2C on your Raspberry Pi

https://learn.adafruit.com/adafruits-raspberry-pi-lesson-4-gpio-setup/configuring-i2c

Install the Adafruit stepper motor HAT library.

```bash
sudo pip install git+https://github.com/adafruit/Adafruit-Motor-HAT-Python-Library
```

Install OpenCV 3. Follow all steps for python 2.7 instructions

http://www.pyimagesearch.com/2016/04/18/install-guide-raspberry-pi-3-raspbian-jessie-opencv-3/

Make sure to create your virtual environment with the extra flag.

```bash
mkvirtualenv cv --system-site-packages -p python2
```

Source your bash profile

```bash
source ~/.profile
```

Activate your virtual environment

```
workon cv
```

Clone this repository

```
git clone git@github.com:HackerHouseYT/Tracking-Turret.git
```

Navigate to the directory

```
cd Tracking-Turret
```

Install dependencies to your virtual environment

```
pip install imutils RPi.GPIO
```

Run the project!

```
python turret.py
```

## Setting Parameters

turret.py has a couple parameters that you can set.

```python
### User Parameters ###

MOTOR_X_REVERSED = False
MOTOR_Y_REVERSED = False

MAX_STEPS_X = 30
MAX_STEPS_Y = 15

RELAY_PIN = 22

#######################
```

These will be located at the top of the file. Use `vim turret.py` to open the file. Press `i` to edit.
Once you've made your changes, press `esc` then `ZZ` to save.
