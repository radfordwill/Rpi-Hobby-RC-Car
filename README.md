
# Rpi-Hobby-RC-Car

This was for a custom hobby rc car with a raspberry pi sbc. Python3 and pip are required. Opencv takes some time to build. I had to replace the steering servo to a three wire version in the rc car I chose. I added the link but most three wire that take pwm signals will do. You may not need to replace yours, mine was a five wire servo. Cut the red power wire from the ESC before you connect it to the PCA9685 board.

**requirements:**

`sudo apt-get update`


adafruit-blinka


`sudo pip3 install adafruit-blinka`

`sudo pip3 install adafruit-circuitpython-pca9685`

`sudo pip3 install adafruit-circuitpython-servokit`


flask


`sudo pip3 install flask`


opencv dependencies


`sudo apt-get install build-essential cmake pkg-config libjpeg-dev libtiff5-dev libjasper-dev libpng-dev libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libxvidcore-dev libx264-dev libfontconfig1-dev libcairo2-dev libgdk-pixbuf2.0-dev libpango1.0-dev libgtk2.0-dev libgtk-3-dev libatlas-base-dev gfortran libhdf5-dev libhdf5-serial-dev libhdf5-103 python3-pyqt5 python3-dev -y`

`pip install --upgrade pip setuptools wheel`


opencv


`sudo pip3 install opencv-python`

or

`sudo pip3 install opencv-contrib-python`


rpi car install


`sudo git clone https://github.com/radfordwill/Rpizero-Hobby-RC-Car.git`

from the folder you just cloned, run main.py

`sudo python main.py`

**hardware:**

raspberry pi of your choice. pi zero and upwards to run minimum of opencv
RC hobby car of your choice. STL files will be linked soon. I used the frame from the donkey car design. 

https://github.com/autorope/donkeycar

They suggest the following rc cars. I used a 1:10 model car and modified the frame by enlarging it a little.

>Magnet

>Desert Monster

>SCT

>Blaze

>LaTrax Prerunner


(amazon affiliate links)


[Holyton 1 10 Large High Speed Remote Control Car](https://www.amazon.com/gp/product/B08B1F3494/?&_encoding=UTF8&tag=radfordwill-20&linkCode=ur2&linkId=a5871a5b2f58d54da336be76d4abd529&camp=1789&creative=9325)

[Brushed ESC](https://www.amazon.com/gp/product/B07792FPD8/?&_encoding=UTF8&tag=radfordwill-20&linkCode=ur2&linkId=b229acde647191fe04d091e4002ce954&camp=1789&creative=9325)


[IMU MPU6050](https://www.amazon.com/Gy-521-MPU-6050-MPU6050-Sensors-Accelerometer/dp/B008BOPN40/?&_encoding=UTF8&tag=radfordwill-20&linkCode=ur2&linkId=7cf9cfd6a0703f639e5233b96f02dc2f&camp=1789&creative=9325)

[WS2812B 5050 RGB 8-Bit Light Strip Driver Board](https://www.amazon.com/gp/product/B081BBF4R3/?&_encoding=UTF8&tag=radfordwill-20&linkCode=ur2&linkId=ac957b4343d7128fc49eb0634e6abc5e&camp=1789&creative=9325)

[MakerFocus Lidar Range Finder Sensor Module TF-Luna](https://www.amazon.com/gp/product/B088NVX2L7/?&_encoding=UTF8&tag=radfordwill-20&linkCode=ur2&linkId=57179b9cb3dfd3e78f85faa5a1d0c6bb&camp=1789&creative=9325)

[16 Channel 12 bit PWM Servo Motor Driver I2C](https://www.amazon.com/gp/product/B082QT9D5F/?&_encoding=UTF8&tag=radfordwill-20&linkCode=ur2&linkId=b0fab216e1d7decc94ec049af2f824df&camp=1789&creative=9325)

[5200mAh 2S LiPo Battery 50C POWAY](https://www.amazon.com/gp/product/B07Y1M571D/?&_encoding=UTF8&tag=radfordwill-20&linkCode=ur2&linkId=111564028c9594cd01c6afb5d2fd89c0&camp=1789&creative=9325)

[kuman UPS Battery Pack Expansion Board Power Supply](https://www.amazon.com/gp/product/B06W9FWDSP/?&_encoding=UTF8&tag=radfordwill-20&linkCode=ur2&linkId=55f9575e86e2699d50cd8f58029f507f&camp=1789&creative=9325)

[Arducam 5MP Camera for Raspberry Pi](https://www.amazon.com/Arducam-Megapixels-Sensor-OV5647-Raspberry/dp/B012V1HEP4/?&_encoding=UTF8&tag=radfordwill-20&linkCode=ur2&linkId=0a9469df29b006fb3b4a6dabb4f4665d&camp=1789&creative=9325)

[20 cm Jumper Wires](https://www.amazon.com/gp/product/B0B1ZYMFBJ/?&_encoding=UTF8&tag=radfordwill-20&linkCode=ur2&linkId=0f45c52f3e1e26fdbdd30c4379d4393b&camp=1789&creative=9325)

[Goolsky Mini 2 8Kg 3 Wire Metal Gear Steering Servo -optional-](https://www.amazon.com/gp/product/B07D75QZND?&_encoding=UTF8&tag=radfordwill-20&linkCode=ur2&linkId=5602e361dff0d8a37ccf5b37db0102aa&camp=1789&creative=9325)

[PS4 DualShock 4 Wireless Controller](https://amzn.to/409KCmI)




## Authors

- [@radfordwill](https://github.com/radfordwill)


## License

[MIT](https://choosealicense.com/licenses/mit/)

