import os
def requirements():
# 1) Update os system:
    os.system('sudo apt-get update')
    os.system('sudo apt-get upgrade')

# 2) Install dependencies:
    os.system('sudo apt-get install build-essential cmake pkg-config')
    os.system('sudo apt-get install libjpeg-dev libtiff5-dev libjasper-dev libpng12-dev')
    os.system('sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev')
    os.system('sudo apt-get install libxvidcore-dev libx264-dev')
    os.system('sudo apt-get install libgtk2.0-dev libgtk-3-dev')
    os.system('sudo apt-get install libatlas-base-dev gfortran')
    os.system('sudo apt-get install libqtgui4')
    os.system('sudo apt-get install libqt4-test')

# 3) Install Python 3 and Pip3:
    os.system('sudo apt-get install python3-dev')
    os.system('sudo apt-get install python3-pip')

# 4) Install Opencv:
    os.system('sudo pip3 install opencv-python')
    os.system('sudo pip install imutils')
    os.system('pip3 install dlib')
    os.system('pip3 install playsound')
    os.system('pip3 install scipy')

# 5) Enable Camera:
#    start -> preferences -> Raspberry Pi Configuration
#    -> Interfaces -> Enable Camera

# 6) Extra depencies for Opencv and the Camera:
#   sudo modprobe bcm2835-v4l2

requirements()