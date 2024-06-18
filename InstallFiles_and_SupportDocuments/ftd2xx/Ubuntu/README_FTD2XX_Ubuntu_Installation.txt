########################### ftd2xx installation instructions on Ubuntu (a very painful and buggy process which you might want to skip)

https://ftdichip.com/wp-content/uploads/2020/08/AN_220_FTDI_Drivers_Installation_Guide_for_Linux-1.pdf

https://ftdichip.com/drivers/d2xx-drivers/

https://ftdichip.com/wp-content/uploads/2022/07/libftd2xx-x86_64-1.4.27.tgz

sudo rmmod ftdi_sio #MIGHT HAVE TO DO THIS AFTER REBOOT
sudo rmmod usbserial #MIGHT HAVE TO DO THIS AFTER REBOOT
extract libftd2xx-x86_64-1.4.27.tgz #will put "release" folder onto the Desktop
sudo chmod 777 -R ~/Desktop/
sudo cp release/build/lib* /usr/local/lib

cd /usr/local/lib
sudo ln -s libftd2xx.so.1.4.27 libftd2xx.so
sudo chmod 0755 libftd2xx.so.1.4.27
sudo ldconfig #otherwise can’t find libftd2xx.so

check for cable: lsusb -v | grep "FT"

NOTE: CURRENTLY YOU HAVE TO USE SUDO FOR FTD2XX
###########################