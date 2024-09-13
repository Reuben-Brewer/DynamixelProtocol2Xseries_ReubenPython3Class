########################

Dynamixel_Protocol2_Xseries_ReubenPython2and3Class

Wrapper (including ability to hook to Tkinter GUI) to control Dynamixel motors that use Protocol 2 (X series and possibly some other series).

Reuben Brewer, Ph.D.

reuben.brewer@gmail.com

www.reubotics.com

Apache 2 License

Software Revision K, 09/12/2024

Verified working on:
Python 3.11.
Windows 11 64-bit
Raspberry Pi Buster
(no Mac testing yet)

This code works ONLY for Dynamixel's Protocol 2 (X series and possibly some other series).

To use this code, you'll need to find the unique Serial Number for your U2D2, following the instructions in the included image "FindingTheSerialNumberOfU2D2inWindows.png".
When instantiating the class obect via Dynamixel_Protocol2_Xseries_ReubenPython2and3Class(DYNAMIXEL_X_setup_dict),
EACH INPUT LIST MUST BE THE SAME LENGTH AS THE NUMBER OF MOTORS, AND
EACH MOTOR'S NUMERICAL ID MUST BE IN ORDER FROM 0 T0 (NumberOfMotors - 1) (E.G. FOR 3 MOTORS, THE ID'S WOULD BE 0, 1, AND 2).

You will also need to configure your Dynamixel motor (including Motor ID and Baud Rate) within R+Manager.exe (downloaded from the Robotis website).
Note that there is another interface from Robotis (Dynamixel Wizard 2.0) for Protocol 2 motors, but I have not yet tested if it works with Protocol 2 motors.
Even though the stated-maximum baud rate for Protocol 2 motors is 4500000 (4.5MBPS), that doesn't work. The max I've had succeed is 4000000 (4MBPS).

IMPORTANT NOTE:

If ENABLE_GETS is 1 and you're not receiving updated information from the Dynamixel,
double-check that you don't have more than 1 type of motor hooked up to the U2D2 simultaneously.
For instance, if both an XM-series (protocol 2) and AX-series (protocol 1) motor are hooked up to the U2D2 at the same time,
you won't be able to receive any updates from motors (but the motors themselves will move, albeit with more jitter).

########################  

########################### Installation instructions

This code isn't "installed" like a typical python module (no "pip install MyModule"). If you want to call it in your own code, you'll need to import a local copy of the files.

###

DynamixelProtocol2Xseries_ReubenPython3Class, ListOfModuleDependencies: ['dynamixel_sdk', 'ftd2xx', 'LowPassFilter_ReubenPython2and3Class', 'serial', 'serial.tools']

DynamixelProtocol2Xseries_ReubenPython3Class, ListOfModuleDependencies_TestProgram: ['MyPlotterPureTkinterStandAloneProcess_ReubenPython2and3Class', 'MyPrint_ReubenPython2and3Class']

DynamixelProtocol2Xseries_ReubenPython3Class, ListOfModuleDependencies_NestedLayers: ['future.builtins', 'numpy', 'pexpect', 'psutil']

DynamixelProtocol2Xseries_ReubenPython3Class, ListOfModuleDependencies_All:['dynamixel_sdk', 'ftd2xx', 'future.builtins', 'LowPassFilter_ReubenPython2and3Class', 'MyPlotterPureTkinterStandAloneProcess_ReubenPython2and3Class', 'MyPrint_ReubenPython2and3Class', 'numpy', 'pexpect', 'psutil', 'serial', 'serial.tools']

###

###

Windows and Raspberry Pi:
pip install pyserial

###

###  dynamixel_SDK

http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/library_setup/python_windows/#building-the-library

#Must install manually from .\DynamixelSDK-3.7.21\python using "sudo python setup.py install".
NOTE THAT IN YOU MUST ISSUE THIS COMMAND FROM A TERMINAL WITHIN THIS FOLDER OR ELSE IT WILL FAIL (THE SETUP.PY HAS TO BE A LOCAL FILE WITHIN THAT FOLDER, NOT HAVE A LONG PATH TO IT).

Note that "pip show dynamixel_sdk" says it's version 3.6.0, but it's actually 3.7.21.
Note also that Reuben modified some code in "port_handler.py" before installation (the default baud and latency_timer).

To test the installation, make sure that "import dynamixel_sdk" succeeds within Python.

###

###

Set USB-Serial latency_timer

In Windows:

Manual method:
Follow the instructions in the included image "LatencyTimer_SetManuallyInWindows.png".

Automated method:
python LatencyTimer_DynamixelU2D2_ReadAndWrite_Windows.py

In Linux (including Raspberry Pi):
Run the included script:
sudo dos2unix LatencyTimer_Set_LinuxScript.sh
sudo chmod 777 LatencyTimer_Set_LinuxScript.sh
sudo ./LatencyTimer_Set_LinuxScript.sh 1

###

###ftd2xx

Windows:
If the required driver is already on your Windows machine, then it will be installed automatically when the U2D2 is first plugged-in (note that this installation will occur separately for EACH separate USB port, and that the latency_timer will need to be set for EACH separate USB port.
However, if you don't see a new USB-Serial device appearing with a new "COM" number in Device Manger after Windows says it's done installing your new device, then you'll need to install the driver separately using Windows_FTDI_USBserial_driver_061020-->CDM21228_Setup.exe

To install the Python module:

pip3 install ftd2xx==1.0 (this 1.0 is VERY IMPORTANT as the later versions appear to have issues, including when installed from the whl file 'pip install C:\ftd2xx-1.1.2-py2-none-any.whl').

Raspberry Pi:

To install the Python module:

sudo pip3 install ftd2xx==1.0

####### IMPORTANT

sudo gedit /usr/local/lib/python3.7/dist-packages/ftd2xx/ftd2xx.py

Add the lines:
"elif sys.platform == 'linux':
    
	from . import _ftd2xx_linux as _ft"
right before the linux2 line (otherwise it breaks when run in Python 3).
Alternatively, you can copy Reuben's version of this file (ftd2xx_ReubenModified.py) to /usr/local/lib/python3.7/dist-packages/ftd2xx/ftd2xx.py.

#######

To install the driver:
Download the 1.4.6 ARMv6 hard-float (suits Raspberry Pi) source code from ftdi (http://www.ftdichip.com/Drivers/D2XX.htm) or use the included file. 
Install following these instructions (modified from the readme that comes with the driver):

tar --extract --file libftd2xx-arm-v6-hf-1.4.6.tgz 
cd release
cd build
sudo -s (become root)
cp libftd2xx.* /usr/local/lib
chmod 0755 /usr/local/lib/libftd2xx.so.1.4.6
ln -sf /usr/local/lib/libftd2xx.so.1.4.6 /usr/local/lib/libftd2xx.so
exit
THIS LAST STEP ISN'T IN THE READ ME BUT IS CRITICAL: 'sudo ldconfig' so that your code can find the new library.

###

########################### 
