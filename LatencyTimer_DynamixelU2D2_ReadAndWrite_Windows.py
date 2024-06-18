# -*- coding: utf-8 -*-

'''
Reuben Brewer, Ph.D.
reuben.brewer@gmail.com
www.reubotics.com

Apache 2 License
Software Revision D, 03/13/2022

Verified working on: Python 2.7, 3.8 for Windows 8.1, 10 64-bit and Raspberry Pi Buster (no Mac testing yet).
'''

import sys
from sys import platform as _platform
import os
import time
import traceback

import elevate #Elevates your python's script's permission to "Admin". IF YOU DON'T WANT TO DEAL WITH THE UAC POP-UP WINDOW, YOU CAN DISABLE IT.
#import commandRunner #https://pypi.org/project/commandRunner/#files From commandRunner-0.8.12.tar.gz, run "python setup.py install" (pip fails)

if _platform == "win32":
    from _winreg import *

#######################################################################################################################
#######################################################################################################################
def ReadLatencyTimer():
    global my_platform
    global DynamixelU2D2_SerialNumber
    global LatencyTimerWindowsRegistryKeyPath_Str

    try:
        if my_platform == "windows":

            print("*** Reading value from Registry path '" + LatencyTimerWindowsRegistryKeyPath_Str + "' ***")
            LatencyTimerWindowsRegistryHandle = ConnectRegistry(None,HKEY_LOCAL_MACHINE)
            LatencyTimerWindowsKeyHandleObject = OpenKey(LatencyTimerWindowsRegistryHandle, LatencyTimerWindowsRegistryKeyPath_Str, 0, KEY_READ)

            LatencyTimerValueInMilliseconds_QueriedFromRegistry = QueryValueEx(LatencyTimerWindowsKeyHandleObject, "LatencyTimer")[0] #Returns a tuple, just take the first element.
            print("ReadLatencyTimer(): LatencyTimerValueInMilliseconds_QueriedFromRegistry: " + str(LatencyTimerValueInMilliseconds_QueriedFromRegistry) + " milliseconds.")

            CloseKey(LatencyTimerWindowsKeyHandleObject)

            return LatencyTimerValueInMilliseconds_QueriedFromRegistry

        elif my_platform == "pi":
            pass

        else:
            pass

    except:
        exceptions = sys.exc_info()[0]
        print("ReadLatencyTimer(): Exceptions: %s" % exceptions, 0)
        traceback.print_exc()
#######################################################################################################################
#######################################################################################################################

#######################################################################################################################
#######################################################################################################################
def WriteLatencyTimer(LatencyTimerValueInMilliseconds_ToWrite):
    global my_platform
    global DynamixelU2D2_SerialNumber
    global LatencyTimerWindowsRegistryKeyPath_Str

    try:
        if my_platform == "windows":

            LatencyTimerWindowsRegistryHandle = ConnectRegistry(None, HKEY_LOCAL_MACHINE)
            LatencyTimerWindowsKeyHandleObject = OpenKey(LatencyTimerWindowsRegistryHandle, LatencyTimerWindowsRegistryKeyPath_Str, 0, KEY_ALL_ACCESS)

            MyReservedValue = 0
            print("*** Writing value to Registry path '" + LatencyTimerWindowsRegistryKeyPath_Str + "' ***")
            SetValueEx(LatencyTimerWindowsKeyHandleObject, "LatencyTimer", MyReservedValue, REG_DWORD, LatencyTimerValueInMilliseconds_ToWrite)

            CloseKey(LatencyTimerWindowsKeyHandleObject)

            print("WriteLatencyTimer(): Wrote LatencyTimer to a value of " + str(LatencyTimerValueInMilliseconds_ToWrite))

        elif my_platform == "pi":
            pass

        else:
            pass

    except:
        exceptions = sys.exc_info()[0]
        print("WriteLatencyTimer(): Exceptions: %s" % exceptions, 0)
        traceback.print_exc()
#######################################################################################################################
#######################################################################################################################

#######################################################################################################################
#######################################################################################################################
if __name__ == '__main__':

    ############################################################### RASPBERRY PI VS WINDOWS CODE 1
    global my_platform
    my_platform = str("default")

    if _platform == "linux":
        my_platform = "linux"
    elif _platform == "linux2":
        my_platform = "pi"
    elif _platform == "win32":
        my_platform = "windows"
    else:
        my_platform = "other"

    print("The OS platform is: " + my_platform)
    ###############################################################

    LatencyTimerValueInMilliseconds_ToWrite = 8

    ############################
    global DynamixelU2D2_SerialNumber
    DynamixelU2D2_SerialNumber = "FT3M9STOA" #SerialNumber like 'FT3M9STOA'
    ############################

    ############################
    global LatencyTimerWindowsRegistryKeyPath_Str
    LatencyTimerWindowsRegistryKeyPath_Str = r"SYSTEM\CurrentControlSet\Enum\FTDIBUS\\VID_0403+PID_6014+" + DynamixelU2D2_SerialNumber + "\\0000\Device Parameters"
    ############################

    ############################ Read BEFORE setting a new value
    LatencyTimerValueInMilliseconds_QueriedFromRegistry = ReadLatencyTimer()
    print("LatencyTimerValueInMilliseconds_QueriedFromRegistry: " + str(LatencyTimerValueInMilliseconds_QueriedFromRegistry))
    ############################

    ############################
    if my_platform == "windows":
        elevate.elevate(show_console=True, graphical=True) #Not sure why, but this 'elevate.elevate' fails when called from inside of WriteLatencyTimer()
    ############################

    ############################ Write a new value
    WriteLatencyTimer(LatencyTimerValueInMilliseconds_ToWrite) #WILL NOT PRINT ANYTHING EXCEPT TO THE ELEVATED CONSOLE
    ############################

    ############################ Read AFTER setting a new value
    LatencyTimerValueInMilliseconds_QueriedFromRegistry = ReadLatencyTimer()
    print("LatencyTimerValueInMilliseconds_QueriedFromRegistry: " + str(LatencyTimerValueInMilliseconds_QueriedFromRegistry))
    ############################

#######################################################################################################################
#######################################################################################################################

