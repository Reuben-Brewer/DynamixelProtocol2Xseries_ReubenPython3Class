# -*- coding: utf-8 -*-

'''
Reuben Brewer, Ph.D.
reuben.brewer@gmail.com
www.reubotics.com

Apache 2 License
Software Revision L, 10/17/2024

Verified working on: Python 3.12 for Windows 11 64-bit and Raspberry Pi Buster (no Mac testing yet).
'''

__author__ = 'reuben.brewer'

##########################################
from LowPassFilter_ReubenPython2and3Class import *
##########################################

##########################################
import os
import sys
from sys import platform as _platform
import time
import datetime
import threading
import collections
from copy import * #for deepcopy
import traceback
##########################################

##########################################
from tkinter import *
import tkinter.font as tkFont
from tkinter import ttk
##########################################

##########################################
import platform
if platform.system() == "Windows":
    import ctypes
    winmm = ctypes.WinDLL('winmm')
    winmm.timeBeginPeriod(1) #Set minimum timer resolution to 1ms so that time.sleep(0.001) behaves properly.
##########################################

##########################################
import serial
from serial.tools import list_ports
##########################################

##########################################
global ftd2xx_IMPORTED_FLAG
ftd2xx_IMPORTED_FLAG = 0
try:
    import ftd2xx #https://pypi.org/project/ftd2xx/ 'pip install ftd2xx', current version is 1.3.1 as of 05/06/22. For SetAllFTDIdevicesLatencyTimer function
    ftd2xx_IMPORTED_FLAG = 1

except:
    exceptions = sys.exc_info()[0]
    print("**********")
    print("********** DynamixelProtocol2Xseries_ReubenPython3Class __init__: ERROR, failed to import ftdtxx, Exceptions: %s" % exceptions + " ********** ")
    print("**********")
##########################################

##########################################
#Install via "pip install dynamixel_sdk".
#On 10/17/24, used version dynamixel_sdk-3.7.31, installed to C:\Anaconda3\Lib\site-packages\dynamixel_sdk
#Note also that Reuben modified some code in "port_handler.py" after installation.
#LATENCY_TIMER = 1
#DEFAULT_BAUDRATE = 4000000

from dynamixel_sdk import *
##########################################

########################################## CLASS VARIABLE DECLARATIONS
angular_units_acceptable_list = ["raw", "dynamixelunits", "rad", "deg", "rev"]
angular_speed_units_acceptable_list = ["raw", "dynamixelunits", "radpersec", "degpersec", "revpersec", "revpermin"]
current_units_acceptable_list = ["raw", "dynamixelunits", "milliamps", "amps", "percent"]
OperatingMode_AcceptableStringValuesList = ["CurrentControl", "VelocityControl", "PositionControl", "ExtendedPositionControlMultiTurn", "CurrentBasedPositionControl", "PWMcontrol"]
##########################################

#http://stackoverflow.com/questions/19087515/subclassing-tkinter-to-create-a-custom-widget
class DynamixelProtocol2Xseries_ReubenPython3Class(Frame): #Subclass the Tkinter Frame

    ##########################################################################################################
    ##########################################################################################################
    def __init__(self, setup_dict): #Subclass the Tkinter Frame

        #########################################################
        #########################################################
        if sys.version_info[0] < 3:
            print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: Error, this code can only be run on Python 3 (not 2)!")
            return
        #########################################################
        #########################################################

        #########################################################
        #########################################################
        if platform.system() == "Linux":

            if "raspberrypi" in platform.uname(): #os.uname() doesn't work in windows
                self.my_platform = "pi"
            else:
                self.my_platform = "linux"

        elif platform.system() == "Windows":
            self.my_platform = "windows"

        elif platform.system() == "Darwin":
            self.my_platform = "mac"

        else:
            self.my_platform = "other"

        print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: The OS platform is: " + self.my_platform)
        #########################################################
        #########################################################

        #########################################################
        #########################################################
        self.EXIT_PROGRAM_FLAG = 0
        self.OBJECT_CREATED_SUCCESSFULLY_FLAG = 0
        self.MainThread_still_running_flag = 0
        self.GUI_ready_to_be_updated_flag = 0

        self.SerialObject = serial.Serial()
        self.SerialConnectedFlag = 0
        self.SerialTimeout_Rx_Seconds = 0.002 #This worked at 0.25 for MX series but needed to be set very low for AX-series (0.002)
        self.SerialTimeout_Tx_Seconds = 0.25
        self.SerialParity = serial.PARITY_NONE
        self.SerialStopBits = serial.STOPBITS_ONE
        self.SerialByteSize = serial.EIGHTBITS
        self.SerialXonXoffSoftwareFlowControl = 0  #ABSOLUTELY MUST BE 0 FOR U2D2 (UNLIKE ROBOTEQ). IF SET TO 1, WILL HAVE PROBLEMS READING WITHOUT DISCONNECTING,
        self.SerialPortNameCorrespondingToCorrectSerialNumber = "default"
        self.MainThread_StillRunningFlag = 0
        self.ResetSerialConnection_EventNeedsToBeFiredFlag = 0

        self.TimeBetweenCommands = 0.001

        self.MostRecentDataDict = dict([("Time", -11111.0)])
        self.MostRecentDataDict_ForExternalQueryAndGUIdisplayOnly = dict()
        self.MostRecentDataDict_ForExternalQueryAndGUIdisplayOnly_FullyPopulatedLength = 23 #MUST UPDATE THIS NUMBER EVERY TIME YOU CHANGE THE NUMBER OF KEYS IN MostRecentDataDict
        self.MostRecentDataDict_ForExternalQueryAndGUIdisplayOnly_BlockExternalCopyingFlag = 0

        self.OperatingMode_AcceptableList = ["CurrentControl",
                                             "VelocityControl",
                                             "PositionControl",
                                             "ExtendedPositionControlMultiTurn",
                                             "CurrentBasedPositionControl",
                                             "PWMcontrol"]

        self.ListOfAcceptableVariableNameStringsForReading = ["Baud",
                                             "OperatingMode",
                                             "TemperatureLimit",
                                             "MaxVoltageLimit",
                                             "MinVoltageLimit",
                                             "PWMlimit",
                                             "CurrentLimit",
                                             "AccelerationLimit",
                                             "VelocityLimit",
                                             "HardwareErrorStatus",
                                             "RealtimeTick",
                                             "Moving",
                                             "MovingStatus",
                                             "PresentPWM",
                                             "PresentCurrent",
                                             "PresentVelocity",
                                             "PresentPosition",
                                             "PresentInputVoltage",
                                             "PresentTemperature",
                                             "Shutdown"]



        self.MotorType_AcceptableDict = dict([("None", dict([("MotorType_DynamixelInteger", -1)])),
                                                     ("NONE", dict([("MotorType_DynamixelInteger", -1), ("MotorTorqueConstant_Dict", dict([("PolynomialFitMotorCurrentToMotorTorque", []), ("Current_MinValueInputToPolyfit", 0.0), ("Current_MaxValueInputToPolyfit", 0.0)]))])),
                                                     ("XC330-181-T", dict([("MotorType_DynamixelInteger", 0), ("MotorTorqueConstant_Dict", dict([("PolynomialFitMotorCurrentToMotorTorque", [-0.53234, 1.0999, -0.12041]), ("Current_MinValueInputToPolyfit", 0.20), ("Current_MaxValueInputToPolyfit", 0.63)]))])),
                                                     ("XC330-288-T", dict([("MotorType_DynamixelInteger", 1), ("MotorTorqueConstant_Dict", dict([("PolynomialFitMotorCurrentToMotorTorque", [-0.53234, 1.0999, -0.12041]), ("Current_MinValueInputToPolyfit", 0.16), ("Current_MaxValueInputToPolyfit", 0.73)]))])),
                                                     ("XM540", dict([("MotorType_DynamixelInteger", 2), ("MotorTorqueConstant_Dict", dict([("PolynomialFitMotorCurrentToMotorTorque", []), ("Current_MinValueInputToPolyfits", 0.0), ("Current_MaxValueInputToPolyfit", 0.0)]))]))])
        #########################################################
        #########################################################

        #########################################################
        #########################################################
        if "GUIparametersDict" in setup_dict:
            self.GUIparametersDict = setup_dict["GUIparametersDict"]

            #########################################################
            #########################################################
            if "USE_GUI_FLAG" in self.GUIparametersDict:
                self.USE_GUI_FLAG = self.PassThrough0and1values_ExitProgramOtherwise("USE_GUI_FLAG", self.GUIparametersDict["USE_GUI_FLAG"])
            else:
                self.USE_GUI_FLAG = 0

            print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: USE_GUI_FLAG: " + str(self.USE_GUI_FLAG))
            #########################################################
            #########################################################

            #########################################################
            #########################################################
            if "root" in self.GUIparametersDict:
                self.root = self.GUIparametersDict["root"]
            else:
                print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: ERROR, must pass in 'root'")
                return
            #########################################################
            #########################################################

            #########################################################
            #########################################################
            if "EnableInternal_MyPrint_Flag" in self.GUIparametersDict:
                self.EnableInternal_MyPrint_Flag = self.PassThrough0and1values_ExitProgramOtherwise("EnableInternal_MyPrint_Flag", self.GUIparametersDict["EnableInternal_MyPrint_Flag"])
            else:
                self.EnableInternal_MyPrint_Flag = 0

            print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: EnableInternal_MyPrint_Flag: " + str(self.EnableInternal_MyPrint_Flag))
            #########################################################
            #########################################################

            #########################################################
            #########################################################
            if "PrintToConsoleFlag" in self.GUIparametersDict:
                self.PrintToConsoleFlag = self.PassThrough0and1values_ExitProgramOtherwise("PrintToConsoleFlag", self.GUIparametersDict["PrintToConsoleFlag"])
            else:
                self.PrintToConsoleFlag = 1

            print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: PrintToConsoleFlag: " + str(self.PrintToConsoleFlag))
            #########################################################
            #########################################################

            #########################################################
            #########################################################
            if "NumberOfPrintLines" in self.GUIparametersDict:
                self.NumberOfPrintLines = int(self.PassThroughFloatValuesInRange_ExitProgramOtherwise("NumberOfPrintLines", self.GUIparametersDict["NumberOfPrintLines"], 0.0, 50.0))
            else:
                self.NumberOfPrintLines = 10

            print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: NumberOfPrintLines: " + str(self.NumberOfPrintLines))
            #########################################################
            #########################################################

            #########################################################
            #########################################################
            if "UseBorderAroundThisGuiObjectFlag" in self.GUIparametersDict:
                self.UseBorderAroundThisGuiObjectFlag = self.PassThrough0and1values_ExitProgramOtherwise("UseBorderAroundThisGuiObjectFlag", self.GUIparametersDict["UseBorderAroundThisGuiObjectFlag"])
            else:
                self.UseBorderAroundThisGuiObjectFlag = 0

            print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: UseBorderAroundThisGuiObjectFlag: " + str(self.UseBorderAroundThisGuiObjectFlag))
            #########################################################
            #########################################################

            #########################################################
            #########################################################
            if "GUI_ROW" in self.GUIparametersDict:
                self.GUI_ROW = int(self.PassThroughFloatValuesInRange_ExitProgramOtherwise("GUI_ROW", self.GUIparametersDict["GUI_ROW"], 0.0, 1000.0))
            else:
                self.GUI_ROW = 0

            print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: GUI_ROW: " + str(self.GUI_ROW))
            #########################################################
            #########################################################

            #########################################################
            #########################################################
            if "GUI_COLUMN" in self.GUIparametersDict:
                self.GUI_COLUMN = int(self.PassThroughFloatValuesInRange_ExitProgramOtherwise("GUI_COLUMN", self.GUIparametersDict["GUI_COLUMN"], 0.0, 1000.0))
            else:
                self.GUI_COLUMN = 0

            print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: GUI_COLUMN: " + str(self.GUI_COLUMN))
            #########################################################
            #########################################################

            #########################################################
            #########################################################
            if "GUI_PADX" in self.GUIparametersDict:
                self.GUI_PADX = int(self.PassThroughFloatValuesInRange_ExitProgramOtherwise("GUI_PADX", self.GUIparametersDict["GUI_PADX"], 0.0, 1000.0))
            else:
                self.GUI_PADX = 0

            print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: GUI_PADX: " + str(self.GUI_PADX))
            #########################################################
            #########################################################

            #########################################################
            #########################################################
            if "GUI_PADY" in self.GUIparametersDict:
                self.GUI_PADY = int(self.PassThroughFloatValuesInRange_ExitProgramOtherwise("GUI_PADY", self.GUIparametersDict["GUI_PADY"], 0.0, 1000.0))
            else:
                self.GUI_PADY = 0

            print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: GUI_PADY: " + str(self.GUI_PADY))
            #########################################################
            #########################################################

            ##########################################
            if "GUI_ROWSPAN" in self.GUIparametersDict:
                self.GUI_ROWSPAN = int(self.PassThroughFloatValuesInRange_ExitProgramOtherwise("GUI_ROWSPAN", self.GUIparametersDict["GUI_ROWSPAN"], 1.0, 1000.0))
            else:
                self.GUI_ROWSPAN = 1

            print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: GUI_ROWSPAN: " + str(self.GUI_ROWSPAN))
            #########################################################
            #########################################################

            #########################################################
            #########################################################
            if "GUI_COLUMNSPAN" in self.GUIparametersDict:
                self.GUI_COLUMNSPAN = int(self.PassThroughFloatValuesInRange_ExitProgramOtherwise("GUI_COLUMNSPAN", self.GUIparametersDict["GUI_COLUMNSPAN"], 1.0, 1000.0))
            else:
                self.GUI_COLUMNSPAN = 1

            print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: GUI_COLUMNSPAN: " + str(self.GUI_COLUMNSPAN))
            #########################################################
            #########################################################

            #########################################################
            #########################################################
            if "GUI_STICKY" in self.GUIparametersDict:
                self.GUI_STICKY = str(self.GUIparametersDict["GUI_STICKY"])
            else:
                self.GUI_STICKY = "w"

            print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: GUI_STICKY: " + str(self.GUI_STICKY))
            #########################################################
            #########################################################

        else:
            self.GUIparametersDict = dict()
            self.USE_GUI_FLAG = 0
            print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: No GUIparametersDict present, setting USE_GUI_FLAG: " + str(self.USE_GUI_FLAG))

        #print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: GUIparametersDict: " + str(self.GUIparametersDict))
        #########################################################
        #########################################################

        #########################################################
        #########################################################
        #########################################################
        #########################################################

        #########################################################
        #########################################################
        #########################################################
        self.MotorType_DynamixelIntegerList = []
        self.MotorTorqueConstant_ListOfDicts = []

        #########################################################
        #########################################################
        if "MotorType_StringList" not in setup_dict:
            print("DynamixelProtocol2Xseries_ReubenPython3Class ERROR: Must initialize object with 'MotorType_StringList' argument.")
            return
        #########################################################
        #########################################################

        #########################################################
        #########################################################
        else:
            self.MotorType_StringList = setup_dict["MotorType_StringList"]
            self.NumberOfMotors = len(self.MotorType_StringList)

            for index, MotorType_element in enumerate(self.MotorType_StringList):
                #########################################################
                if MotorType_element not in self.MotorType_AcceptableDict:
                    print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: Error, MotorType of " + str(MotorType_element) + " is not a supported type.")
                    MotorType_element = "NONE"

                servo_type_dict_temp = self.MotorType_AcceptableDict[MotorType_element]
                self.MotorType_DynamixelIntegerList.append(servo_type_dict_temp["MotorType_DynamixelInteger"])
                self.MotorTorqueConstant_ListOfDicts.append(servo_type_dict_temp["MotorTorqueConstant_Dict"])
                #########################################################

        #########################################################
        #########################################################

        #########################################################
        #########################################################
        #########################################################

        #########################################################
        #########################################################
        #########################################################

        #########################################################
        #########################################################
        if self.NumberOfMotors == 0:
            print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: Error, 'MotorType_StringList' argument must be a list of length >= 1.")
            return
        #########################################################
        #########################################################

        #########################################################
        #########################################################
        else:

            #########################################################
            for element in self.MotorType_DynamixelIntegerList:
                if element != self.MotorType_DynamixelIntegerList[0]:
                    print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: Error, all elements in the list 'MotorType_StringList' must be the same type")
                    return
            #########################################################

        #########################################################
        #########################################################

        #########################################################
        #########################################################
        #########################################################
        print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: NumberOfMotors: " + str(self.NumberOfMotors))
        print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: MotorType_StringList: " + str(self.MotorType_StringList))
        print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: MotorType_DynamixelIntegerList: " + str(self.MotorType_DynamixelIntegerList))
        print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: MotorTorqueConstant_ListOfDicts: " + str(self.MotorTorqueConstant_ListOfDicts))
        #########################################################
        #########################################################
        #########################################################
        #########################################################

        #########################################################
        #########################################################
        if "DesiredSerialNumber_USBtoSerialConverter" in setup_dict:
            self.DesiredSerialNumber_USBtoSerialConverter = setup_dict["DesiredSerialNumber_USBtoSerialConverter"]

        else:
            print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: ERROR, must initialize object with 'DesiredSerialNumber_USBtoSerialConverter' argument.")
            return
        #########################################################
        #########################################################

        #########################################################
        #########################################################
        if "NameToDisplay_UserSet" in setup_dict:
            self.NameToDisplay_UserSet = setup_dict["NameToDisplay_UserSet"]
        else:
            self.NameToDisplay_UserSet = ""

        print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: NameToDisplay_UserSet" + str(self.NameToDisplay_UserSet))
        #########################################################
        #########################################################

        #########################################################
        #########################################################
        if "SerialBaudRate" in setup_dict:
            self.SerialBaudRate = int(self.PassThroughFloatValuesInRange_ExitProgramOtherwise("SerialBaudRate", setup_dict["SerialBaudRate"], 9600, 4500000))

        else:
            self.SerialBaudRate = 4000000

        print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: SerialBaudRate: " + str(self.SerialBaudRate))
        #########################################################
        #########################################################

        #########################################################
        #########################################################
        if "SerialRxBufferSize" in setup_dict:
            self.SerialRxBufferSize = round(self.PassThroughFloatValuesInRange_ExitProgramOtherwise("SerialRxBufferSize", setup_dict["SerialRxBufferSize"], 0.0, 4096.0)) #Maybe 64 to 4096

        else:
            self.SerialRxBufferSize = 4096

        print("DynamixelProtocol2Xseries_ReubenPython3Class: SerialRxBufferSize: " + str(self.SerialRxBufferSize))
        #########################################################
        #########################################################

        #########################################################
        #########################################################
        if "SerialTxBufferSize" in setup_dict:
            self.SerialTxBufferSize = round(self.PassThroughFloatValuesInRange_ExitProgramOtherwise("SerialTxBufferSize", setup_dict["SerialTxBufferSize"], 0.0, 4096.0)) #Maybe 64 to 4096

        else:
            self.SerialTxBufferSize = 4096

        print("DynamixelProtocol2Xseries_ReubenPython3Class: SerialTxBufferSize: " + str(self.SerialTxBufferSize))
        #########################################################
        #########################################################

        #########################################################
        #########################################################
        if "WatchdogTimeIntervalMilliseconds" in setup_dict:
            WatchdogTimeIntervalMilliseconds_TEMP = setup_dict["WatchdogTimeIntervalMilliseconds"]
            if int(WatchdogTimeIntervalMilliseconds_TEMP) in [-1, 0]:
                self.WatchdogTimeIntervalMilliseconds = WatchdogTimeIntervalMilliseconds_TEMP
            else:
                self.WatchdogTimeIntervalMilliseconds = self.PassThroughFloatValuesInRange_ExitProgramOtherwise("WatchdogTimeIntervalMilliseconds", WatchdogTimeIntervalMilliseconds_TEMP, 20.0*1, 20.0*127) 

        else:
            self.WatchdogTimeIntervalMilliseconds = -1 #Off

        print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: WatchdogTimeIntervalMilliseconds: " + str(self.WatchdogTimeIntervalMilliseconds))
        #########################################################
        #########################################################

        #########################################################
        #########################################################
        if "MainThread_TimeToSleepEachLoop" in setup_dict:
            self.MainThread_TimeToSleepEachLoop = self.PassThroughFloatValuesInRange_ExitProgramOtherwise("MainThread_TimeToSleepEachLoop", setup_dict["MainThread_TimeToSleepEachLoop"], 0.001, 100000)

        else:
            self.MainThread_TimeToSleepEachLoop = 0.002

        print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: MainThread_TimeToSleepEachLoop: " + str(self.MainThread_TimeToSleepEachLoop))
        #########################################################
        #########################################################

        #########################################################
        #########################################################
        if "EnableSafetyShutoff" in setup_dict:
            EnableSafetyShutoff_TEMP = self.PassThrough0and1values_ExitProgramOtherwise("EnableSafetyShutoff", setup_dict["EnableSafetyShutoff"])
        else:
            EnableSafetyShutoff_TEMP = 1

        self.EnableSafetyShutoff = [EnableSafetyShutoff_TEMP]*self.NumberOfMotors

        print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: self.EnableSafetyShutoff: " + str(self.EnableSafetyShutoff))
        #########################################################
        #########################################################

        #########################################################
        #########################################################
        if "ENABLE_SETS" in setup_dict:
            self.ENABLE_SETS = self.PassThrough0and1values_ExitProgramOtherwise("ENABLE_SETS", setup_dict["ENABLE_SETS"])
        else:
            self.ENABLE_SETS = 0

        print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: ENABLE_SETS: " + str(self.ENABLE_SETS))
        #########################################################
        #########################################################

        #########################################################
        #########################################################
        if "GetVariablesEveryNloopsCycles" in setup_dict:
            self.GetVariablesEveryNloopsCycles = self.PassThroughFloatValuesInRange_ExitProgramOtherwise("GetVariablesEveryNloopsCycles", setup_dict["GetVariablesEveryNloopsCycles"], 0, 1000000000)
        else:
            self.GetVariablesEveryNloopsCycles = -1

        print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: GetVariablesEveryNloopsCycles: " + str(self.GetVariablesEveryNloopsCycles))
        #########################################################
        #########################################################

        #########################################################
        #########################################################
        self.ListOfVariableNameStringsToGet = []
        if "ListOfVariableNameStringsToGet" in setup_dict:
            ListOfVariableNameStringsToGet_temp = setup_dict["ListOfVariableNameStringsToGet"]

            for VariableNameString in ListOfVariableNameStringsToGet_temp:
                if VariableNameString in self.ListOfAcceptableVariableNameStringsForReading:
                    self.ListOfVariableNameStringsToGet.append(VariableNameString)

                    if VariableNameString not in self.MostRecentDataDict:
                        self.MostRecentDataDict[VariableNameString] = [0] * self.NumberOfMotors
                else:
                    print("ERROR: VariableNameString no an acceptable value.")

        print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: ListOfVariableNameStringsToGet: " + str(self.ListOfVariableNameStringsToGet))
        #########################################################
        #########################################################

        #########################################################
        #########################################################
        self.MotorName_StringList = []
        if "MotorName_StringList" in setup_dict:
            MotorName_StringList_temp = setup_dict["MotorName_StringList"]
            print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: ########################## " + str(MotorName_StringList_temp) + " ########################## ")

            try:
                for element in MotorName_StringList_temp:
                    self.MotorName_StringList.append(str(element))
            except:
                exceptions = sys.exc_info()[0]
                print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: MotorName_StringList coldn't convert to string, Exceptions: %s" % exceptions)
                self.MotorName_StringList = [""] * self.NumberOfMotors
        else:
            self.MotorName_StringList = [""] * self.NumberOfMotors

        print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: MotorName_StringList: " + str(self.MotorName_StringList))
        #########################################################
        #########################################################

        #########################################################
        #########################################################
        self.OperatingMode_StartingValueList = []
        self.Position_DynamixelUnits_Min_FWlimit = []
        self.Position_DynamixelUnits_Max_FWlimit = []
        self.Velocity_DynamixelUnits_Min_FWlimit = []
        self.Velocity_DynamixelUnits_Max_FWlimit = []
        self.ConversionFactorFromDynamixelUnitsToDegrees = []
        self.ConversionFactorFromDegreesToDynamixelUnits = []

        Position_DynamixelUnits_Max_FWlimit_SingleTurn = 4095.0
        Position_DynamixelUnits_Min_FWlimit_SingleTurn = 0.0

        Position_DynamixelUnits_Max_FWlimit_MultiTurn = 1048575
        Position_DynamixelUnits_Min_FWlimit_MultiTurn = -1048575

        Velocity_DynamixelUnits_Max_FWlimit = 2047.0
        Velocity_DynamixelUnits_Min_FWlimit = 1.0 #0.0 would stop the motor

        if "OperatingMode_StartingValueList" in setup_dict:
            OperatingMode_StartingValueList_temp = setup_dict["OperatingMode_StartingValueList"]

            for index, element in enumerate(OperatingMode_StartingValueList_temp):
                if element in OperatingMode_AcceptableStringValuesList:

                    self.OperatingMode_StartingValueList.append(str(element))

                    if element == "ExtendedPositionControlMultiTurn" or element == "CurrentBasedPositionControl":
                        self.Position_DynamixelUnits_Min_FWlimit.append(Position_DynamixelUnits_Min_FWlimit_MultiTurn)
                        self.Position_DynamixelUnits_Max_FWlimit.append(Position_DynamixelUnits_Max_FWlimit_MultiTurn)
                    else:
                        self.Position_DynamixelUnits_Min_FWlimit.append(Position_DynamixelUnits_Min_FWlimit_SingleTurn)
                        self.Position_DynamixelUnits_Max_FWlimit.append(Position_DynamixelUnits_Max_FWlimit_SingleTurn)

                    self.Velocity_DynamixelUnits_Min_FWlimit.append(Velocity_DynamixelUnits_Min_FWlimit)
                    self.Velocity_DynamixelUnits_Max_FWlimit.append(Velocity_DynamixelUnits_Max_FWlimit)

                    self.ConversionFactorFromDynamixelUnitsToDegrees.append(360.0 / Position_DynamixelUnits_Max_FWlimit_SingleTurn) #Doesn't change for SingleTurn vs MultiTurn
                    self.ConversionFactorFromDegreesToDynamixelUnits.append(1.0 / self.ConversionFactorFromDynamixelUnitsToDegrees[index])

                else:
                    print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: Error, all elements of OperatingMode_StartingValueList must be contained within the set " + str(OperatingMode_AcceptableStringValuesList))
                    return

        else:
            print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: OperatingMode_StartingValueList must be supplied in setup_dict.")
            return

        print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: OperatingMode_StartingValueList: " + str(self.OperatingMode_StartingValueList))
        print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: Position_DynamixelUnits_Min_FWlimit: " + str(self.Position_DynamixelUnits_Min_FWlimit))
        print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: Position_DynamixelUnits_Max_FWlimit: " + str(self.Position_DynamixelUnits_Max_FWlimit))
        print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: Velocity_DynamixelUnits_Min_FWlimit: " + str(self.Velocity_DynamixelUnits_Min_FWlimit))
        print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: Velocity_DynamixelUnits_Max_FWlimit: " + str(self.Velocity_DynamixelUnits_Max_FWlimit))
        print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: ConversionFactorFromDynamixelUnitsToDegrees: " + str(self.ConversionFactorFromDynamixelUnitsToDegrees))
        print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: ConversionFactorFromDegreesToDynamixelUnits: " + str(self.ConversionFactorFromDegreesToDynamixelUnits))

        self.OperatingMode = list(self.OperatingMode_StartingValueList)
        self.OperatingMode_TO_BE_SET = list(self.OperatingMode_StartingValueList)
        self.OperatingMode_NEEDS_TO_BE_CHANGED_FLAG = [0] * self.NumberOfMotors
        self.OperatingMode_GUI_NEEDS_TO_BE_CHANGED_FLAG = [0] * self.NumberOfMotors
        self.OperatingMode_NEEDS_TO_BE_ASKED_FLAG = [1] * self.NumberOfMotors
        #########################################################
        #########################################################

        ##################################################################################################################
        ##################################################################################################################
        ##################################################################################################################
        ##################################################################################################################
        self.Position_DynamixelUnits_StartingValueList = []

        #########################################################
        #########################################################
        #########################################################
        if "Position_Deg_StartingValueList" in setup_dict and "Position_DynamixelUnits_StartingValueList" in setup_dict:
            print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: Error, cannot pass in BOTH Position_Deg_StartingValueList and Position_DynamixelUnits_StartingValueList.")
            return
        #########################################################
        #########################################################
        #########################################################

        #########################################################
        #########################################################
        #########################################################
        elif "Position_Deg_StartingValueList" in setup_dict and "Position_DynamixelUnits_StartingValueList" not in setup_dict:

            #########################################################
            #########################################################
            temp_list = setup_dict["Position_Deg_StartingValueList"]

            if len(temp_list) != self.NumberOfMotors:
                print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: "+\
                      "Error, all input lists in setup_dict must be the same length as "+\
                      "'MotorType_StringList' (length = " +\
                      str(self.NumberOfMotors) + ").")
                return

            for index, element_deg in enumerate(temp_list):
                #########################################################

                element_dynamixelunits = DynamixelProtocol2Xseries_ReubenPython3Class.ConvertBetweenAllAngularUnits(element_deg,"deg")["dynamixelunits"]
                element_dynamixelunits = self.PassThroughFloatValuesInRange_ExitProgramOtherwise("Position_Deg_StartingValueList_element", element_dynamixelunits, self.Position_DynamixelUnits_Min_FWlimit[index], self.Position_DynamixelUnits_Max_FWlimit[index])

                if element_dynamixelunits != -11111.0:
                    self.Position_DynamixelUnits_StartingValueList.append(element_dynamixelunits)
                else:
                    return
                #########################################################

            print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: *Calculated from Position_Deg_StartingValueList*, Position_DynamixelUnits_StartingValueList valid: " + str(self.Position_DynamixelUnits_StartingValueList))
            #########################################################
            #########################################################

        #########################################################
        #########################################################
        #########################################################

        #########################################################
        #########################################################
        #########################################################
        elif "Position_DynamixelUnits_StartingValueList" in setup_dict and "Position_Deg_StartingValueList"  not in setup_dict:

            #########################################################
            #########################################################
            temp_list = setup_dict["Position_DynamixelUnits_StartingValueList"]

            if len(temp_list) != self.NumberOfMotors:
                print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: "+\
                      "Error, all input lists in setup_dict must be the same length as "+\
                      "'MotorType_StringList' (length = " +\
                      str(self.NumberOfMotors) + ").")
                return

            for index, element in enumerate(temp_list):
                #########################################################
                element = self.PassThroughFloatValuesInRange_ExitProgramOtherwise("Position_DynamixelUnits_StartingValueList_element", element, self.Position_DynamixelUnits_Min_FWlimit[index], self.Position_DynamixelUnits_Max_FWlimit[index])

                if element != -11111.0:
                    self.Position_DynamixelUnits_StartingValueList.append(element)
                else:
                    return
                #########################################################

            print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: *Calculated from Position_DynamixelUnits_StartingValueList*, Position_DynamixelUnits_StartingValueList valid: " + str(self.Position_DynamixelUnits_StartingValueList))
            #########################################################
            #########################################################

        #########################################################
        #########################################################
        #########################################################

        #########################################################
        #########################################################
        #########################################################
        else:
            self.Position_DynamixelUnits_StartingValueList = [0.0]*self.NumberOfMotors
            print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: *Calculated from default*, Position_DynamixelUnits_StartingValueList valid: " + str(self.Position_DynamixelUnits_StartingValueList))
        #########################################################
        #########################################################
        #########################################################

        ##################################################################################################################
        ##################################################################################################################
        ##################################################################################################################
        ##################################################################################################################

        ##########################################
        self.Position_Deg_Max_UserSet = []
        self.Position_DynamixelUnits_Max_UserSet = []
        if "Position_Deg_Max_UserSet" in setup_dict:
            temp_list = setup_dict["Position_Deg_Max_UserSet"]

            FailedFlag = 0
            for item in temp_list:
                if isinstance(item, float) == 1 or isinstance(item, int) == 1:
                    item = int(item)
                    item_dynamixelunits = DynamixelProtocol2Xseries_ReubenPython3Class.ConvertBetweenAllAngularUnits(item, "deg")["dynamixelunits"]
                    if 1:#item_dynamixelunits >= 0 and item <= 4095.0:
                        self.Position_Deg_Max_UserSet.append(item)
                        self.Position_DynamixelUnits_Max_UserSet.append(item_dynamixelunits)
                    #else:
                    #    print("ERROR: Position_DynamixelUnits_Max_UserSet values must be integers between 0 and 4095.")
                    #    FailedFlag = 1
                else:
                    print("ERROR: Position_Deg_Max_UserSet values must be numbers.")
                    FailedFlag = 1

            if FailedFlag == 1:
                print("Position_Deg_Max_UserSet was not valid.")
                return
        else:
            self.Position_DynamixelUnits_Max_UserSet = list(self.Position_DynamixelUnits_Max_FWlimit)

            for element in self.Position_DynamixelUnits_Max_UserSet:
                self.Position_Deg_Max_UserSet.append(DynamixelProtocol2Xseries_ReubenPython3Class.ConvertBetweenAllAngularUnits(element, "dynamixelunits")["deg"])

        print("Position_DynamixelUnits_Max_UserSet valid: " + str(self.Position_DynamixelUnits_Max_UserSet))
        ##########################################

        ##########################################
        self.Position_Deg_Min_UserSet = []
        self.Position_DynamixelUnits_Min_UserSet = []
        if "Position_Deg_Min_UserSet" in setup_dict:
            temp_list = setup_dict["Position_Deg_Min_UserSet"]

            FailedFlag = 0
            for item in temp_list:
                if isinstance(item, float) == 1 or isinstance(item, int) == 1:
                    item = int(item)
                    item_dynamixelunits = DynamixelProtocol2Xseries_ReubenPython3Class.ConvertBetweenAllAngularUnits(item, "deg")["dynamixelunits"]

                    if 1:#item_dynamixelunits >= 0 and item <= 4095.0:
                        self.Position_Deg_Min_UserSet.append(item)
                        self.Position_DynamixelUnits_Min_UserSet.append(item_dynamixelunits)
                    #else:
                    #    print("ERROR: Position_DynamixelUnits_Min_UserSet values must be integers between 0 and 4095.")
                    #    FailedFlag = 1
                else:
                    print("ERROR: Position_Deg_Min_UserSet values must be numbers.")
                    FailedFlag = 1

            if FailedFlag == 1:
                print("Position_Deg_Min_UserSet was not valid.")
                return
        else:
            self.Position_DynamixelUnits_Min_UserSet = list(self.Position_DynamixelUnits_Max_FWlimit)

            for element in self.Position_DynamixelUnits_Min_UserSet:
                self.Position_Deg_Min_UserSet.append(DynamixelProtocol2Xseries_ReubenPython3Class.ConvertBetweenAllAngularUnits(element, "dynamixelunits")["deg"])

        print("Position_DynamixelUnits_Min_UserSet valid: " + str(self.Position_DynamixelUnits_Min_UserSet))
        ##########################################
        ##########################################

        ##########################################
        ##########################################

        ##########################################
        ##########################################

        #########################################################
        #########################################################
        #########################################################
        if "Velocity_DynamixelUnits_StartingValueList" in setup_dict:

            #########################################################
            #########################################################
            self.Velocity_DynamixelUnits_StartingValueList = []
            temp_list = setup_dict["Velocity_DynamixelUnits_StartingValueList"]

            if len(temp_list) != self.NumberOfMotors:
                print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: "+\
                      "Error, all input lists in setup_dict must be the same length as "+\
                      "'MotorType_StringList' (length = " +\
                      str(self.NumberOfMotors) + ").")
                return

            for index, element in enumerate(temp_list):
                #########################################################
                element = self.PassThroughFloatValuesInRange_ExitProgramOtherwise("Velocity_DynamixelUnits_StartingValueList_element", element, self.Velocity_DynamixelUnits_Min_FWlimit[index], self.Velocity_DynamixelUnits_Max_FWlimit[index])

                if element != -11111.0:
                    self.Velocity_DynamixelUnits_StartingValueList.append(element)
                else:
                    return
                #########################################################

            #########################################################
            #########################################################

        else:
            self.Velocity_DynamixelUnits_StartingValueList = [(Velocity_DynamixelUnits_Max_FWlimit + Velocity_DynamixelUnits_Min_FWlimit)/2.0] * self.NumberOfMotors

        print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: Velocity_DynamixelUnits_StartingValueList valid: " + str(self.Velocity_DynamixelUnits_StartingValueList))
        #########################################################
        #########################################################
        #########################################################

        #########################################################
        #########################################################
        #########################################################
        if "Velocity_DynamixelUnits_Max_UserSet" in setup_dict:

            #########################################################
            #########################################################
            self.Velocity_DynamixelUnits_Max_UserSet = []
            temp_list = setup_dict["Velocity_DynamixelUnits_Max_UserSet"]

            if len(temp_list) != self.NumberOfMotors:
                print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: "+\
                      "Error, all input lists in setup_dict must be the same length as "+\
                      "'MotorType_StringList' (length = " +\
                      str(self.NumberOfMotors) + ").")
                return

            for index, element in enumerate(temp_list):
                #########################################################
                element = self.PassThroughFloatValuesInRange_ExitProgramOtherwise("Velocity_DynamixelUnits_Max_UserSet_element", element, self.Velocity_DynamixelUnits_Min_FWlimit[index], self.Velocity_DynamixelUnits_Max_FWlimit[index])

                if element != -11111.0:
                    self.Velocity_DynamixelUnits_Max_UserSet.append(element)
                else:
                    return
                #########################################################

            #########################################################
            #########################################################

        else:
            self.Velocity_DynamixelUnits_Max_UserSet = [Velocity_DynamixelUnits_Max_FWlimit] * self.NumberOfMotors

        print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: Velocity_DynamixelUnits_Max_UserSet valid: " + str(self.Velocity_DynamixelUnits_Max_UserSet))
        #########################################################
        #########################################################
        #########################################################

        #########################################################
        #########################################################
        #########################################################
        if "Velocity_DynamixelUnits_Min_UserSet" in setup_dict:

            #########################################################
            #########################################################
            self.Velocity_DynamixelUnits_Min_UserSet = []
            temp_list = setup_dict["Velocity_DynamixelUnits_Min_UserSet"]

            if len(temp_list) != self.NumberOfMotors:
                print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: "+\
                      "Error, all input lists in setup_dict must be the same length as "+\
                      "'MotorType_StringList' (length = " +\
                      str(self.NumberOfMotors) + ").")
                return

            for index, element in enumerate(temp_list):
                #########################################################
                element = self.PassThroughFloatValuesInRange_ExitProgramOtherwise("Velocity_DynamixelUnits_Min_UserSet_element", element, -1.0*self.Velocity_DynamixelUnits_Max_FWlimit[index], self.Velocity_DynamixelUnits_Max_FWlimit[index])

                if element != -11111.0:
                    self.Velocity_DynamixelUnits_Min_UserSet.append(element)
                else:
                    return
                #########################################################

            #########################################################
            #########################################################

        else:
            self.Velocity_DynamixelUnits_Min_UserSet = [-1.0*Velocity_DynamixelUnits_Max_FWlimit] * self.NumberOfMotors

        print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: Velocity_DynamixelUnits_Min_UserSet valid: " + str(self.Velocity_DynamixelUnits_Min_UserSet))
        #########################################################
        #########################################################
        #########################################################

        ##########################################
        ##########################################

        ##########################################
        ##########################################

        ##########################################
        self.Current_DynamixelUnits_StartingValueList = []
        if "Current_DynamixelUnits_StartingValueList" in setup_dict:
            temp_list = setup_dict["Current_DynamixelUnits_StartingValueList"]

            FailedFlag = 0
            for item in temp_list:
                if isinstance(item, float) == 1 or isinstance(item, int) == 1:
                    item = int(item)
                    if item >= -2047.0 and item <= 2047.0:
                        self.Current_DynamixelUnits_StartingValueList.append(item)
                    else:
                        print("ERROR: Current_DynamixelUnits_StartingValueList values must be integers between 0 and 4095.")
                        FailedFlag = 1
                else:
                    print("ERROR: Current_DynamixelUnits_StartingValueList values must be numbers.")
                    FailedFlag = 1

            if FailedFlag == 1:
                print("Current_DynamixelUnits_StartingValueList was not valid.")
                return

        else:
            self.Current_DynamixelUnits_StartingValueList = [2047.0] * self.NumberOfMotors

        print("Current_DynamixelUnits_StartingValueList valid: " + str(self.Current_DynamixelUnits_StartingValueList))
        ##########################################

        ##########################################
        self.Current_DynamixelUnits_max = []
        self.Current_Percent0to1_max = []
        if "Current_Percent0to1_max" in setup_dict:
            temp_list = setup_dict["Current_Percent0to1_max"]

            FailedFlag = 0
            for item in temp_list:
                if isinstance(item, float) == 1 or isinstance(item, int) == 1:
                    item = item
                    item_dynamixel_units = int(self.ConvertBetweenAllCurrentUnits(item, "percent")["dynamixelunits"])
                    if item >= 0 and item <= 1:
                        self.Current_Percent0to1_max.append(item)
                        self.Current_DynamixelUnits_max.append(item_dynamixel_units)
                    else:
                        print("ERROR: Current_Percent0to1_max values must be in the range [0, 1].")
                        FailedFlag = 1
                else:
                    print("ERROR: Current_Percent0to1_max values must be numbers.")
                    FailedFlag = 1

            if FailedFlag == 1:
                print("Current_Percent0to1_max was not valid.")
                return
        else:
            self.Current_Percent0to1_max = [1.0] * self.NumberOfMotors
            self.Current_DynamixelUnits_max = [2047.0] * self.NumberOfMotors

        print("Current_Percent0to1_max valid: " + str(self.Current_Percent0to1_max))
        print("Current_DynamixelUnits_max valid: " + str(self.Current_DynamixelUnits_max))
        ##########################################

        ##########################################
        self.Current_DynamixelUnits_min = []
        if "Current_DynamixelUnits_min" in setup_dict:
            temp_list = setup_dict["Current_DynamixelUnits_min"]

            FailedFlag = 0
            for item in temp_list:
                if isinstance(item, float) == 1 or isinstance(item, int) == 1:
                    item = int(item)
                    if item >= -2047.0 and item <= 2047.0:
                        self.Current_DynamixelUnits_min.append(item)
                    else:
                        print("ERROR: Current_DynamixelUnits_min values must be integers between 0 and 4095.")
                        FailedFlag = 1
                else:
                    print("ERROR: Current_DynamixelUnits_min values must be numbers.")
                    FailedFlag = 1

            if FailedFlag == 1:
                print("Current_DynamixelUnits_min was not valid.")
                return
        else:
            self.Current_DynamixelUnits_min = [-2047.0] * self.NumberOfMotors

        print("Current_DynamixelUnits_min valid: " + str(self.Current_DynamixelUnits_min))
        ##########################################

        ##########################################
        ##########################################

        ##########################################
        ##########################################

        ##########################################
        self.PWM_DynamixelUnits_StartingValueList = []
        if "PWM_DynamixelUnits_StartingValueList" in setup_dict:
            temp_list = setup_dict["PWM_DynamixelUnits_StartingValueList"]

            FailedFlag = 0
            for item in temp_list:
                if isinstance(item, float) == 1 or isinstance(item, int) == 1:
                    item = int(item)
                    if item >= 0 and item <= 885.0:
                        self.PWM_DynamixelUnits_StartingValueList.append(item)
                    else:
                        print("ERROR: PWM_DynamixelUnits_StartingValueList values must be integers between 0 and 4095.")
                        FailedFlag = 1
                else:
                    print("ERROR: PWM_DynamixelUnits_StartingValueList values must be numbers.")
                    FailedFlag = 1

            if FailedFlag == 1:
                print("PWM_DynamixelUnits_StartingValueList was not valid.")
                return

        else:
            self.PWM_DynamixelUnits_StartingValueList = [885.0] * self.NumberOfMotors

        print("PWM_DynamixelUnits_StartingValueList valid: " + str(self.PWM_DynamixelUnits_StartingValueList))
        ##########################################

        ##########################################
        self.PWM_DynamixelUnits_max = []
        if "PWM_DynamixelUnits_max" in setup_dict:
            temp_list = setup_dict["PWM_DynamixelUnits_max"]

            FailedFlag = 0
            for item in temp_list:
                if isinstance(item, float) == 1 or isinstance(item, int) == 1:
                    item = int(item)
                    if item >= 0 and item <= 885.0:
                        self.PWM_DynamixelUnits_max.append(item)
                    else:
                        print("ERROR: PWM_DynamixelUnits_max values must be integers between 0 and 4095.")
                        FailedFlag = 1
                else:
                    print("ERROR: PWM_DynamixelUnits_max values must be numbers.")
                    FailedFlag = 1

            if FailedFlag == 1:
                print("PWM_DynamixelUnits_max was not valid.")
                return
        else:
            self.PWM_DynamixelUnits_max = [885.0] * self.NumberOfMotors

        print("PWM_DynamixelUnits_max valid: " + str(self.PWM_DynamixelUnits_max))
        ##########################################

        ##########################################
        self.PWM_DynamixelUnits_min = []
        if "PWM_DynamixelUnits_min" in setup_dict:
            temp_list = setup_dict["PWM_DynamixelUnits_min"]

            FailedFlag = 0
            for item in temp_list:
                if isinstance(item, float) == 1 or isinstance(item, int) == 1:
                    item = int(item)
                    if item >= 0 and item <= 885.0:
                        self.PWM_DynamixelUnits_min.append(item)
                    else:
                        print("ERROR: PWM_DynamixelUnits_min values must be integers between 0 and 4095.")
                        FailedFlag = 1
                else:
                    print("ERROR: PWM_DynamixelUnits_min values must be numbers.")
                    FailedFlag = 1

            if FailedFlag == 1:
                print("PWM_DynamixelUnits_min was not valid.")
                return
        else:
            self.PWM_DynamixelUnits_min = [0] * self.NumberOfMotors

        print("PWM_DynamixelUnits_min valid: " + str(self.PWM_DynamixelUnits_min))
        ##########################################

        ##########################################
        self.StartEngagedFlag = []
        if "StartEngagedFlag" in setup_dict:
            temp_list = setup_dict["StartEngagedFlag"]

            FailedFlag = 0
            for item in temp_list:
                #print(item)
                if isinstance(item, float) == 1 or isinstance(item, int) == 1:
                    item = int(item)
                    if item == 0 or item == 1:
                        self.StartEngagedFlag.append(item)
                    else:
                        print("ERROR: StartEngagedFlag values must be integers between 0 and 1.")
                        FailedFlag = 1
                else:
                    print("ERROR: StartEngagedFlag values must be numbers.")
                    FailedFlag = 1

            if FailedFlag == 1:
                print("StartEngagedFlag was not valid.")
                return

        else:
            self.StartEngagedFlag = [1] * self.NumberOfMotors

        print("StartEngagedFlag valid: " + str(self.StartEngagedFlag))
        ##########################################

        ##########################################
        self.SendInstructionPacket_SetPosition_Counter = [0] * self.NumberOfMotors
        ##########################################

        #########################################################
        #########################################################
        self.ProcessSetupDictInputsTheCanBeLiveChanged(setup_dict)
        #########################################################
        #########################################################

        #########################################################
        #########################################################
        self.PrintToGui_Label_TextInputHistory_List = [" "]*self.NumberOfPrintLines
        self.PrintToGui_Label_TextInput_Str = ""
        self.GUI_ready_to_be_updated_flag = 0
        #########################################################
        #########################################################

        #########################################################
        #########################################################
        try:
            self.DataStreamingFrequency_Tx_CalculatedFromMainThread_LowPassFilter_ReubenPython2and3ClassObject = LowPassFilter_ReubenPython2and3Class(dict([("UseMedianFilterFlag", 1),
                                                                                                            ("UseExponentialSmoothingFilterFlag", 1),
                                                                                                            ("ExponentialSmoothingFilterLambda", 0.05)])) #new_filtered_value = k * raw_sensor_value + (1 - k) * old_filtered_value

        except:
            exceptions = sys.exc_info()[0]
            print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: DataStreamingFrequency_Tx_CalculatedFromMainThread_LowPassFilter_ReubenPython2and3ClassObject, Exceptions: %s" % exceptions)
            traceback.print_exc()
            return
        #########################################################
        #########################################################

        #########################################################
        #########################################################
        try:
            self.DataStreamingFrequency_Rx_CalculatedFromMainThread_LowPassFilter_ReubenPython2and3ClassObject = LowPassFilter_ReubenPython2and3Class(dict([("UseMedianFilterFlag", 1),
                                                                                                            ("UseExponentialSmoothingFilterFlag", 1),
                                                                                                            ("ExponentialSmoothingFilterLambda", 0.05)])) #new_filtered_value = k * raw_sensor_value + (1 - k) * old_filtered_value

        except:
            exceptions = sys.exc_info()[0]
            print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: DataStreamingFrequency_CalculatedFromDedicatedRxThread_LowPassFilter_ReubenPython2and3ClassObject, Exceptions: %s" % exceptions)
            traceback.print_exc()
            return
        #########################################################
        #########################################################

        self.TimeToWaitBetweenCriticalInstructions = 0.002

        self.PWM_DynamixelUnits_NeedsToBeChangedFlag = [0] * self.NumberOfMotors
        self.PWM_DynamixelUnits_GUI_NeedsToBeChangedFlag = [0] * self.NumberOfMotors

        self.Current_DynamixelUnits_NeedsToBeChangedFlag = [1] * self.NumberOfMotors
        self.Current_DynamixelUnits_GUI_NeedsToBeChangedFlag = [0] * self.NumberOfMotors

        self.Velocity_DynamixelUnits_NeedsToBeChangedFlag = [1] * self.NumberOfMotors
        self.Velocity_DynamixelUnits_GUI_NeedsToBeChangedFlag = [0] * self.NumberOfMotors

        self.Position_DynamixelUnits_NeedsToBeChangedFlag = [0] * self.NumberOfMotors
        self.Position_DynamixelUnits_GUI_NeedsToBeChangedFlag = [0] * self.NumberOfMotors
        self.MotionDirectionCommandedByExternalProgram = [-1] * self.NumberOfMotors

        self.MaxPWM_DynamixelUnits = [-1] * self.NumberOfMotors
        self.MaxPWM_DynamixelUnits_TO_BE_SET = [-1] * self.NumberOfMotors
        self.MaxPWM_DynamixelUnits_NeedsToBeChangedFlag = [0] * self.NumberOfMotors
        self.MaxPWM_DynamixelUnits_GUI_NeedsToBeChangedFlag = [0] * self.NumberOfMotors

        self.ToggleMinMax_state = [-1] * self.NumberOfMotors
        self.ToggleMinMax_TO_BE_SET = [-1] * self.NumberOfMotors
        self.ToggleMinMax_NeedsToTakePlaceFlag = [-1] * self.NumberOfMotors
        self.ResetSerial_NeedsToTakePlaceFlag = [-1] * self.NumberOfMotors
        self.Reboot_NeedsToTakePlaceFlag = [-1] * self.NumberOfMotors
        self.OperatingModeReceived_int = [-1] * self.NumberOfMotors
        self.OperatingModeReceived_string = ["default"] * self.NumberOfMotors
        self.RealTimeTicksMillisec = [-1] * self.NumberOfMotors
        self.RealTimeTicksMillisec_last = [-1] * self.NumberOfMotors

        self.ErrorFlag_BYTE = [-1] * self.NumberOfMotors
        self.ErrorFlag_Overload_Received = [-1] * self.NumberOfMotors
        self.ErrorFlag_ElectricalShock_Received = [-1] * self.NumberOfMotors
        self.ErrorFlag_MotorEncoder_Received = [-1] * self.NumberOfMotors
        self.ErrorFlag_Overheating_Received = [-1] * self.NumberOfMotors
        self.ErrorFlag_InputVoltage_Received = [-1] * self.NumberOfMotors
        self.ErrorFlag_SerialCommunication = [-1] * self.NumberOfMotors
        self.ErrorFlag_OperatingModeMismatch = [0] * self.NumberOfMotors #Assume there's no error unless we explicitly check.

        self.WatchdogTimerDurationMilliseconds = [-1] * self.NumberOfMotors

        self.AskForInfrequentDataReadLoopCounter = 0
        print("self.AskForInfrequentDataReadLoopCounter: " + str(self.AskForInfrequentDataReadLoopCounter))

        self.EngagedState = [0]*self.NumberOfMotors #TO BE SET LATER. DEFINED HERE TO PREVENT GUI THREAD CREATION ERRORS.
        self.EngagedState_TO_BE_SET = [0]*self.NumberOfMotors
        self.EngagedState_NeedsToBeChangedFlag = [0]*self.NumberOfMotors
        self.EngagedState_GUI_NeedsToBeChangedFlag = [0]*self.NumberOfMotors
        self.StoppedState = [-1]*self.NumberOfMotors

        self.LEDstate = [1]*self.NumberOfMotors #TO BE SET LATER. DEFINED HERE TO PREVENT GUI THREAD CREATION ERRORS.
        self.LEDstate_TO_BE_SET = [1]*self.NumberOfMotors
        self.LEDstate_NeedsToBeChangedFlag = [0]*self.NumberOfMotors
        self.LEDstate_GUI_NeedsToBeChangedFlag = [0]*self.NumberOfMotors

        self.HasMotorEverBeenInitializedFlag = [0]*self.NumberOfMotors

        self.StartingTime_CalculatedFromMainThread = -11111.0

        self.CurrentTime_Rx_CalculatedFromMainThread = -11111.0
        self.LastTime_Rx_CalculatedFromMainThread = -11111.0
        self.DataStreamingFrequency_Rx_CalculatedFromMainThread = -11111.0
        self.DataStreamingDeltaT_Rx_CalculatedFromMainThread = -11111.0
        self.LoopCounter_Rx_CalculatedFromMainThread = 0

        self.CurrentTime_Tx_CalculatedFromMainThread = -11111.0
        self.LastTime_Tx_CalculatedFromMainThread = -11111.0
        self.DataStreamingFrequency_Tx_CalculatedFromMainThread = -11111.0
        self.DataStreamingDeltaT_Tx_CalculatedFromMainThread = -11111.0
        self.LoopCounter_Tx_CalculatedFromMainThread = 0

        self.DataStreamingFrequency_RealTimeTicksMillisecFromDynamixel = [-1]*self.NumberOfMotors
        self.DataStreamingDeltaT_RealTimeTicksMillisecFromDynamixel = [-1]*self.NumberOfMotors

        self.Position_DynamixelUnits = list(self.Position_DynamixelUnits_StartingValueList)
        self.Position_DynamixelUnits_TO_BE_SET = list(self.Position_DynamixelUnits_StartingValueList)

        self.Velocity_DynamixelUnits = list(self.Velocity_DynamixelUnits_StartingValueList)
        self.Velocity_DynamixelUnits_TO_BE_SET = list(self.Velocity_DynamixelUnits_StartingValueList)

        self.Current_DynamixelUnits = list(self.Current_DynamixelUnits_StartingValueList)
        self.Current_DynamixelUnits_TO_BE_SET = list(self.Current_DynamixelUnits_StartingValueList)

        self.PWM_DynamixelUnits = list(self.PWM_DynamixelUnits_StartingValueList)
        self.PWM_DynamixelUnits_TO_BE_SET = list(self.PWM_DynamixelUnits_StartingValueList)
        #########################################################
        #########################################################

        #########################################################
        #########################################################
        try:

            #########################################################
            if ftd2xx_IMPORTED_FLAG == 1:
                self.SetAllFTDIdevicesLatencyTimer()
            #########################################################

            #########################################################
            self.FindAssignAndOpenSerialPort()
            self.CloseSerialPort() #Since Dynamixel will be handling this.
            #########################################################

            #########################################################
            self.portHandler = PortHandler(self.SerialPortNameCorrespondingToCorrectSerialNumber)
            self.packetHandler = PacketHandler(2.0)

            if self.portHandler.openPort():
                print("Succeeded to open the port")
                self.SerialConnectedFlag = 1
            else:
                print("Failed to open the port")
                self.SerialConnectedFlag = 0

            if self.portHandler.setBaudRate(self.SerialBaudRate):
                print("Succeeded to change the baudrate")
                self.SerialConnectedFlag = 1
            else:
                print("Failed to change the baudrate")
                self.SerialConnectedFlag = 0
            #########################################################

            #########################################################
            if self.SerialConnectedFlag != 1:
                return
            #########################################################

        except:
            exceptions = sys.exc_info()[0]
            print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: Exceptions: %s" % exceptions)
            traceback.print_exc()
            return
        #########################################################
        #########################################################

        #########################################################
        #########################################################
        self.MainThread_ThreadingObject = threading.Thread(target=self.MainThread, args=())
        self.MainThread_ThreadingObject.start()
        #########################################################
        #########################################################

        #########################################################
        #########################################################
        if self.USE_GUI_FLAG == 1:
            self.StartGUI(self.root)
        #########################################################
        #########################################################

        #########################################################
        #########################################################
        time.sleep(0.25)
        #########################################################
        #########################################################

        #########################################################
        #########################################################
        self.OBJECT_CREATED_SUCCESSFULLY_FLAG = 1
        #########################################################
        #########################################################

    ##########################################################################################################
    ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    def ProcessSetupDictInputsTheCanBeLiveChanged(self, setup_dict):
        pass
    ##########################################################################################################
    ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    def __del__(self):
        pass
    ##########################################################################################################
    ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    ##########################################################################################################
    def PassThrough0and1values_ExitProgramOtherwise(self, InputNameString, InputNumber, ExitProgramIfFailureFlag = 0):

        ##########################################################################################################
        ##########################################################################################################
        try:

            ##########################################################################################################
            InputNumber_ConvertedToFloat = float(InputNumber)
            ##########################################################################################################

        except:

            ##########################################################################################################
            exceptions = sys.exc_info()[0]
            print("PassThrough0and1values_ExitProgramOtherwise Error. InputNumber must be a numerical value, Exceptions: %s" % exceptions)

            ##########################
            if ExitProgramIfFailureFlag == 1:
                sys.exit()
            else:
                return -1
            ##########################

            ##########################################################################################################

        ##########################################################################################################
        ##########################################################################################################

        ##########################################################################################################
        ##########################################################################################################
        try:

            ##########################################################################################################
            if InputNumber_ConvertedToFloat == 0.0 or InputNumber_ConvertedToFloat == 1.0:
                return InputNumber_ConvertedToFloat

            else:

                print("PassThrough0and1values_ExitProgramOtherwise Error. '" +
                              str(InputNameString) +
                              "' must be 0 or 1 (value was " +
                              str(InputNumber_ConvertedToFloat) +
                              "). Press any key (and enter) to exit.")

                ##########################
                if ExitProgramIfFailureFlag == 1:
                    sys.exit()

                else:
                    return -1
                ##########################

            ##########################################################################################################

        except:

            ##########################################################################################################
            exceptions = sys.exc_info()[0]
            print("PassThrough0and1values_ExitProgramOtherwise Error, Exceptions: %s" % exceptions)

            ##########################
            if ExitProgramIfFailureFlag == 1:
                sys.exit()
            else:
                return -1
            ##########################

            ##########################################################################################################

        ##########################################################################################################
        ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    ##########################################################################################################
    def PassThroughFloatValuesInRange_ExitProgramOtherwise(self, InputNameString, InputNumber, RangeMinValue, RangeMaxValue, ExitProgramIfFailureFlag = 0):

        ##########################################################################################################
        ##########################################################################################################
        try:
            ##########################################################################################################
            InputNumber_ConvertedToFloat = float(InputNumber)
            ##########################################################################################################

        except:
            ##########################################################################################################
            exceptions = sys.exc_info()[0]
            print("PassThroughFloatValuesInRange_ExitProgramOtherwise Error. InputNumber for '" + str(InputNameString) + "' must be a float value, Exceptions: %s" % exceptions)

            ##########################
            if ExitProgramIfFailureFlag == 1:
                sys.exit()
            else:
                return -11111.0
            ##########################

            ##########################################################################################################

        ##########################################################################################################
        ##########################################################################################################

        ##########################################################################################################
        ##########################################################################################################
        try:

            ##########################################################################################################
            InputNumber_ConvertedToFloat_Limited = self.LimitNumber_FloatOutputOnly(RangeMinValue, RangeMaxValue, InputNumber_ConvertedToFloat)

            if InputNumber_ConvertedToFloat_Limited != InputNumber_ConvertedToFloat:
                print("PassThroughFloatValuesInRange_ExitProgramOtherwise Error. '" +
                      str(InputNameString) +
                      "' must be in the range [" +
                      str(RangeMinValue) +
                      ", " +
                      str(RangeMaxValue) +
                      "] (value was " +
                      str(InputNumber_ConvertedToFloat) + ")")

                ##########################
                if ExitProgramIfFailureFlag == 1:
                    sys.exit()
                else:
                    return -11111.0
                ##########################

            else:
                return InputNumber_ConvertedToFloat_Limited
            ##########################################################################################################

        except:
            ##########################################################################################################
            exceptions = sys.exc_info()[0]
            print("PassThroughFloatValuesInRange_ExitProgramOtherwise Error, Exceptions: %s" % exceptions)

            ##########################
            if ExitProgramIfFailureFlag == 1:
                sys.exit()
            else:
                return -11111.0
            ##########################

            ##########################################################################################################

        ##########################################################################################################
        ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    def ComputeTwosComplement(self, IntegerToBeConverted, NumberOfBitsInInteger = 16):
        return IntegerToBeConverted & ((2 ** NumberOfBitsInInteger) - 1)

    ##########################################################################################################
    ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    def SetAllFTDIdevicesLatencyTimer(self, FTDI_LatencyTimer_ToBeSet = 1):

        FTDI_LatencyTimer_ToBeSet = self.LimitNumber_IntOutputOnly(1, 16, FTDI_LatencyTimer_ToBeSet)

        FTDI_DeviceList = ftd2xx.listDevices()
        print("FTDI_DeviceList: " + str(FTDI_DeviceList))

        if FTDI_DeviceList != None:

            for Index, FTDI_SerialNumber in enumerate(FTDI_DeviceList):

                #################################
                try:
                    if sys.version_info[0] < 3: #Python 2
                        FTDI_SerialNumber = str(FTDI_SerialNumber)
                    else:
                        FTDI_SerialNumber = FTDI_SerialNumber.decode('utf-8')

                    FTDI_Object = ftd2xx.open(Index)
                    FTDI_DeviceInfo = FTDI_Object.getDeviceInfo()

                    '''
                    print("FTDI device with serial number " +
                          str(FTDI_SerialNumber) +
                          ", DeviceInfo: " +
                          str(FTDI_DeviceInfo))
                    '''

                except:
                    exceptions = sys.exc_info()[0]
                    print("FTDI device with serial number " + str(FTDI_SerialNumber) + ", could not open FTDI device, Exceptions: %s" % exceptions)
                #################################

                #################################
                try:
                    FTDI_Object.setLatencyTimer(FTDI_LatencyTimer_ToBeSet)
                    time.sleep(0.005)

                    FTDI_LatencyTimer_ReceivedFromDevice = FTDI_Object.getLatencyTimer()
                    FTDI_Object.close()

                    if FTDI_LatencyTimer_ReceivedFromDevice == FTDI_LatencyTimer_ToBeSet:
                        SuccessString = "succeeded!"
                    else:
                        SuccessString = "failed!"

                    print("FTDI device with serial number " +
                          str(FTDI_SerialNumber) +
                          " commanded setLatencyTimer(" +
                          str(FTDI_LatencyTimer_ToBeSet) +
                          "), and getLatencyTimer() returned: " +
                          str(FTDI_LatencyTimer_ReceivedFromDevice) +
                          ", so command " +
                          SuccessString)

                except:
                    exceptions = sys.exc_info()[0]
                    print("FTDI device with serial number " + str(FTDI_SerialNumber) + ", could not set/get Latency Timer, Exceptions: %s" % exceptions)
                #################################

        else:
            print("SetAllFTDIdevicesLatencyTimer ERROR: FTDI_DeviceList is empty, cannot proceed.")
    ##########################################################################################################
    ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    def FindAssignAndOpenSerialPort(self):
        self.MyPrint_WithoutLogFile("FindAssignAndOpenSerialPort: Finding all serial ports...")

        ##############
        SerialNumberToCheckAgainst = str(self.DesiredSerialNumber_USBtoSerialConverter)
        if self.my_platform == "linux" or self.my_platform == "pi":
            SerialNumberToCheckAgainst = SerialNumberToCheckAgainst[:-1] #The serial number gets truncated by one digit in linux
        else:
            SerialNumberToCheckAgainst = SerialNumberToCheckAgainst
        ##############

        ##############
        SerialPortsAvailable_ListPortInfoObjetsList = serial.tools.list_ports.comports()
        ##############

        ###########################################################################
        SerialNumberFoundFlag = 0
        for SerialPort_ListPortInfoObjet in SerialPortsAvailable_ListPortInfoObjetsList:

            SerialPortName = SerialPort_ListPortInfoObjet[0]
            Description = SerialPort_ListPortInfoObjet[1]
            VID_PID_SerialNumber_Info = SerialPort_ListPortInfoObjet[2]
            self.MyPrint_WithoutLogFile(SerialPortName + ", " + Description + ", " + VID_PID_SerialNumber_Info)

            if VID_PID_SerialNumber_Info.find(SerialNumberToCheckAgainst) != -1 and SerialNumberFoundFlag == 0: #Haven't found a match in a prior loop
                self.SerialPortNameCorrespondingToCorrectSerialNumber = SerialPortName
                SerialNumberFoundFlag = 1 #To ensure that we only get one device
                self.MyPrint_WithoutLogFile("FindAssignAndOpenSerialPort: Found serial number " + SerialNumberToCheckAgainst + " on port " + self.SerialPortNameCorrespondingToCorrectSerialNumber)
                #WE DON'T BREAK AT THIS POINT BECAUSE WE WANT TO PRINT ALL SERIAL DEVICE NUMBERS WHEN PLUGGING IN A DEVICE WITH UNKNOWN SERIAL NUMBE RFOR THE FIRST TIME.
        ###########################################################################

        ###########################################################################
        if(self.SerialPortNameCorrespondingToCorrectSerialNumber != "default"): #We found a match

            try: #Will succeed as long as another program hasn't already opened the serial line.

                self.SerialObject = serial.Serial(self.SerialPortNameCorrespondingToCorrectSerialNumber,
                                                  self.SerialBaudRate,
                                                  timeout=self.SerialTimeout_Rx_Seconds,
                                                  write_timeout=self.SerialTimeout_Tx_Seconds,
                                                  parity=self.SerialParity,
                                                  stopbits=self.SerialStopBits,
                                                  bytesize=self.SerialByteSize,
                                                  xonxoff=self.SerialXonXoffSoftwareFlowControl)

                try:
                    if self.my_platform == "windows":
                        #pass
                        self.SerialObject.set_buffer_size(rx_size=self.SerialRxBufferSize, tx_size=self.SerialTxBufferSize)

                except:
                    self.SerialConnectedFlag = 0
                    exceptions = sys.exc_info()[0]
                    self.MyPrint_WithoutLogFile("FindAssignAndOpenSerialPort, 'set_buffer_size' call failed, exception: %s" % exceptions)

                self.SerialObject.flushInput()
                self.SerialObject.flushOutput()

                self.SerialConnectedFlag = 1
                self.MyPrint_WithoutLogFile("FindAssignAndOpenSerialPort: Serial is connected and open on port: " + self.SerialPortNameCorrespondingToCorrectSerialNumber)

            except:
                self.SerialConnectedFlag = 0
                print("FindAssignAndOpenSerialPort: ERROR: Serial is physically plugged in but IS IN USE BY ANOTHER PROGRAM.")
                exceptions = sys.exc_info()[0]
                print("FindAssignAndOpenSerialPort, exception: %s" % exceptions)
        else:
            self.SerialConnectedFlag = -1
            self.MyPrint_WithoutLogFile("FindAssignAndOpenSerialPort: ERROR: Could not find the serial device. IS IT PHYSICALLY PLUGGED IN?")
        ###########################################################################

    ##########################################################################################################
    ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    def ResetSerialConnection(self):

        try:
            self.SerialObject.reset_input_buffer()
            self.SerialObject.reset_output_buffer()
            self.SerialObject.close()
            self.SerialConnectedFlag = 0

            self.SerialObject = serial.Serial(self.SerialPortNameCorrespondingToCorrectSerialNumber,
                                              self.SerialBaudRate,
                                              timeout=self.SerialTimeout_Rx_Seconds,
                                              write_timeout=self.SerialTimeout_Tx_Seconds,
                                              parity=self.SerialParity,
                                              stopbits=self.SerialStopBits,
                                              bytesize=self.SerialByteSize,
                                              xonxoff=self.SerialXonXoffSoftwareFlowControl)

            if self.my_platform == "windows":
                self.SerialObject.set_buffer_size(rx_size=self.SerialRxBufferSize, tx_size=self.SerialTxBufferSize)

            self.SerialConnectedFlag = 1

            print("########## ResetSerialConnection EVENT FIRED! ##########")

        except:
            self.SerialConnectedFlag = 0
            exceptions = sys.exc_info()[0]
            self.MyPrint_WithoutLogFile("ResetSerialConnection, exception: %s" % exceptions)

    ##########################################################################################################
    ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    def CloseSerialPort(self):

        if self.SerialConnectedFlag == 1:
            self.SerialObject.close()
            self.MyPrint_WithoutLogFile("Closed serial connection.")
    ##########################################################################################################
    ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    def getPreciseSecondsTimeStampString(self):
        ts = time.time()

        return ts
    ##########################################################################################################
    ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    ##########################################################################################################
    def GetMostRecentDataDict(self, OverrideLengthMatchingRequirementFlag = 0):

        if self.EXIT_PROGRAM_FLAG == 0:

            ##########################################################################################################
            ##########################################################################################################
            if OverrideLengthMatchingRequirementFlag == 0:

                if len(self.MostRecentDataDict_ForExternalQueryAndGUIdisplayOnly) == self.MostRecentDataDict_ForExternalQueryAndGUIdisplayOnly_FullyPopulatedLength:

                    ##########################################################################################################
                    while self.MostRecentDataDict_ForExternalQueryAndGUIdisplayOnly_BlockExternalCopyingFlag == 1:
                        time.sleep(0.001)
                        print("DynamixelProtocol2Xseries_ReubenPython3Class, GetMostRecentDataDict WAIT, self.MostRecentDataDict_ForExternalQueryAndGUIdisplayOnly_BlockExternalCopyingFlag == 1")
                    ##########################################################################################################

                    ##########################################################################################################
                    if self.MostRecentDataDict_ForExternalQueryAndGUIdisplayOnly_BlockExternalCopyingFlag == 0:
                        return deepcopy(self.MostRecentDataDict_ForExternalQueryAndGUIdisplayOnly) #deepcopy IS required as MostRecentDataDict contains lists.
                    ##########################################################################################################

                else:
                    return dict()
            ##########################################################################################################
            ##########################################################################################################

            ##########################################################################################################
            ##########################################################################################################
            else:
                return deepcopy(self.MostRecentDataDict_ForExternalQueryAndGUIdisplayOnly) #deepcopy IS required as MostRecentDataDict contains lists.
            ##########################################################################################################
            ##########################################################################################################

        else:
            return dict()  # So that we're not returning variables during the close-down process.
    ##########################################################################################################
    ##########################################################################################################
    ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    def SetOperatingMode_FROM_EXTERNAL_PROGRAM(self, MotorIndex, OperatingModeStringExternalProgram):

        if OperatingModeStringExternalProgram not in OperatingMode_AcceptableStringValuesList:
            self.MyPrint_WithoutLogFile("SetOperatingMode_FROM_EXTERNAL_PROGRAM ERROR: OperatingModeStringExternalProgram must be in " + str(OperatingMode_AcceptableStringValuesList))
            return 0

        self.MyPrint_WithoutLogFile("SetOperatingMode_FROM_EXTERNAL_PROGRAM changing OperatingMode on motor " + str(MotorIndex) + " to a value of " + str(OperatingModeStringExternalProgram))

        self.OperatingMode_TO_BE_SET[MotorIndex] = OperatingModeStringExternalProgram
        self.OperatingMode_NEEDS_TO_BE_CHANGED_FLAG[MotorIndex] = 1
        self.OperatingMode_GUI_NEEDS_TO_BE_CHANGED_FLAG[MotorIndex] = 1

        return 1
    ##########################################################################################################
    ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    def SetCurrent_FROM_EXTERNAL_PROGRAM(self, MotorIndex, CurrentFromExternalProgram, Units = "None", NumberOfTimesToIssueCommand = 1):
        Units = Units.lower()

        #self.MyPrint_WithoutLogFile("CurrentFromExternalProgram: " + str(CurrentFromExternalProgram))

        if Units not in current_units_acceptable_list:
            self.MyPrint_WithoutLogFile("SetCurrent_FROM_EXTERNAL_PROGRAM ERROR: units of " + Units + " is not in the acceptable list of " + str(angular_units_acceptable_list))
            return 0

        if NumberOfTimesToIssueCommand < 1:
            self.MyPrint_WithoutLogFile("SetCurrent_FROM_EXTERNAL_PROGRAM ERROR: must set NumberOfTimesToIssueCommand >= 1")
            return 0

        CurrentFromExternalProgram_ConvertedToDynamixelUnits_unlimited = 0
        CurrentFromExternalProgram_ConvertedToDynamixelUnits_limited = 0

        CurrentFromExternalProgram_ConvertedToDynamixelUnits_unlimited = self.ConvertBetweenAllCurrentUnits(CurrentFromExternalProgram, Units)["dynamixelunits"]
        CurrentFromExternalProgram_ConvertedToDynamixelUnits_limited = self.LimitNumber_IntOutputOnly(self.Current_DynamixelUnits_min[MotorIndex], self.Current_DynamixelUnits_max[MotorIndex], CurrentFromExternalProgram_ConvertedToDynamixelUnits_unlimited)

        self.Current_DynamixelUnits_TO_BE_SET[MotorIndex] = CurrentFromExternalProgram_ConvertedToDynamixelUnits_limited
        self.Current_DynamixelUnits_NeedsToBeChangedFlag[MotorIndex] = NumberOfTimesToIssueCommand
        self.Current_DynamixelUnits_GUI_NeedsToBeChangedFlag[MotorIndex] = 1

        return 1
    ##########################################################################################################
    ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    def SetPosition_FROM_EXTERNAL_PROGRAM(self, MotorIndex, PositionFromExternalProgram, Units = "None"):
        Units = Units.lower()

        if Units not in angular_units_acceptable_list:
            self.MyPrint_WithoutLogFile("SetPosition_FROM_EXTERNAL_PROGRAM ERROR: units of " + Units + " is not in the acceptable list of " + str(angular_units_acceptable_list))
            return 0

        PositionFromExternalProgram_ConvertedToDynamixelUnits_unlimited = 0
        PositionFromExternalProgram_ConvertedToDynamixelUnits_limited = 0

        PositionFromExternalProgram_ConvertedToDynamixelUnits_unlimited = DynamixelProtocol2Xseries_ReubenPython3Class.ConvertBetweenAllAngularUnits(PositionFromExternalProgram, Units)["dynamixelunits"]
        PositionFromExternalProgram_ConvertedToDynamixelUnits_limited = self.LimitNumber_IntOutputOnly(self.Position_DynamixelUnits_Min_UserSet[MotorIndex], self.Position_DynamixelUnits_Max_UserSet[MotorIndex], PositionFromExternalProgram_ConvertedToDynamixelUnits_unlimited)

        self.Position_DynamixelUnits_TO_BE_SET[MotorIndex] = PositionFromExternalProgram_ConvertedToDynamixelUnits_limited
        self.Position_DynamixelUnits_NeedsToBeChangedFlag[MotorIndex] = 1
        self.Position_DynamixelUnits_GUI_NeedsToBeChangedFlag[MotorIndex] = 1

        return 1
    ##########################################################################################################
    ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    def SetVelocity_FROM_EXTERNAL_PROGRAM(self, MotorIndex, VelocityFromExternalProgram, Units = "None"):
        Units = Units.upper()

        if Units not in angular_speed_units_acceptable_list:
            self.MyPrint_WithoutLogFile("SetVelocity_FROM_EXTERNAL_PROGRAM ERROR: units of " + Units + " is not in the acceptable list of " + str(angular_speed_units_acceptable_list))
            return 0

        VelocityFromExternalProgram_unlimited = 0
        if Units == "NONE":
            VelocityFromExternalProgram_unlimited = VelocityFromExternalProgram

        if Units == "PERCENT":
            VelocityFromExternalProgram = self.LimitNumber_FloatOutputOnly(0.0, 100.0, VelocityFromExternalProgram) #To limit the input to [0,100]%
            VelocityFromExternalProgram_unlimited = ((self.Velocity_DynamixelUnits_max[MotorIndex] - self.Velocity_DynamixelUnits_Min_UserSet[MotorIndex])/100.0)*VelocityFromExternalProgram + self.Velocity_DynamixelUnits_Min_UserSet[MotorIndex]

        self.Velocity_DynamixelUnits_TO_BE_SET[MotorIndex] = self.LimitNumber_FloatOutputOnly(self.Velocity_DynamixelUnits_Min_UserSet[MotorIndex], self.Velocity_DynamixelUnits_max[MotorIndex], VelocityFromExternalProgram_unlimited)
        self.Velocity_DynamixelUnits_NeedsToBeChangedFlag[MotorIndex] = 1
        self.Velocity_DynamixelUnits_GUI_NeedsToBeChangedFlag[MotorIndex] = 1

        return 1
    ##########################################################################################################
    ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    def SetEngagedState_FROM_EXTERNAL_PROGRAM(self, MotorIndex, EngagedStateExternalProgram):

        if EngagedStateExternalProgram != 0 and EngagedStateExternalProgram != 1:
            self.MyPrint_WithoutLogFile("SetEngagedState_FROM_EXTERNAL_PROGRAM ERROR: EngagedState must be 0 or 1.")
            return 0

        #self.MyPrint_WithoutLogFile("SetEngagedState_FROM_EXTERNAL_PROGRAM changing EngagedState on motor " + str(MotorIndex) + " to a value of " + str(EngagedStateExternalProgram))

        self.EngagedState_TO_BE_SET[MotorIndex] = EngagedStateExternalProgram
        self.EngagedState_NeedsToBeChangedFlag[MotorIndex] = 1
        self.EngagedState_GUI_NeedsToBeChangedFlag[MotorIndex] = 1

        return 1
    ##########################################################################################################
    ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    def ToggleLEDstate_FROM_EXTERNAL_PROGRAM(self, MotorIndex):

        #self.MyPrint_WithoutLogFile("SetLEDstate_FROM_EXTERNAL_PROGRAM changing LEDstate on motor " + str(MotorIndex) + " to a value of " + str(LEDstateExternalProgram))

        if self.LEDstate[MotorIndex] == 0:
            self.SetLEDstate_FROM_EXTERNAL_PROGRAM(MotorIndex, 1)
        else:
            self.SetLEDstate_FROM_EXTERNAL_PROGRAM(MotorIndex, 0)


        return 1
    ##########################################################################################################
    ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    def SetLEDstate_FROM_EXTERNAL_PROGRAM(self, MotorIndex, LEDstateExternalProgram):

        if LEDstateExternalProgram != 0 and LEDstateExternalProgram != 1:
            self.MyPrint_WithoutLogFile("SetLEDstate_FROM_EXTERNAL_PROGRAM ERROR: LEDstate must be 0 or 1.")
            return 0

        #self.MyPrint_WithoutLogFile("SetLEDstate_FROM_EXTERNAL_PROGRAM changing LEDstate on motor " + str(MotorIndex) + " to a value of " + str(LEDstateExternalProgram))

        self.LEDstate_TO_BE_SET[MotorIndex] = LEDstateExternalProgram
        self.LEDstate_NeedsToBeChangedFlag[MotorIndex] = 1
        self.LEDstate_GUI_NeedsToBeChangedFlag[MotorIndex] = 1

        return 1
    ##########################################################################################################
    ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    def UpdateFrequencyCalculation_Tx_CalculatedFromMainThread_Filtered(self):

        try:
            self.DataStreamingDeltaT_Tx_CalculatedFromMainThread = self.CurrentTime_Tx_CalculatedFromMainThread - self.LastTime_Tx_CalculatedFromMainThread

            if self.DataStreamingDeltaT_Tx_CalculatedFromMainThread != 0.0:
                DataStreamingFrequency_Tx_CalculatedFromMainThread_TEMP = 1.0/self.DataStreamingDeltaT_Tx_CalculatedFromMainThread
                self.DataStreamingFrequency_Tx_CalculatedFromMainThread = self.DataStreamingFrequency_Tx_CalculatedFromMainThread_LowPassFilter_ReubenPython2and3ClassObject.AddDataPointFromExternalProgram(DataStreamingFrequency_Tx_CalculatedFromMainThread_TEMP)["SignalOutSmoothed"]

            self.LastTime_Tx_CalculatedFromMainThread = self.CurrentTime_Tx_CalculatedFromMainThread
        except:
            exceptions = sys.exc_info()[0]
            print("UpdateFrequencyCalculation_DedicatedTxThread_Filtered ERROR with Exceptions: %s" % exceptions)
            traceback.print_exc()
    ##########################################################################################################
    ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    def UpdateFrequencyCalculation_Rx_CalculatedFromMainThread_Filtered(self):

        try:
            self.DataStreamingDeltaT_Rx_CalculatedFromMainThread = self.CurrentTime_Rx_CalculatedFromMainThread - self.LastTime_Rx_CalculatedFromMainThread

            if self.DataStreamingDeltaT_Rx_CalculatedFromMainThread != 0.0:
                DataStreamingFrequency_Rx_CalculatedFromMainThread_TEMP = 1.0/self.DataStreamingDeltaT_Rx_CalculatedFromMainThread
                self.DataStreamingFrequency_Rx_CalculatedFromMainThread = self.DataStreamingFrequency_Rx_CalculatedFromMainThread_LowPassFilter_ReubenPython2and3ClassObject.AddDataPointFromExternalProgram(DataStreamingFrequency_Rx_CalculatedFromMainThread_TEMP)["SignalOutSmoothed"]

            self.LastTime_Rx_CalculatedFromMainThread = self.CurrentTime_Rx_CalculatedFromMainThread
        except:
            exceptions = sys.exc_info()[0]
            print("UpdateFrequencyCalculation_DedicatedRxThread_Filtered ERROR with Exceptions: %s" % exceptions)
            traceback.print_exc()
    ##########################################################################################################
    ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    def UpdateFrequencyCalculation_RealTimeTicksMillisecFromDynamixel(self, MotorIndex):

        try:
            self.DataStreamingDeltaT_RealTimeTicksMillisecFromDynamixel[MotorIndex] = (self.RealTimeTicksMillisec[MotorIndex] - self.RealTimeTicksMillisec_last[MotorIndex])*0.001

            ##########################
            if self.DataStreamingDeltaT_RealTimeTicksMillisecFromDynamixel[MotorIndex] != 0.0:
                self.DataStreamingFrequency_RealTimeTicksMillisecFromDynamixel[MotorIndex] = 1.0/self.DataStreamingDeltaT_RealTimeTicksMillisecFromDynamixel[MotorIndex]
            ##########################

            self.RealTimeTicksMillisec_last[MotorIndex] = self.RealTimeTicksMillisec[MotorIndex]

        except:
            exceptions = sys.exc_info()[0]
            self.MyPrint_WithoutLogFile("UpdateFrequencyCalculation_RealTimeTicksMillisecDynamixel ERROR, exceptions: %s" % exceptions)
            #traceback.print_exc()
    ##########################################################################################################
    ##########################################################################################################

    #######################################################################################################################
    def ConvertOperatingModeStringToInt(self, OperatingModeString):

        if OperatingModeString == "CurrentControl":
            OperatingModeInt = 0
        elif OperatingModeString == "VelocityControl":
            OperatingModeInt = 1
        elif OperatingModeString == "PositionControl":
            OperatingModeInt = 3
        elif OperatingModeString == "ExtendedPositionControlMultiTurn":
            OperatingModeInt = 4
        elif OperatingModeString == "CurrentBasedPositionControl":
            OperatingModeInt = 5
        elif OperatingModeString == "PWMcontrol":
            OperatingModeInt = 16
        else:
            OperatingModeInt = -1
            #self.MyPrint_WithoutLogFile("ConvertOperatingModeStringToInt ERROR: OperatingModeString '" + OperatingModeString + "' is invalid.")

        return OperatingModeInt
    #######################################################################################################################

    #######################################################################################################################
    def ConvertOperatingModeIntToString(self, OperatingModeInt):

        if OperatingModeInt == 0:
            OperatingModeString = "CurrentControl"
        elif OperatingModeInt == 1:
            OperatingModeString = "VelocityControl"
        elif OperatingModeInt == 3:
            OperatingModeString = "PositionControl"
        elif OperatingModeInt == 4:
            OperatingModeString = "ExtendedPositionControlMultiTurn"
        elif OperatingModeInt == 5:
            OperatingModeString = "CurrentBasedPositionControl"
        elif OperatingModeInt == 16:
            OperatingModeString = "PWMcontrol"
        else:
            OperatingModeString = "invalid"
            #self.MyPrint_WithoutLogFile("ConvertOperatingModeIntToString ERROR: OperatingModeInt '" + str(OperatingModeInt) + "' is invalid.")

        return OperatingModeString
    #######################################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    def ResetWatchdogTimerInMilliseconds(self, MotorID):
        self.SetWatchdogTimerInMilliseconds(MotorID, 0)
    ##########################################################################################################
    ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    def SetWatchdogTimerInMilliseconds(self, MotorID, WatchdogTimerDurationMilliseconds_Input):

        if WatchdogTimerDurationMilliseconds_Input >= 0:

            ADDR_PRO_GOAL_Watchdog = 98

            WatchdogTimerValueToWrite_limited = int(self.LimitNumber_IntOutputOnly(0, 127, round(WatchdogTimerDurationMilliseconds_Input/20.0))) #1 is the actual minimum, but 0 clears the watchdog timer

            print("SetWatchdogTimerInMilliseconds on motor " + str(MotorID) + " to a value of " + str(WatchdogTimerValueToWrite_limited))

            dxl_comm_result = self.packetHandler.write1ByteTxOnly(self.portHandler, MotorID, ADDR_PRO_GOAL_Watchdog, 1)

            if dxl_comm_result != COMM_SUCCESS:
                dummy_var = 0
                print("SetWatchdogTimerInMilliseconds, %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            else:
                self.WatchdogTimerDurationMilliseconds[MotorID] = WatchdogTimerValueToWrite_limited

    ##########################################################################################################
    ##########################################################################################################


    #######################################################################################################################
    #######################################################################################################################
    def SendInstructionPacket_SetOperatingMode(self, MotorID, OperatingModeString, print_bytes_for_debugging = 0):

        print("SendInstructionPacket_SetOperatingMode event fired!")

        ############################################################
        ############################################################ FOR SOME REASON, SendInstructionPacket_SetOperatingMode ONLY WORKS IF WE FIRST REBOOT THE MOTOR
        self.SendInstructionPacket_Reboot(MotorID)
        time.sleep(0.10) #Have to wait a long time after the REBOOT for this to work
        ############################################################
        ############################################################

        OperatingModeInt = self.ConvertOperatingModeStringToInt(OperatingModeString)

        print("OperatingModeString: " + OperatingModeString)
        print("OperatingModeInt: " + str(OperatingModeInt))

        ADDR_PRO_OPERATINGMODE = 11

        dxl_comm_result = self.packetHandler.write1ByteTxOnly(self.portHandler, MotorID, ADDR_PRO_OPERATINGMODE, OperatingModeInt)
        if dxl_comm_result != COMM_SUCCESS:
            self.MyPrint_WithoutLogFile("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        else:
            dummy = 0
            self.OperatingMode[MotorID] = OperatingModeString
            print("@@@@@@@@@@@@@@@@@@@@ SendInstructionPacket_SetOperatingMode set operating mode to " + self.OperatingMode[MotorID] + " on motor " + str(MotorID) + " @@@@@@@@@@@@@@@@@@@@")

            if OperatingModeString == "CurrentControl":
                self.Current_DynamixelUnits_NeedsToBeChangedFlag[MotorID] = 1
            elif OperatingModeString == "VelocityControl":
                self.Velocity_DynamixelUnits_NeedsToBeChangedFlag[MotorID] = 1
            elif OperatingModeString == "PositionControl":
                self.Position_DynamixelUnits_NeedsToBeChangedFlag[MotorID] = 1
            elif OperatingModeString == "ExtendedPositionControlMultiTurn":
                self.Position_DynamixelUnits_NeedsToBeChangedFlag[MotorID] = 1
            elif OperatingModeString == "CurrentBasedPositionControl":
                self.Position_DynamixelUnits_NeedsToBeChangedFlag[MotorID] = 1
                self.Current_DynamixelUnits_NeedsToBeChangedFlag[MotorID] = 1
            elif OperatingModeString == "PWMcontrol":
                self.PWM_DynamixelUnits_NeedsToBeChangedFlag[MotorID] = 1

            self.EngagedState_TO_BE_SET[MotorID] = 1  # From page 9 of motor manual: "After setting the operating mode (11) to speed control mode, change the Torque Enable (64) to '1'."
            self.EngagedState_NeedsToBeChangedFlag[MotorID] = 1

        return 1
    #######################################################################################################################
    #######################################################################################################################

    #######################################################################################################################
    #######################################################################################################################
    def SendInstructionPacket_SetShutdown(self, MotorID, EnableSafetyShutoff, print_bytes_for_debugging = 0):

        if EnableSafetyShutoff in [0, 1]:

            if EnableSafetyShutoff == 1:
                ShutdownValue_Decimal = 52
            else:
                ShutdownValue_Decimal = 0

            ADDR_PRO_SHUTDOWN = 63

            dxl_comm_result = self.packetHandler.write1ByteTxOnly(self.portHandler, MotorID, ADDR_PRO_SHUTDOWN, ShutdownValue_Decimal)
            if dxl_comm_result != COMM_SUCCESS:
                self.MyPrint_WithoutLogFile("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            else:
                dummy = 0

            self.EnableSafetyShutoff[MotorID] = EnableSafetyShutoff

            print("SendInstructionPacket_SetShutdown event fired for MotorID " + str(MotorID) + " with EnabledSafetyShutoff state " + str(self.EnableSafetyShutoff[MotorID]) + " and decimal value " + str(ShutdownValue_Decimal))

        else:
            print("SendInstructionPacket_SetShutdown error: EnableSafetyShutoff must be 0 or 1.")
    #######################################################################################################################
    #######################################################################################################################

    #######################################################################################################################
    #######################################################################################################################
    def SendInstructionPacket_SetLED(self, MotorID, LEDstate, print_bytes_for_debugging = 0):

        ADDR_PRO_LED = 65

        dxl_comm_result = self.packetHandler.write1ByteTxOnly(self.portHandler, MotorID, ADDR_PRO_LED, LEDstate)
        if dxl_comm_result != COMM_SUCCESS:
            self.MyPrint_WithoutLogFile("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        else:
            dummy = 0

        self.LEDstate[MotorID] = LEDstate
    #######################################################################################################################
    #######################################################################################################################

    #######################################################################################################################
    #######################################################################################################################
    def SendInstructionPacket_SetLEDalarmSettings(self, MotorID, LEDalarmSettingsByte, print_bytes_for_debugging = 0):
        dummy = 0

    #######################################################################################################################
    #######################################################################################################################

    #######################################################################################################################
    #######################################################################################################################
    def SendInstructionPacket_SetID(self, MotorID, IDToSet, print_bytes_for_debugging = 0):

        ############################################
        ############################################
        #In order to change the ID in the EEPROM Area, Torque Enable(64) has to be cleared to "0" in advance.
        EngagedState_temp = self.EngagedState[MotorID]
        self.SendInstructionPacket_SetTorqueEnable(MotorID, 0) #In order to change the ID in the EEPROM Area, Torque Enable(64) has to be cleared to "0" in advance.
        time.sleep(0.002)
        ############################################
        ############################################

        ADDR_PRO_ID = 7

        dxl_comm_result = self.packetHandler.write1ByteTxOnly(self.portHandler, MotorID, ADDR_PRO_ID, IDToSet)
        if dxl_comm_result != COMM_SUCCESS:
            self.MyPrint_WithoutLogFile("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        else:
            dummy = 0
        time.sleep(0.002)

        ############################################
        ############################################
        self.SendInstructionPacket_SetTorqueEnable(MotorID, EngagedState_temp)
        time.sleep(0.002)
        ############################################
        ############################################

    #######################################################################################################################
    #######################################################################################################################

    #######################################################################################################################
    #######################################################################################################################
    def SendInstructionPacket_SetBaudRate(self, MotorID, BaudRateToSet, print_bytes_for_debugging = 0):

        ADDR_PRO_BAUD_RATE = 8

        dxl_comm_result = self.packetHandler.write1ByteTxOnly(self.portHandler, MotorID, ADDR_PRO_BAUD_RATE, 6)
        if dxl_comm_result != COMM_SUCCESS:
            dummy_var = 0
            #self.MyPrint_WithoutLogFile("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        else:
            dummy = 0

    #######################################################################################################################
    #######################################################################################################################

    #######################################################################################################################
    #######################################################################################################################
    def SendInstructionPacket_SetTorqueEnable(self, MotorID, TorqueEnableState, print_bytes_for_debugging = 0):

        ADDR_PRO_TORQUE_ENABLE = 64

        dxl_comm_result = self.packetHandler.write1ByteTxOnly(self.portHandler, MotorID, ADDR_PRO_TORQUE_ENABLE, TorqueEnableState)
        if dxl_comm_result != COMM_SUCCESS:
            dummy_var = 0
            #self.MyPrint_WithoutLogFile("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        else:
            dummy = 0
            self.EngagedState[MotorID] = TorqueEnableState

        #self.MyPrint_WithoutLogFile("SendInstructionPacket_SetTorqueEnable fired!")
    #######################################################################################################################
    #######################################################################################################################

    #######################################################################################################################
    #######################################################################################################################
    def ResetSerial(self, print_bytes_for_debugging = 0):

        self.portHandler.clearPort()
        self.portHandler.closePort()
        time.sleep(0.025)
        self.portHandler.openPort()

        for MotorIndex in range(0, self.NumberOfMotors):
            self.SendInstructionPacket_SetStatusReturnLevel(MotorIndex, 1)  # 0: Do not respond to any instructions, 1: Respond only to READ_DATA instructions, 2: Respond to all instructions
            time.sleep(self.TimeToWaitBetweenCriticalInstructions)

        self.MyPrint_WithoutLogFile("########## ResetSerial! ##########")
    #######################################################################################################################
    #######################################################################################################################

    #######################################################################################################################
    #######################################################################################################################
    def SendInstructionPacket_Reboot(self, MotorID, print_bytes_for_debugging = 0):

        dxl_comm_result = self.packetHandler.reboot(self.portHandler, MotorID)
        if dxl_comm_result != COMM_SUCCESS:
            dummy_var = 0
            #self.MyPrint_WithoutLogFile("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        else:
            dummy = 0

        self.MyPrint_WithoutLogFile("SendInstructionPacket_Reboot: REBOOTED MOTOR " + str(MotorID))
    #######################################################################################################################
    #######################################################################################################################

    #######################################################################################################################
    #######################################################################################################################
    def SendInstructionPacket_ReturnDelayTime(self, MotorID, ReturnDelayTime, print_bytes_for_debugging = 0):

        ReturnDelayTime_limited = int(self.LimitNumber_IntOutputOnly(0, 254, ReturnDelayTime))

        ADDR_PRO_RETURN_DELAY_TIME = 9

        dxl_comm_result = self.packetHandler.write1ByteTxOnly(self.portHandler, MotorID, ADDR_PRO_RETURN_DELAY_TIME, ReturnDelayTime_limited)
        if dxl_comm_result != COMM_SUCCESS:
            dummy_var = 0
            #self.MyPrint_WithoutLogFile("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        else:
            dummy = 0
    #######################################################################################################################
    #######################################################################################################################

    #######################################################################################################################
    #######################################################################################################################
    def SendInstructionPacket_SetStatusReturnLevel(self, MotorID, StatusReturnLevel, print_bytes_for_debugging = 0):

        # 0: Do not respond to any instructions
        # 1: Respond only to READ_DATA instructions
        # 2: Respond to all instructions

        if StatusReturnLevel == 0 or StatusReturnLevel == 1 or StatusReturnLevel == 2:

            ADDR_PRO_STATUS_RETURN_LEVEL = 68

            dxl_comm_result = self.packetHandler.write1ByteTxOnly(self.portHandler, MotorID, ADDR_PRO_STATUS_RETURN_LEVEL, int(StatusReturnLevel))
            if dxl_comm_result != COMM_SUCCESS:
                dummy_var = 0
                #self.MyPrint_WithoutLogFile("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            else:
                dummy = 0
        else:
            self.MyPrint_WithoutLogFile("SendInstructionPacket_SetStatusReturnLevel ERROR: StatusReturnLevel must be 0, 1, or 2.")
    #######################################################################################################################
    #######################################################################################################################

    #######################################################################################################################
    #######################################################################################################################
    def SendInstructionPacket_SetPosition(self, MotorID, Position, print_bytes_for_debugging = 0):

        try:
            ADDR_PRO_GOAL_POSITION = 116

            Position_limited = int(self.LimitNumber_IntOutputOnly(self.Position_DynamixelUnits_Min_UserSet[MotorID], self.Position_DynamixelUnits_Max_UserSet[MotorID], Position))

            #if Position_limited < 0:
            #    Position_limited = self.ComputeTwosComplement(Position_limited) #NOT NEEDED

            dxl_comm_result = self.packetHandler.write4ByteTxOnly(self.portHandler, MotorID, ADDR_PRO_GOAL_POSITION, Position_limited)
            if dxl_comm_result != COMM_SUCCESS:
                #dummy_var = 0
                self.MyPrint_WithoutLogFile("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            else:
                dummy = 0

            self.Position_DynamixelUnits[MotorID] = Position_limited

            self.SendInstructionPacket_SetPosition_Counter[MotorID] = self.SendInstructionPacket_SetPosition_Counter[MotorID] + 1

            if print_bytes_for_debugging == 1:
                print("SendInstructionPacket_SetPosition: Sent Position_limited command of " + str(Position_limited) + " to MotorID " + str(MotorID))

        except:
            exceptions = sys.exc_info()[0]
            print("SendInstructionPacket_SetPosition: Exceptions: %s" % exceptions)
            traceback.print_exc()
    #######################################################################################################################
    #######################################################################################################################

    #######################################################################################################################
    #######################################################################################################################
    def SendInstructionPacket_SetGoalCurrent_ForCurrentAndCurrentBasedPositionControl(self, MotorID, Current, print_bytes_for_debugging = 0):

        ADDR_PRO_GOAL_Current = 102

        Current_limited = int(self.LimitNumber_IntOutputOnly(-2047.0, 2047.0, Current))

        if Current_limited < 0:
            Current_limited_TwosComplement = self.ComputeTwosComplement(Current_limited, 16) #2 bytes = 16bits
            dxl_comm_result = self.packetHandler.write2ByteTxOnly(self.portHandler, MotorID, ADDR_PRO_GOAL_Current, Current_limited_TwosComplement)
        else:
            dxl_comm_result = self.packetHandler.write2ByteTxOnly(self.portHandler, MotorID, ADDR_PRO_GOAL_Current, Current_limited)

        if dxl_comm_result != COMM_SUCCESS:
            dummy_var = 0
            #self.MyPrint_WithoutLogFile("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        else:
            dummy = 0
            self.Current_DynamixelUnits[MotorID] = Current_limited
    #######################################################################################################################
    #######################################################################################################################

    #######################################################################################################################
    #######################################################################################################################
    def SendInstructionPacket_SetCurrentLimit(self, MotorID, CurrentLimit, print_bytes_for_debugging = 0):

        ########### IT SEEMS LIKE THIS FUNCTION LIKES TO DECREASE THE CURRENT LIMIT BUT NEVER INCREASE IT

        ####################################################
        EngagedState_temp = self.EngagedState[MotorID]
        self.SendInstructionPacket_SetTorqueEnable(MotorID, 0) #DYNAMIXEL LIMITS CAN ONLY BE CHANGED WITH TORQUE DISABLED
        ####################################################

        ####################################################
        CurrentLimit_limited = int(self.LimitNumber_IntOutputOnly(0.0, 2047.0, CurrentLimit))

        ADDR_PRO_CURRENT_LIMIT = 38

        dxl_comm_result = self.packetHandler.write2ByteTxOnly(self.portHandler, MotorID, ADDR_PRO_CURRENT_LIMIT, CurrentLimit_limited)
        if dxl_comm_result != COMM_SUCCESS:
            dummy_var = 0
            #self.MyPrint_WithoutLogFile("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        else:
            dummy = 0

        #self.Current_DynamixelUnits_max[MotorID] = CurrentLimit_limited
        ####################################################

        ####################################################
        self.SendInstructionPacket_SetTorqueEnable(MotorID, EngagedState_temp) #Must set the EngagedState back to what it was before we changed this limit
        ####################################################

        #self.MyPrint_WithoutLogFile("SendInstructionPacket_SetCurrentLimit EVENT FIRED!")
    #######################################################################################################################
    #######################################################################################################################

    #######################################################################################################################
    #######################################################################################################################
    def SendInstructionPacket_SetGoalVelocity_ForVelocityModeOnly(self, MotorID, Velocity, print_bytes_for_debugging = 0):

        ADDR_PRO_GOAL_VELOCITY = 104

        Velocity_limited = int(self.LimitNumber_IntOutputOnly(-1023.0, 1023.0, Velocity))

        if Velocity_limited < 0:
            Velocity_limited_TwosComplement = self.ComputeTwosComplement(Velocity_limited, 32) #4 bytes = 32bits
            #print("negative: " + str('{0:016b}'.format(Velocity_limited_TwosComplement)))

            dxl_comm_result = self.packetHandler.write4ByteTxOnly(self.portHandler, MotorID, ADDR_PRO_GOAL_VELOCITY, Velocity_limited_TwosComplement)
        else:
            #print('{0:016b}'.format(Velocity_limited))
            dxl_comm_result = self.packetHandler.write4ByteTxOnly(self.portHandler, MotorID, ADDR_PRO_GOAL_VELOCITY, Velocity_limited)

        if dxl_comm_result != COMM_SUCCESS:
            dummy_var = 0
            #self.MyPrint_WithoutLogFile("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        else:
            dummy = 0
            #self.MyPrint_WithoutLogFile("SendInstructionPacket_SetGoalVelocity_ForVelocityModeOnly fired!")
            self.Velocity_DynamixelUnits[MotorID] = Velocity_limited
    #######################################################################################################################
    #######################################################################################################################

    #######################################################################################################################
    #######################################################################################################################
    def SendInstructionPacket_SetMinPositionLimit_PositionModeOnly(self, MotorID, MinPositionLimit, print_bytes_for_debugging = 0):

        # $$$$$$$$$$$$$ These values are not used in Extended Position Control Mode and Current-based Position Control Mode.
        if self.OperatingMode == "PositionControl":
            ####################################################
            EngagedState_temp = self.EngagedState[MotorID]
            self.SendInstructionPacket_SetTorqueEnable(MotorID, 0) #DYNAMIXEL LIMITS CAN ONLY BE CHANGED WITH TORQUE DISABLED
            ####################################################

            ####################################################
            MinPositionLimit_limited = int(self.LimitNumber_IntOutputOnly(0.0, self.Position_DynamixelUnits_Max_FWlimit_SingleTurn, MinPositionLimit))

            ADDR_PRO_MIN_POSITION_LIMIT = 52

            dxl_comm_result = self.packetHandler.write4ByteTxOnly(self.portHandler, MotorID, ADDR_PRO_MIN_POSITION_LIMIT, MinPositionLimit_limited)
            if dxl_comm_result != COMM_SUCCESS:
                dummy_var = 0
                #self.MyPrint_WithoutLogFile("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            else:
                dummy = 0
            ####################################################

            ####################################################
            self.SendInstructionPacket_SetTorqueEnable(MotorID, EngagedState_temp) #Must set the EngagedState back to what it was before we changed this limit
            ####################################################

        else:
            self.MyPrint_WithoutLogFile("SendInstructionPacket_SetMinPositionLimit_PositionModeOnly ERROR, function can only be called in 'PositionMode' (single-turn).")

    #######################################################################################################################
    #######################################################################################################################

    #######################################################################################################################
    #######################################################################################################################
    def SendInstructionPacket_SetMaxPositionLimit_PositionModeOnly(self, MotorID, MaxPositionLimit, print_bytes_for_debugging=0):

        #$$$$$$$$$$$$$ These values are not used in Extended Position Control Mode and Current-based Position Control Mode.
        if self.OperatingMode == "PositionControl":
            ####################################################
            EngagedState_temp = self.EngagedState[MotorID]
            self.SendInstructionPacket_SetTorqueEnable(MotorID, 0)  # DYNAMIXEL LIMITS CAN ONLY BE CHANGED WITH TORQUE DISABLED
            ####################################################

            ####################################################
            MaxPositionLimit_limited = int(self.LimitNumber_IntOutputOnly(0.0, self.Position_DynamixelUnits_Max_FWlimit_SingleTurn, MaxPositionLimit))

            ADDR_PRO_Max_POSITION_LIMIT = 48

            dxl_comm_result = self.packetHandler.write4ByteTxOnly(self.portHandler, MotorID, ADDR_PRO_Max_POSITION_LIMIT, MaxPositionLimit_limited)
            if dxl_comm_result != COMM_SUCCESS:
                dummy_var = 0
                #self.MyPrint_WithoutLogFile("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            else:
                dummy = 0
            ####################################################

            ####################################################
            self.SendInstructionPacket_SetTorqueEnable(MotorID, EngagedState_temp)  # Must set the EngagedState back to what it was before we changed this limit
            ####################################################

        else:
            self.MyPrint_WithoutLogFile("SendInstructionPacket_SetMinPositionLimit_PositionModeOnly ERROR, function can only be called in 'PositionMode' (single-turn).")

    #######################################################################################################################
    #######################################################################################################################

    #######################################################################################################################
    #######################################################################################################################
    def SendInstructionPacket_SetMaxVelocity(self, MotorID, MaxVelocity, print_bytes_for_debugging = 0):

        #This value indicates maximum velocity of Goal Velocity(104) and Profile Velocity(112). For more details, please refer to the Profile Velocity(112).

        ####################################################
        EngagedState_temp = self.EngagedState[MotorID]
        #self.SendInstructionPacket_SetTorqueEnable(MotorID, 0) #DYNAMIXEL LIMITS CAN ONLY BE CHANGED WITH TORQUE DISABLED
        self.SetEngagedState_FROM_EXTERNAL_PROGRAM(MotorID, 0) #DYNAMIXEL LIMITS CAN ONLY BE CHANGED WITH TORQUE DISABLED
        ####################################################

        ####################################################
        Velocity_DynamixelUnits_max_limited = int(self.LimitNumber_IntOutputOnly(0.0, 1023.0, MaxVelocity)) #NOT 2047

        ADDR_PRO_MAX_VELOCITY_LIMIT = 44

        dxl_comm_result = self.packetHandler.write4ByteTxOnly(self.portHandler, MotorID, ADDR_PRO_MAX_VELOCITY_LIMIT, Velocity_DynamixelUnits_max_limited)
        if dxl_comm_result != COMM_SUCCESS:
            dummy_var = 0
            #self.MyPrint_WithoutLogFile("SendInstructionPacket_SetMaxVelocity %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        else:
            dummy = 0

        self.Velocity_DynamixelUnits_Max_UserSet[MotorID] = Velocity_DynamixelUnits_max_limited
        ####################################################

        ####################################################
        #self.SendInstructionPacket_SetTorqueEnable(MotorID, EngagedState_temp) #Must set the EngagedState back to what it was before we changed this limit
        self.SetEngagedState_FROM_EXTERNAL_PROGRAM(MotorID, EngagedState_temp) #Must set the EngagedState back to what it was before we changed this limit
        ####################################################

        if print_bytes_for_debugging == 1:
            print("SendInstructionPacket_SetMaxVelocity event fired for motor " + str(MotorID))
    #######################################################################################################################
    #######################################################################################################################

    #######################################################################################################################
    #######################################################################################################################
    def SendInstructionPacket_SetProfileVelocity_ForAllModesExceptCurrentAndVelocity(self, MotorID, ProfileVelocity, print_bytes_for_debugging = 0):
        '''
        Described on page 9 of motor manual
        The Maximum velocity of Profile can be set with this value.
        Profile Velocity(112) can be used in all control modes except Torque Control Mode and Velocity Control Mode.
        Profile Velocity(112) cannot exceed Velocity Limit(44).
        Velocity Control Mode only uses Profile Acceleration(108) instead of Profile Velocity(112).
        '''

        if self.OperatingMode != "CurrentControl" and self.OperatingMode != "VelocityControl":
            ProfileVelocity_limited = int(self.LimitNumber_IntOutputOnly(self.Velocity_DynamixelUnits_Min_UserSet[MotorID], self.Velocity_DynamixelUnits_Max_UserSet[MotorID], ProfileVelocity))

            ADDR_PRO_PROFILE_VELOCITY_LIMIT = 112

            dxl_comm_result = self.packetHandler.write4ByteTxOnly(self.portHandler, MotorID, ADDR_PRO_PROFILE_VELOCITY_LIMIT, ProfileVelocity_limited)
            if dxl_comm_result != COMM_SUCCESS:
                dummy_var = 0
                #self.MyPrint_WithoutLogFile("SendInstructionPacket_SetProfileVelocity_ForAllModesExceptCurrentAndVelocity %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            else:
                dummy = 0

            if print_bytes_for_debugging == 1:
                print("SendInstructionPacket_SetProfileVelocity_ForAllModesExceptCurrentAndVelocity event fired for MotorID " + str(MotorID) + ", ProfileVelocity_limited: " + str(ProfileVelocity_limited))

    #######################################################################################################################
    #######################################################################################################################

    #######################################################################################################################
    #######################################################################################################################
    def SendInstructionPacket_SetMaxPWM(self, MotorID, MaxPWM, print_bytes_for_debugging = 0):
        ####################################################
        EngagedState_temp = self.EngagedState[MotorID]
        self.SendInstructionPacket_SetTorqueEnable(MotorID, 0) #DYNAMIXEL LIMITS CAN ONLY BE CHANGED WITH TORQUE DISABLED
        ####################################################

        MaxPWM_limited = int(self.LimitNumber_IntOutputOnly(0.0, 885.0, MaxPWM))

        ADDR_PRO_MAX_PWM_LIMIT = 36

        dxl_comm_result = self.packetHandler.write2ByteTxOnly(self.portHandler, MotorID, ADDR_PRO_MAX_PWM_LIMIT, MaxPWM_limited)
        if dxl_comm_result != COMM_SUCCESS:
            dummy_var = 0
            #self.MyPrint_WithoutLogFile("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        else:
            dummy = 0

        self.MaxPWM_DynamixelUnits[MotorID] = MaxPWM_limited

        ####################################################
        self.SendInstructionPacket_SetTorqueEnable(MotorID, EngagedState_temp) #Must set the EngagedState back to what it was before we changed this limit
        ####################################################
    #######################################################################################################################
    #######################################################################################################################

    #######################################################################################################################
    #######################################################################################################################
    def SendInstructionPacket_SetPWM(self, MotorID, GoalPWM, print_bytes_for_debugging = 0):

        GoalPWM_limited = int(self.LimitNumber_IntOutputOnly(0.0, 885.0, GoalPWM))

        ADDR_PRO_GOAL_PWM = 100

        dxl_comm_result = self.packetHandler.write2ByteTxOnly(self.portHandler, MotorID, ADDR_PRO_GOAL_PWM, GoalPWM_limited)
        if dxl_comm_result != COMM_SUCCESS:
            dummy_var = 0
            #self.MyPrint_WithoutLogFile("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        else:
            dummy = 0

        self.PWM_DynamixelUnits[MotorID] = GoalPWM_limited
    #######################################################################################################################
    #######################################################################################################################

    #######################################################################################################################
    #######################################################################################################################
    def PingMotor(self, MotorID, print_bytes_for_debugging = 0):

        dummy = 0

    #######################################################################################################################
    #######################################################################################################################

    #######################################################################################################################
    #######################################################################################################################
    def ConvertUnsignedToSignedInteger(self, IntegerToBeConverted, NumberOfBytesComposingInteger):
        return int.from_bytes(IntegerToBeConverted.to_bytes(NumberOfBytesComposingInteger, 'little', signed=False), 'little', signed=True)
    #######################################################################################################################
    #######################################################################################################################

    #######################################################################################################################
    #######################################################################################################################
    def ConvertSignedToUnsignedInteger(self, IntegerToBeConverted, NumberOfBytesComposingInteger):
        return int.from_bytes(IntegerToBeConverted.to_bytes(NumberOfBytesComposingInteger, 'little', signed=True), 'little', signed=False)
    #######################################################################################################################
    #######################################################################################################################

    #######################################################################################################################
    #######################################################################################################################
    def ReadVariable(self, MotorID, VariableNameString, print_bytes_for_debugging = 0):

        try:
            #print("ReadVariable event fired!")

            PortRegister = -1
            ReadLength = -1
            dxl_data = -1
            dxl_comm_result = -1
            dxl_error = -1
            Data_Value = -1111111111

            ##########################################################
            if VariableNameString == "Baud":
                PortRegister = 8
                ReadLength = 1
            elif VariableNameString == "OperatingMode":
                PortRegister = 11
                ReadLength = 1
            elif VariableNameString == "TemperatureLimit":
                PortRegister = 31
                ReadLength = 1
            elif VariableNameString == "MaxVoltageLimit":
                PortRegister = 32
                ReadLength = 2
            elif VariableNameString == "MinVoltageLimit":
                PortRegister = 34
                ReadLength = 2
            elif VariableNameString == "PWMlimit":
                PortRegister = 36
                ReadLength = 2
            elif VariableNameString == "CurrentLimit":
                PortRegister = 38
                ReadLength = 2
            elif VariableNameString == "AccelerationLimit":
                PortRegister = 40
                ReadLength = 4
            elif VariableNameString == "VelocityLimit":
                PortRegister = 44
                ReadLength = 4
            elif VariableNameString == "HardwareErrorStatus":
                PortRegister = 70
                ReadLength = 1
            elif VariableNameString == "RealtimeTick":
                PortRegister = 120
                ReadLength = 2
            elif VariableNameString == "Moving":
                PortRegister = 122
                ReadLength = 1
            elif VariableNameString == "MovingStatus":
                PortRegister = 123
                ReadLength = 1
            elif VariableNameString == "PresentPWM":
                PortRegister = 124
                ReadLength = 2
            elif VariableNameString == "PresentCurrent":
                PortRegister = 126
                ReadLength = 2
            elif VariableNameString == "PresentVelocity":
                PortRegister = 128
                ReadLength = 4
            elif VariableNameString == "PresentPosition":
                PortRegister = 132
                ReadLength = 4
            elif VariableNameString == "PresentInputVoltage":
                PortRegister = 144
                ReadLength = 2
            elif VariableNameString == "PresentTemperature":
                PortRegister = 146
                ReadLength = 1 #Should be 1 according to the datasheet, but 2 works and I confirmed here: https://github.com/ROBOTIS-GIT/DynamixelSDK/issues/152
            elif VariableNameString == "Shutdown":
                PortRegister = 63
                ReadLength = 1
            else:
                self.print("$$$$$$$$$$$$$$$$$$$$$$$$$ ReadVariable ERROR: Variable name '" + VariableNameString + " is no valid! $$$$$$$$$$$$$$$$$$$$$$$$$")
                return "error_VariableNameString"
            ##########################################################

            ##########################################################
            if ReadLength == 1:
                try:
                    dxl_data, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, MotorID, PortRegister)
                except:
                    exceptions = sys.exc_info()[0]
                    #print("ReadVariable, exceptions: %s" % exceptions)
                    #traceback.print_exc()

            elif ReadLength == 2:
                try:
                    dxl_data, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, MotorID, PortRegister)

                    dxl_data = self.ConvertUnsignedToSignedInteger(dxl_data, 2)

                except:
                    exceptions = sys.exc_info()[0]
                    #print("ReadVariable, exceptions: %s" % exceptions)
                    #traceback.print_exc()

            elif ReadLength == 4:
                try:
                    dxl_data, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, MotorID, PortRegister)

                    dxl_data = self.ConvertUnsignedToSignedInteger(dxl_data, 4)
                except:
                    exceptions = sys.exc_info()[0]
                    #print("ReadVariable, exceptions: %s" % exceptions)
                    #traceback.print_exc()
            ##########################################################

            ##########################################################
            if dxl_comm_result != COMM_SUCCESS:
                print("MotorID = " + str(MotorID) +", ReadVariable: error_serial_dxl_comm with dxl_data = " + str(dxl_data))
                self.portHandler.clearPort()
                return
            elif dxl_error != 0:
                print("MotorID = " + str(MotorID) + ", ReadVariable: error_serial_dxl_error with dxl_data = " + str(dxl_data))
                self.portHandler.clearPort()
            ##########################################################

            ##########################################################
            if VariableNameString == "PresentInputVoltage" or VariableNameString == "MaxVoltageLimit" or VariableNameString == "MinVoltageLimit":
                Data_Value = 0.1*dxl_data
                #if dxl_data >= 95 and dxl_data <= 160:
                #    Data_Value = dxl_data*0.1
                #else:
                #    return -1111111111

            elif VariableNameString == "PresentCurrent" or VariableNameString == "CurrentLimit":
                Data_Value = dxl_data/1000.0

                #if dxl_data >= 0 and dxl_data <= 100:
                #    Data_Value = dxl_data*1.0
                #else:
                #    return -1111111111

            elif VariableNameString == "PresentTemperature" or VariableNameString == "TemperatureLimit":
                Data_Value = dxl_data
                #if dxl_data >= 0 and dxl_data <= 100:
                #    Data_Value = dxl_data*1.0
                #else:
                #    return -1111111111

            elif VariableNameString == "HardwareErrorStatus":
                Data_Value = dxl_data

                self.ErrorFlag_BYTE[MotorID] = Data_Value
                # Bit 7 --> 0 (self.ErrorFlag_BYTE[MotorID] & 0b10000000)
                # Bit 6 --> 0 (self.ErrorFlag_BYTE[MotorID] & 0b01000000)
                self.ErrorFlag_Overload_Received[MotorID] = (self.ErrorFlag_BYTE[MotorID] & 0b00100000)  # Bit 5 --> Instruction Error. Set to 1 if an undefined instruction is sent or an action instruction is sent without a Reg_Write instruction.
                self.ErrorFlag_ElectricalShock_Received[MotorID] = (self.ErrorFlag_BYTE[MotorID] & 0b00010000)  # Bit 4 --> Overload Error. Set to 1 if the specified maximum torque can't control the applied load.
                self.ErrorFlag_MotorEncoder_Received[MotorID] = (self.ErrorFlag_BYTE[MotorID] & 0b00001000)  # Bit 3 --> Range Error. Set to 1 if the instruction sent is out of the defined range.
                self.ErrorFlag_Overheating_Received[MotorID] = (self.ErrorFlag_BYTE[MotorID] & 0b00000100)  # Bit 2 --> Overheating Error. Set to 1 if the internal temperature of the Dynamixel unit is above the operating temperature range as defined in the control table.
                # Bit 1 --> 0
                self.ErrorFlag_InputVoltage_Received[MotorID] = (self.ErrorFlag_BYTE[MotorID] & 0b00000001)  # Bit 0 --> Input Voltage Error. Set to 1 if the voltage is out of the operating voltage range as defined in the control table.

            else:
                Data_Value = dxl_data
            ##########################################################

            ##########################################################
            if print_bytes_for_debugging == 1:
                print("ReadVariable, " + VariableNameString + " on PortRegister " + str(PortRegister) + ": " + str(Data_Value))
            ##########################################################

            #self.portHandler.clearPort()
            #print("ReadVariable for variable " + str(VariableNameString) + ": "+ str(Data_Value))
            return Data_Value

        except:
            exceptions = sys.exc_info()[0]
            print("ReadVariable, exceptions: %s" % exceptions)
            traceback.print_exc()
    #######################################################################################################################
    #######################################################################################################################

    #######################################################################################################################
    #######################################################################################################################
    @staticmethod
    def ConvertBetweenAllAngularUnits(commanded_value, commanded_units):

        commanded_units = commanded_units.lower()

        commanded_value_raw = -1111111111
        commanded_value_dynamixelunits = -1111111111
        commanded_value_rad = -1111111111
        commanded_value_deg = -1111111111
        commanded_value_rev = -1111111111

        ######################################
        if commanded_units in angular_units_acceptable_list:
            if commanded_units == "raw" or commanded_units == "dynamixelunits":

                commanded_value_raw = commanded_value
                commanded_value_dynamixelunits = commanded_value

                commanded_value_deg = commanded_value_dynamixelunits * (1.0/4096.0) #0.088deg/dynamixelunit (1/4096.0)
                commanded_value_rev = commanded_value_deg / 360.0
                commanded_value_rad = commanded_value_rev * (2.0 * math.pi)

            elif commanded_units == "rad":

                commanded_value_rad = commanded_value

                commanded_value_rev = commanded_value_rad / (2.0 * math.pi)
                commanded_value_deg = 360.0 * commanded_value_rev
                commanded_value_dynamixelunits = commanded_value_deg / (1.0/4096.0) #0.088deg/dynamixelunit (1/4096.0)
                commanded_value_raw = commanded_value_dynamixelunits

            elif commanded_units == "deg":

                commanded_value_deg = commanded_value

                commanded_value_dynamixelunits = commanded_value_deg / 0.088 #0.088deg/dynamixelunit
                commanded_value_raw = commanded_value_dynamixelunits
                commanded_value_rev = commanded_value_deg / 360.0
                commanded_value_rad = commanded_value_rev * (2.0 * math.pi)

            elif commanded_units == "rev":

                commanded_value_rev = commanded_value

                commanded_value_deg = 360.0 * commanded_value_rev
                commanded_value_dynamixelunits = commanded_value_deg / (1.0/4096.0) #0.088deg/dynamixelunit
                commanded_value_raw = commanded_value_dynamixelunits
                commanded_value_rad = commanded_value_rev * (2.0 * math.pi)
            ######################################

            results_dict = dict([("raw", commanded_value_raw),
                                ("dynamixelunits", commanded_value_dynamixelunits),
                                ("deg", commanded_value_deg),
                                ("rad", commanded_value_rad),
                                ("rev", commanded_value_rev)])

            return results_dict

        else:
            print("DynamixelProtocol2Xseries_ReubenPython3Class.ConvertBetweenAllAngularUnits ERROR: The units '" + commanded_units + "' are not compatible but are limited to " + str(angular_units_acceptable_list))
            return dict()
    #######################################################################################################################
    #######################################################################################################################

    #######################################################################################################################
    #######################################################################################################################
    @staticmethod
    def ConvertBetweenAllAngularSpeedUnits(commanded_value, commanded_units):

        commanded_units = commanded_units.lower()

        commanded_value_raw = -1111111111
        commanded_value_dynamixelunits = -1111111111
        commanded_value_deg_per_sec = -1111111111
        commanded_value_rev_per_sec = -1111111111
        commanded_value_rev_per_min = -1111111111
        commanded_value_rad_per_sec = -1111111111

        ######################################
        if commanded_units in angular_speed_units_acceptable_list:
            if commanded_units == "raw" or commanded_units == "dynamixelunits":

                commanded_value_raw = commanded_value
                commanded_value_dynamixelunits = commanded_value

                commanded_value_rev_per_min = commanded_value_dynamixelunits * 0.229 #0.229RPM/dynamixelunit of speed
                commanded_value_rev_per_sec = commanded_value_rev_per_min / 60.0
                commanded_value_deg_per_sec = commanded_value_rev_per_sec * 360.0
                commanded_value_rad_per_sec = commanded_value_rev_per_sec * (2.0 * math.pi)

            elif commanded_units == "radpersec":

                commanded_value_rad_per_sec = commanded_value

                commanded_value_rev_per_sec = commanded_value_rad_per_sec / (2.0 * math.pi)
                commanded_value_rev_per_min = commanded_value_rad_per_sec * 60.0
                commanded_value_deg_per_sec = 360.0 * commanded_value_rev_per_sec
                commanded_value_dynamixelunits = commanded_value_rev_per_min / 0.229 #0.229RPM/dynamixelunit of speed
                commanded_value_raw = commanded_value_dynamixelunits

            elif commanded_units == "degpersec":

                commanded_value_deg_per_sec = commanded_value

                commanded_value_rev_per_sec = commanded_value_deg_per_sec / 360.0
                commanded_value_rev_per_min = commanded_value_rad_per_sec * 60.0
                commanded_value_rad_per_sec = commanded_value_rev_per_sec * (2.0 * math.pi)
                commanded_value_dynamixelunits = commanded_value_rev_per_min / 0.229 #0.229RPM/dynamixelunit of speed
                commanded_value_raw = commanded_value_dynamixelunits

            elif commanded_units == "revpersec":

                commanded_value_rev_per_sec = commanded_value

                commanded_value_rev_per_min = commanded_value_rad_per_sec * 60.0
                commanded_value_deg_per_sec = 360.0 * commanded_value_rev_per_sec
                commanded_value_rad_per_sec = commanded_value_rev_per_sec * (2.0 * math.pi)
                commanded_value_dynamixelunits = commanded_value_rev_per_min / 0.229 #0.229RPM/dynamixelunit of speed
                commanded_value_raw = commanded_value_dynamixelunits

            elif commanded_units == "revpermin":

                commanded_value_rev_per_min = commanded_value

                commanded_value_rev_per_sec = commanded_value / 60.0
                commanded_value_deg_per_sec = 360.0 * commanded_value_rev_per_sec
                commanded_value_rad_per_sec = commanded_value_rev_per_sec * (2.0 * math.pi)
                commanded_value_dynamixelunits = commanded_value_rev_per_min / 0.229 #0.229RPM/dynamixelunit of speed
                commanded_value_raw = commanded_value_dynamixelunits
            ######################################

            results_dict = dict([("raw", commanded_value_raw),
                                        ("dynamixelunits", commanded_value_dynamixelunits),
                                        ("degPerSec", commanded_value_deg_per_sec),
                                        ("radPerSec", commanded_value_rad_per_sec),
                                        ("revPerSec", commanded_value_rev_per_sec),
                                        ("revPerMin", commanded_value_rev_per_min)])

            return results_dict

        else:
            print("DynamixelProtocol2Xseries_ReubenPython3Class.ConvertBetweenAllAngularSpeedUnits ERROR: The units '" + commanded_units + "' are not compatible but are limited to " + str(angular_speed_units_acceptable_list))
            return dict()
    #######################################################################################################################
    #######################################################################################################################

    #######################################################################################################################
    #######################################################################################################################
    @staticmethod
    def ConvertBetweenAllCurrentUnits(commanded_value, commanded_units):

        commanded_units = commanded_units.lower()

        commanded_value_raw = -1111111111
        commanded_value_dynamixelunits = -1111111111
        commanded_value_milliamps = -1111111111
        commanded_value_amps = -1111111111
        commanded_value_percent = -1111111111

        ######################################
        if commanded_units in current_units_acceptable_list:
            if commanded_units == "raw" or commanded_units == "dynamixelunits":

                commanded_value_raw = commanded_value
                commanded_value_dynamixelunits = commanded_value

                commanded_value_milliamps = commanded_value_dynamixelunits * 2.69 #2.69mA/dynamixelunit
                commanded_value_amps = commanded_value_milliamps / 1000.0
                commanded_value_percent = commanded_value_dynamixelunits / 2047.0

            elif commanded_units == "milliamps":

                commanded_value_milliamps = commanded_value

                commanded_value_amps = commanded_value_milliamps / 1000.0
                commanded_value_dynamixelunits = commanded_value_milliamps / 2.69 #2.69mA/dynamixelunit
                commanded_value_raw = commanded_value_dynamixelunits
                commanded_value_percent = commanded_value_dynamixelunits / 2047.0

            elif commanded_units == "amps":

                commanded_value_amps = commanded_value

                commanded_value_milliamps = commanded_value_amps * 1000.0
                commanded_value_dynamixelunits = commanded_value_milliamps / 2.69 #2.69mA/dynamixelunit
                commanded_value_raw = commanded_value_dynamixelunits
                commanded_value_percent = commanded_value_dynamixelunits / 2047.0

            elif commanded_units == "percent":

                if commanded_value < -1.0 or commanded_value > 1.0:
                    print("ConvertBetweenAllCurrentUnits ERROR: The value '" + str(commanded_value) + "% must be in the range [0,1]")
                    return dict()

                commanded_value_percent = commanded_value

                commanded_value_dynamixelunits = commanded_value_percent*2047.0
                commanded_value_raw = commanded_value_dynamixelunits
                commanded_value_milliamps = commanded_value_dynamixelunits * 2.69 #2.69mA/dynamixelunit
                commanded_value_amps = commanded_value_milliamps / 1000.0
            ######################################

            results_dict = dict([("raw", commanded_value_raw),
                                        ("dynamixelunits", commanded_value_dynamixelunits),
                                        ("milliamps", commanded_value_milliamps),
                                        ("amps", commanded_value_amps),
                                        ("percent", commanded_value_percent)])

            return results_dict

        else:
            print("ConvertBetweenAllCurrentUnits ERROR: The units '" + commanded_units + "' are not compatible but are limited to " + str(current_units_acceptable_list))
            return dict()
    #######################################################################################################################
    #######################################################################################################################

    #######################################################################################################################
    #######################################################################################################################
    def InitializeMotor(self, MotorIndex):

        self.SendInstructionPacket_SetCurrentLimit(MotorIndex, self.Current_DynamixelUnits_max[MotorIndex], 0)
        time.sleep(self.TimeToWaitBetweenCriticalInstructions)
        self.SendInstructionPacket_SetMaxVelocity(MotorIndex, self.Velocity_DynamixelUnits_Max_UserSet[MotorIndex], 0)
        time.sleep(self.TimeToWaitBetweenCriticalInstructions)
        self.SendInstructionPacket_SetProfileVelocity_ForAllModesExceptCurrentAndVelocity(MotorIndex, self.Velocity_DynamixelUnits_Max_UserSet[MotorIndex], 0)
        time.sleep(self.TimeToWaitBetweenCriticalInstructions)
        self.SendInstructionPacket_SetMaxPWM(MotorIndex, self.PWM_DynamixelUnits_max[MotorIndex], 0)
        time.sleep(self.TimeToWaitBetweenCriticalInstructions)
        self.SendInstructionPacket_ReturnDelayTime(MotorIndex, 2)
        time.sleep(self.TimeToWaitBetweenCriticalInstructions)
        self.SendInstructionPacket_SetStatusReturnLevel(MotorIndex, 1) #0: Do not respond to any instructions, 1: Respond only to READ_DATA instructions, 2: Respond to all instructions
        time.sleep(self.TimeToWaitBetweenCriticalInstructions)
        self.SendInstructionPacket_SetShutdown(MotorIndex, self.EnableSafetyShutoff[MotorIndex])
        time.sleep(self.TimeToWaitBetweenCriticalInstructions)

        if self.HasMotorEverBeenInitializedFlag[MotorIndex] == 0:
            if self.StartEngagedFlag[MotorIndex] == 1:
                self.SetEngagedState_FROM_EXTERNAL_PROGRAM(MotorIndex, 1)
            else:
                self.SetEngagedState_FROM_EXTERNAL_PROGRAM(MotorIndex, 0)
        else:
            self.SetEngagedState_FROM_EXTERNAL_PROGRAM(MotorIndex, 1)

        #self.ResetWatchdogTimerInMilliseconds(MotorIndex)
        #time.sleep(self.TimeToWaitBetweenCriticalInstructions)
        #self.SetWatchdogTimerInMilliseconds(MotorIndex,  self.WatchdogTimeIntervalMilliseconds)
        #time.sleep(self.TimeToWaitBetweenCriticalInstructions)

        #'''
        ################################################
        NumberOfQueries = 2
        for i in range(0, NumberOfQueries):
            self.OperatingModeReceived_int[MotorIndex] = self.ReadVariable(MotorIndex, "OperatingMode", 0)
            self.OperatingModeReceived_string[MotorIndex] = self.ConvertOperatingModeIntToString(self.OperatingModeReceived_int[MotorIndex])
            print("InitializeMotor, " + str(self.OperatingModeReceived_int[MotorIndex]) + " | " + self.OperatingModeReceived_string[MotorIndex] + " | " + self.OperatingMode_StartingValueList[MotorIndex])

            if self.OperatingModeReceived_string[MotorIndex] != self.OperatingMode_StartingValueList[MotorIndex]:
                print("InitializeMotor, ERROR: self.OperatingModeReceived_string = " + str(self.OperatingModeReceived_string) +
                      " does not match self.OperatingMode_StartingValueList = " + str(self.OperatingMode_StartingValueList))

            time.sleep(self.TimeToWaitBetweenCriticalInstructions)



        #if self.OperatingModeReceived_string[MotorIndex] != self.OperatingMode_StartingValueList[MotorIndex]:
        #    self.OperatingMode_NEEDS_TO_BE_CHANGED_FLAG[MotorIndex] = 1
        ################################################
        #'''

        ################################################ Will tell the main loop how to update the motor
        if self.OperatingMode[MotorIndex] == "CurrentControl":
            self.Current_DynamixelUnits_NeedsToBeChangedFlag[MotorIndex] = 1

        if self.OperatingMode[MotorIndex] == "VelocityControl":
            self.Velocity_DynamixelUnits_NeedsToBeChangedFlag[MotorIndex] = 1

        if self.OperatingMode[MotorIndex] == "PositionControl" or self.OperatingMode[MotorIndex] == "ExtendedPositionControlMultiTurn" or self.OperatingMode[MotorIndex] == "CurrentBasedPositionControl":
            self.Position_DynamixelUnits_NeedsToBeChangedFlag[MotorIndex] = 1
            self.Velocity_DynamixelUnits_NeedsToBeChangedFlag[MotorIndex] = 1

        if self.OperatingMode[MotorIndex] == "PWMcontrol":
            self.PWM_DynamixelUnits_NeedsToBeChangedFlag[MotorIndex] = 1
        ################################################

        self.HasMotorEverBeenInitializedFlag[MotorIndex] = 1

    #######################################################################################################################
    #######################################################################################################################

    #######################################################################################################################
    #######################################################################################################################
    def InitializeAllMotors(self):

        for MotorIndex in range(0, self.NumberOfMotors):
            self.InitializeMotor(MotorIndex)

    #######################################################################################################################
    #######################################################################################################################

    #######################################################################################################################
    def MainThread(self): #unicorn

        self.MyPrint_WithoutLogFile("Started the MainThread thread for DynamixelProtocol2Xseries_ReubenPython3Class object.")

        self.InitializeAllMotors()

        self.StartingTime_CalculatedFromMainThread = self.getPreciseSecondsTimeStampString()

        #############################################################################################################################################
        #############################################################################################################################################
        #############################################################################################################################################
        while self.EXIT_PROGRAM_FLAG == 0:

            ##############################################################################################  Start GET's
            ##############################################################################################
            ##############################################################################################
            ##############################################################################################
            if self.GetVariablesEveryNloopsCycles >= 0:
                try:

                    ##############################################################################################
                    ##############################################################################################
                    ##############################################################################################
                    if self.AskForInfrequentDataReadLoopCounter == 0:

                        self.AskForInfrequentDataReadLoopCounter = self.AskForInfrequentDataReadLoopCounter + 1

                        ##############################################################################################
                        ##############################################################################################
                        self.CurrentTime_Rx_CalculatedFromMainThread = self.getPreciseSecondsTimeStampString() - self.StartingTime_CalculatedFromMainThread
                        ##############################################################################################
                        ##############################################################################################

                        ##############################################################################################
                        ##############################################################################################
                        for MotorIndex in range(0, self.NumberOfMotors):

                            for VariableNameString in self.ListOfVariableNameStringsToGet:
                                Value = self.ReadVariable(MotorIndex, VariableNameString, print_bytes_for_debugging=0)
                                #print("Value: " + str(Value))
                                try:
                                    self.MostRecentDataDict[VariableNameString][MotorIndex] = float(Value)
                                except:
                                    self.MostRecentDataDict[VariableNameString][MotorIndex] = str(Value)
                        ##############################################################################################
                        ##############################################################################################

                    else:

                        self.AskForInfrequentDataReadLoopCounter  = self.AskForInfrequentDataReadLoopCounter  + 1

                        if self.AskForInfrequentDataReadLoopCounter >= self.GetVariablesEveryNloopsCycles:
                            self.AskForInfrequentDataReadLoopCounter = 0

                    ##############################################################################################
                    ##############################################################################################
                    ##############################################################################################

                    ##############################################################################################
                    ##############################################################################################
                    ##############################################################################################
                    try:

                        if "OperatingModeReceived_string" in self.MostRecentDataDict:

                            for MotorIndex in range(0, self.NumberOfMotors):
                                ##############################################################################################
                                ##############################################################################################
                                if self.MostRecentDataDict["OperatingModeReceived_string"][MotorIndex] != self.OperatingMode_StartingValueList[MotorIndex]:
                                    self.ErrorFlag_OperatingModeMismatch[MotorIndex] = 1
                                else:
                                    self.ErrorFlag_OperatingModeMismatch[MotorIndex] = 0
                                ##############################################################################################
                                ##############################################################################################

                        ##############################################################################################
                        ##############################################################################################

                    except:
                        exceptions = sys.exc_info()[0]
                        print("if PresentCurrent in self.MostRecentDataDict:, exceptions: %s" % exceptions)
                        traceback.print_exc()

                    ##############################################################################################
                    ##############################################################################################
                    ##############################################################################################

                    ##############################################################################################
                    ##############################################################################################
                    ##############################################################################################
                    try:

                        if "PresentCurrent" in self.MostRecentDataDict:

                            for MotorIndex in range(0, self.NumberOfMotors):
                                ##############################################################################################
                                ##############################################################################################

                                if isinstance(self.MostRecentDataDict["PresentCurrent"][MotorIndex], str) == 0:
                                    Current_AbsVal = abs(self.MostRecentDataDict["PresentCurrent"][MotorIndex]) #Our Polyfit curve only works for positive values of current

                                    Current_MinValueInputToPolyfit = self.MotorTorqueConstant_ListOfDicts[MotorIndex]["Current_MinValueInputToPolyfit"]
                                    Current_MaxValueInputToPolyfit = self.MotorTorqueConstant_ListOfDicts[MotorIndex]["Current_MaxValueInputToPolyfit"]

                                    if Current_AbsVal >= Current_MinValueInputToPolyfit and Current_AbsVal <= Current_MaxValueInputToPolyfit:


                                        ##############################################################################################
                                        if self.MostRecentDataDict["PresentCurrent"][MotorIndex] >= 0:
                                            Current_Sign = 1
                                        else:
                                            Current_Sign = -1
                                        ##############################################################################################

                                        Kt_Polynomial = self.MotorTorqueConstant_ListOfDicts[MotorIndex]["PolynomialFitMotorCurrentToMotorTorque"]
                                        N = len(Kt_Polynomial)
                                        Torque = 0.0

                                        ##############################################################################################
                                        for Index in range(0, len(Kt_Polynomial)):
                                            #print("MotorIndex: " + str(MotorIndex) + ", PolyIndex: " + str(Index) + ", Kt_term: " + str(Kt_Polynomial[Index]) + ", Current_AbsVal: " + str(Current_AbsVal) + ", Torque: " + str(Torque))
                                            #Torque = P[0]*(X**2) + P[1]*(X**1) + P[2]*(X**0)

                                            Torque = Torque + Kt_Polynomial[Index]*(Current_AbsVal**(N-1-Index))
                                        ##############################################################################################

                                        Torque = Current_Sign*Torque

                                        Torque = 0.0*Torque #Torque directionality appears to be reversed, not sure why as the current directionality is correct.
                                        ##############################################################################################

                                        ##############################################################################################
                                        if "TorqueNMcalculatedFromMotorCurrent" not in self.MostRecentDataDict:
                                            self.MostRecentDataDict["TorqueNMcalculatedFromMotorCurrent"] = [0.0]*self.NumberOfMotors
                                        else:
                                            self.MostRecentDataDict["TorqueNMcalculatedFromMotorCurrent"][MotorIndex] =  Torque
                                        ##############################################################################################

                                ##############################################################################################
                                ##############################################################################################

                        else:
                            self.MostRecentDataDict["TorqueNMcalculatedFromMotorCurrent"][MotorIndex] = -1.0
                        ##############################################################################################
                        ##############################################################################################

                    except:
                        exceptions = sys.exc_info()[0]
                        print("if PresentCurrent in self.MostRecentDataDict:, exceptions: %s" % exceptions)
                        traceback.print_exc()

                    ##############################################################################################
                    ##############################################################################################
                    ##############################################################################################

                    self.UpdateFrequencyCalculation_Rx_CalculatedFromMainThread_Filtered()

                except:
                    exceptions = sys.exc_info()[0]
                    print("MainThread GETS, exceptions: %s" % exceptions)
                    #traceback.print_exc()

            ##############################################################################################
            ##############################################################################################
            ##############################################################################################
            ############################################################################################## End GET's

            ############################################################################################## StartSET's
            ##############################################################################################
            ##############################################################################################
            ##############################################################################################
            if self.ENABLE_SETS == 1:
                try:

                    ##############################################################################################
                    ##############################################################################################
                    ##############################################################################################
                    self.CurrentTime_Tx_CalculatedFromMainThread = self.getPreciseSecondsTimeStampString() - self.StartingTime_CalculatedFromMainThread
                    ##############################################################################################
                    ##############################################################################################
                    ##############################################################################################

                    for MotorIndex in range(0, self.NumberOfMotors):

                        ##############################################################################################
                        ##############################################################################################
                        if self.Reboot_NeedsToTakePlaceFlag[MotorIndex] == 1:
                            self.SendInstructionPacket_Reboot(MotorIndex)
                            time.sleep(0.25) #FIGURE OUT HOW LOW THIS CAN GO

                            self.InitializeMotor(MotorIndex)

                            self.ResetSerial()

                            self.Reboot_NeedsToTakePlaceFlag[MotorIndex] = 0
                        ##############################################################################################
                        ##############################################################################################

                        ##############################################################################################
                        ##############################################################################################
                        if self.ResetSerial_NeedsToTakePlaceFlag[MotorIndex] == 1:
                            self.ResetSerial()
                            self.ResetSerial_NeedsToTakePlaceFlag[MotorIndex] = 0
                        ##############################################################################################
                        ##############################################################################################

                        ##############################################################################################
                        ##############################################################################################
                        if self.OperatingMode_NEEDS_TO_BE_CHANGED_FLAG[MotorIndex] == 1:
                            self.SendInstructionPacket_SetOperatingMode(MotorIndex, self.OperatingMode_TO_BE_SET[MotorIndex])
                            self.OperatingMode_NEEDS_TO_BE_ASKED_FLAG[MotorIndex] = 1

                            time.sleep(0.030)

                            self.SendInstructionPacket_SetCurrentLimit(MotorIndex, self.Current_DynamixelUnits_max[MotorIndex], 0)

                            self.OperatingMode_NEEDS_TO_BE_CHANGED_FLAG[MotorIndex] = 0
                        ##############################################################################################
                        ##############################################################################################

                        ##############################################################################################
                        ##############################################################################################
                        if self.ToggleMinMax_NeedsToTakePlaceFlag[MotorIndex] == 1:

                            self.MyPrint_WithoutLogFile("Position_DynamixelUnits_Min_UserSet valid: " + str(self.Position_DynamixelUnits_Min_UserSet))
                            self.MyPrint_WithoutLogFile("Position_DynamixelUnits_StartingValueList valid: " + str(self.Position_DynamixelUnits_StartingValueList))
                            self.MyPrint_WithoutLogFile("Position_DynamixelUnits_Max_UserSet valid: " + str(self.Position_DynamixelUnits_Max_UserSet))

                            if self.ToggleMinMax_TO_BE_SET[MotorIndex] == -1:

                                if self.OperatingMode[MotorIndex] == "CurrentControl":
                                    self.Current_DynamixelUnits_TO_BE_SET[MotorIndex] = self.Current_DynamixelUnits_min[MotorIndex]
                                    self.Current_DynamixelUnits_GUI_NeedsToBeChangedFlag[MotorIndex] = 1
                                    self.Current_DynamixelUnits_NeedsToBeChangedFlag[MotorIndex] = 1

                                elif self.OperatingMode[MotorIndex] == "VelocityControl":
                                    self.Velocity_DynamixelUnits_TO_BE_SET[MotorIndex] = self.Velocity_DynamixelUnits_Min_UserSet[MotorIndex]
                                    self.Velocity_DynamixelUnits_GUI_NeedsToBeChangedFlag[MotorIndex] = 1
                                    self.Velocity_DynamixelUnits_NeedsToBeChangedFlag[MotorIndex] = 1

                                elif self.OperatingMode[MotorIndex] == "PositionControl" or self.OperatingMode[MotorIndex] == "ExtendedPositionControlMultiTurn" or self.OperatingMode[MotorIndex] == "CurrentBasedPositionControl":
                                    self.Position_DynamixelUnits_TO_BE_SET[MotorIndex] = self.Position_DynamixelUnits_Min_UserSet[MotorIndex]
                                    self.Position_DynamixelUnits_GUI_NeedsToBeChangedFlag[MotorIndex] = 1
                                    self.Position_DynamixelUnits_NeedsToBeChangedFlag[MotorIndex] = 1

                                elif self.OperatingMode[MotorIndex] == "PWMcontrol":
                                    self.PWM_DynamixelUnits_TO_BE_SET[MotorIndex] = self.PWM_DynamixelUnits_min[MotorIndex]
                                    self.PWM_DynamixelUnits_GUI_NeedsToBeChangedFlag[MotorIndex] = 1
                                    self.PWM_DynamixelUnits_NeedsToBeChangedFlag[MotorIndex] = 1

                            elif self.ToggleMinMax_TO_BE_SET[MotorIndex] == 0:

                                if self.OperatingMode[MotorIndex] == "CurrentControl":
                                    self.Current_DynamixelUnits_TO_BE_SET[MotorIndex] = self.Current_DynamixelUnits_StartingValueList[MotorIndex]
                                    self.Current_DynamixelUnits_GUI_NeedsToBeChangedFlag[MotorIndex] = 1
                                    self.Current_DynamixelUnits_NeedsToBeChangedFlag[MotorIndex] = 1

                                elif self.OperatingMode[MotorIndex] == "VelocityControl":
                                    self.Velocity_DynamixelUnits_TO_BE_SET[MotorIndex] = self.Velocity_DynamixelUnits_StartingValueList[MotorIndex]
                                    self.Velocity_DynamixelUnits_GUI_NeedsToBeChangedFlag[MotorIndex] = 1
                                    self.Velocity_DynamixelUnits_NeedsToBeChangedFlag[MotorIndex] = 1

                                elif self.OperatingMode[MotorIndex] == "PositionControl" or self.OperatingMode[MotorIndex] == "ExtendedPositionControlMultiTurn" or self.OperatingMode[MotorIndex] == "CurrentBasedPositionControl":
                                    self.Position_DynamixelUnits_TO_BE_SET[MotorIndex] = self.Position_DynamixelUnits_StartingValueList[MotorIndex]
                                    print(self.Position_DynamixelUnits_TO_BE_SET[MotorIndex])
                                    self.Position_DynamixelUnits_GUI_NeedsToBeChangedFlag[MotorIndex] = 1
                                    self.Position_DynamixelUnits_NeedsToBeChangedFlag[MotorIndex] = 1

                                elif self.OperatingMode[MotorIndex] == "PWMcontrol":
                                    self.PWM_DynamixelUnits_TO_BE_SET[MotorIndex] = self.PWM_DynamixelUnits_StartingValueList[MotorIndex]
                                    self.PWM_DynamixelUnits_GUI_NeedsToBeChangedFlag[MotorIndex] = 1
                                    self.PWM_DynamixelUnits_NeedsToBeChangedFlag[MotorIndex] = 1

                            elif self.ToggleMinMax_TO_BE_SET[MotorIndex] == 1:

                                if self.OperatingMode[MotorIndex] == "CurrentControl":
                                    self.Current_DynamixelUnits_TO_BE_SET[MotorIndex] = self.Current_DynamixelUnits_max[MotorIndex]
                                    self.Current_DynamixelUnits_GUI_NeedsToBeChangedFlag[MotorIndex] = 1
                                    self.Current_DynamixelUnits_NeedsToBeChangedFlag[MotorIndex] = 1

                                elif self.OperatingMode[MotorIndex] == "VelocityControl":
                                    self.Velocity_DynamixelUnits_TO_BE_SET[MotorIndex] = self.Velocity_DynamixelUnits_Max_UserSet[MotorIndex]
                                    self.Velocity_DynamixelUnits_GUI_NeedsToBeChangedFlag[MotorIndex] = 1
                                    self.Velocity_DynamixelUnits_NeedsToBeChangedFlag[MotorIndex] = 1

                                elif self.OperatingMode[MotorIndex] == "PositionControl" or self.OperatingMode[MotorIndex] == "ExtendedPositionControlMultiTurn" or self.OperatingMode[MotorIndex] == "CurrentBasedPositionControl":
                                    self.Position_DynamixelUnits_TO_BE_SET[MotorIndex] = self.Position_DynamixelUnits_Max_UserSet[MotorIndex]
                                    self.Position_DynamixelUnits_GUI_NeedsToBeChangedFlag[MotorIndex] = 1
                                    self.Position_DynamixelUnits_NeedsToBeChangedFlag[MotorIndex] = 1

                                elif self.OperatingMode[MotorIndex] == "PWMcontrol":
                                    self.PWM_DynamixelUnits_TO_BE_SET[MotorIndex] = self.PWM_DynamixelUnits_max[MotorIndex]
                                    self.PWM_DynamixelUnits_GUI_NeedsToBeChangedFlag[MotorIndex] = 1
                                    self.PWM_DynamixelUnits_NeedsToBeChangedFlag[MotorIndex] = 1

                            self.ToggleMinMax_state[MotorIndex] = self.ToggleMinMax_TO_BE_SET[MotorIndex]
                            self.ToggleMinMax_NeedsToTakePlaceFlag[MotorIndex] = 0
                        ##############################################################################################
                        ##############################################################################################

                        ##############################################################################################
                        ##############################################################################################
                        if self.EngagedState_NeedsToBeChangedFlag[MotorIndex] == 1:
                            self.SendInstructionPacket_SetTorqueEnable(MotorIndex, self.EngagedState_TO_BE_SET[MotorIndex])

                            if self.EngagedState_TO_BE_SET[MotorIndex] == 1:
                                self.Position_DynamixelUnits_NeedsToBeChangedFlag[MotorIndex] = 1 #To make sure that the motor spins to where it's supposed to go upon engagement
                                self.Position_DynamixelUnits_GUI_NeedsToBeChangedFlag[MotorIndex] = 1 #To make sure that the motor spins to where it's supposed to go upon engagement

                            self.EngagedState_NeedsToBeChangedFlag[MotorIndex] = 0
                        ##############################################################################################
                        ##############################################################################################

                        ##############################################################################################
                        ##############################################################################################
                        if self.LEDstate_NeedsToBeChangedFlag[MotorIndex] == 1:
                            self.SendInstructionPacket_SetLED(MotorIndex, self.LEDstate_TO_BE_SET[MotorIndex])
                            self.LEDstate_NeedsToBeChangedFlag[MotorIndex] = 0
                        ##############################################################################################
                        ##############################################################################################

                        ##############################################################################################
                        ##############################################################################################
                        if self.MaxPWM_DynamixelUnits_NeedsToBeChangedFlag[MotorIndex] == 1:
                            self.SendInstructionPacket_SetMaxPWM(MotorIndex, self.MaxPWM_DynamixelUnits_TO_BE_SET[MotorIndex])
                            self.MaxPWM_DynamixelUnits_NeedsToBeChangedFlag[MotorIndex] = 0
                        ##############################################################################################
                        ##############################################################################################

                        ##############################################################################################
                        ##############################################################################################
                        if self.Current_DynamixelUnits_NeedsToBeChangedFlag[MotorIndex] > 0:

                            if self.OperatingMode[MotorIndex] == "CurrentControl":
                                Current_limited = self.LimitNumber_IntOutputOnly(self.Current_DynamixelUnits_min[MotorIndex], self.Current_DynamixelUnits_max[MotorIndex], self.Current_DynamixelUnits_TO_BE_SET[MotorIndex])
                                self.SendInstructionPacket_SetGoalCurrent_ForCurrentAndCurrentBasedPositionControl(MotorIndex, Current_limited, 0)

                            else:
                                Current_limited = self.LimitNumber_IntOutputOnly(0.0, self.Current_DynamixelUnits_max[MotorIndex], self.Current_DynamixelUnits_TO_BE_SET[MotorIndex])
                                self.SendInstructionPacket_SetCurrentLimit(MotorIndex, Current_limited, 0)

                                # MUST ALSO SET GOAL CURRENT FOR CURRENT LIMIT TO WORK
                                Current_limited = self.LimitNumber_IntOutputOnly(self.Current_DynamixelUnits_min[MotorIndex], self.Current_DynamixelUnits_max[MotorIndex], self.Current_DynamixelUnits_TO_BE_SET[MotorIndex])
                                self.SendInstructionPacket_SetGoalCurrent_ForCurrentAndCurrentBasedPositionControl(MotorIndex, Current_limited, 0)
                                #

                            if self.Current_DynamixelUnits_NeedsToBeChangedFlag[MotorIndex] >= 1:
                                self.Current_DynamixelUnits_NeedsToBeChangedFlag[MotorIndex] = self.Current_DynamixelUnits_NeedsToBeChangedFlag[MotorIndex] - 1
                                #self.MyPrint_WithoutLogFile("self.Current_DynamixelUnits_NeedsToBeChangedFlag[" + str(MotorIndex) + "]: " + str(self.Current_DynamixelUnits_NeedsToBeChangedFlag[MotorIndex]))

                        ##############################################################################################
                        ##############################################################################################

                        ##############################################################################################
                        ##############################################################################################
                        if self.OperatingMode[MotorIndex] == "VelocityControl":
                            if self.Velocity_DynamixelUnits_NeedsToBeChangedFlag[MotorIndex] == 1:
                                Velocity_limited = self.LimitNumber_IntOutputOnly(self.Velocity_DynamixelUnits_Min_UserSet[MotorIndex], self.Velocity_DynamixelUnits_Max_UserSet[MotorIndex], self.Velocity_DynamixelUnits_TO_BE_SET[MotorIndex])
                                self.SendInstructionPacket_SetGoalVelocity_ForVelocityModeOnly(MotorIndex, Velocity_limited, 0)
                                self.Velocity_DynamixelUnits_NeedsToBeChangedFlag[MotorIndex] = 0
                        ##############################################################################################
                        ##############################################################################################

                        ##############################################################################################
                        ##############################################################################################
                        if self.OperatingMode[MotorIndex] == "PositionControl" or self.OperatingMode[MotorIndex] == "ExtendedPositionControlMultiTurn" or self.OperatingMode[MotorIndex] == "CurrentBasedPositionControl":

                            ##############################################################################################
                            if self.Position_DynamixelUnits_NeedsToBeChangedFlag[MotorIndex]:
                                Position_limited = self.LimitNumber_IntOutputOnly(self.Position_DynamixelUnits_Min_UserSet[MotorIndex], self.Position_DynamixelUnits_Max_UserSet[MotorIndex], self.Position_DynamixelUnits_TO_BE_SET[MotorIndex])
                                self.SendInstructionPacket_SetPosition(MotorIndex, Position_limited, print_bytes_for_debugging=0)
                                self.Position_DynamixelUnits_NeedsToBeChangedFlag[MotorIndex] = 0
                            ##############################################################################################

                            ##############################################################################################
                            if self.Velocity_DynamixelUnits_NeedsToBeChangedFlag[MotorIndex] == 1:
                                Velocity_limited = self.LimitNumber_IntOutputOnly(self.Velocity_DynamixelUnits_Min_UserSet[MotorIndex], self.Velocity_DynamixelUnits_Max_UserSet[MotorIndex], self.Velocity_DynamixelUnits_TO_BE_SET[MotorIndex])
                                self.SendInstructionPacket_SetProfileVelocity_ForAllModesExceptCurrentAndVelocity(MotorIndex, Velocity_limited, 0)

                                self.Position_DynamixelUnits_GUI_NeedsToBeChangedFlag[MotorIndex] = 1 #A velocity command cancels that last position command, so this makes sure we finish getting to the last goal if the velocity was updated before it arrived at that target position.
                                self.Position_DynamixelUnits_NeedsToBeChangedFlag[MotorIndex] = 1 #A velocity command cancels that last position command, so this makes sure we finish getting to the last goal if the velocity was updated before it arrived at that target position.

                                self.Velocity_DynamixelUnits_NeedsToBeChangedFlag[MotorIndex] = 0
                            ##############################################################################################

                        ##############################################################################################
                        ##############################################################################################

                        ##############################################################################################
                        ##############################################################################################
                        if self.OperatingMode[MotorIndex] == "PWMcontrol":
                            if self.PWM_DynamixelUnits_NeedsToBeChangedFlag[MotorIndex]:
                                PWM_limited = self.LimitNumber_IntOutputOnly(self.PWM_DynamixelUnits_min[MotorIndex], self.PWM_DynamixelUnits_max[MotorIndex], self.PWM_DynamixelUnits_TO_BE_SET[MotorIndex])
                                self.SendInstructionPacket_SetPWM(MotorIndex, PWM_limited, 0)
                                self.PWM_DynamixelUnits_NeedsToBeChangedFlag[MotorIndex] = 0
                        ##############################################################################################
                        ##############################################################################################

                except:
                    exceptions = sys.exc_info()[0]
                    print("MainThread SETS, exceptions: %s" % exceptions)
                    traceback.print_exc()
            ##############################################################################################
            ##############################################################################################
            ##############################################################################################
            ##############################################################################################  End SET's

            ##############################################################################################
            ##############################################################################################
            ##############################################################################################

            self.MostRecentDataDict["OperatingMode"] = self.OperatingMode

            self.MostRecentDataDict["Time"] = self.CurrentTime_Tx_CalculatedFromMainThread
            self.MostRecentDataDict["DataStreamingFrequency_Rx_CalculatedFromMainThread"] = self.DataStreamingFrequency_Rx_CalculatedFromMainThread
            self.MostRecentDataDict["DataStreamingFrequency_Tx_CalculatedFromMainThread"] = self.DataStreamingFrequency_Tx_CalculatedFromMainThread
            self.MostRecentDataDict["OperatingMode"] = self.OperatingMode
            self.MostRecentDataDict["Position_DynamixelUnits_TO_BE_SET"] = self.Position_DynamixelUnits_TO_BE_SET
            self.MostRecentDataDict["Velocity_DynamixelUnits_TO_BE_SET"] = self.Velocity_DynamixelUnits_TO_BE_SET
            self.MostRecentDataDict["Current_DynamixelUnits_TO_BE_SET"] = self.Current_DynamixelUnits_TO_BE_SET
            self.MostRecentDataDict["PWM_DynamixelUnits_TO_BE_SET"] = self.PWM_DynamixelUnits_TO_BE_SET

            self.MostRecentDataDict["SendInstructionPacket_SetPosition_Counter"] = self.SendInstructionPacket_SetPosition_Counter

            self.MostRecentDataDict["ErrorFlag_BYTE"] = self.ErrorFlag_BYTE
            self.MostRecentDataDict["ErrorFlag_Overload_Received"] = self.ErrorFlag_Overload_Received
            self.MostRecentDataDict["ErrorFlag_ElectricalShock_Received"] = self.ErrorFlag_ElectricalShock_Received
            self.MostRecentDataDict["ErrorFlag_MotorEncoder_Received"] = self.ErrorFlag_MotorEncoder_Received
            self.MostRecentDataDict["ErrorFlag_Overheating_Received"] = self.ErrorFlag_Overheating_Received
            self.MostRecentDataDict["ErrorFlag_InputVoltage_Received"] = self.ErrorFlag_InputVoltage_Received
            self.MostRecentDataDict["ErrorFlag_OperatingModeMismatch"] = self.ErrorFlag_OperatingModeMismatch

            self.MostRecentDataDict["OperatingModeReceived_int"] = self.OperatingModeReceived_int
            self.MostRecentDataDict["OperatingModeReceived_string"] = self.OperatingModeReceived_string

            self.MostRecentDataDict["EngagedState"] = self.EngagedState
            self.MostRecentDataDict["LEDstate"] = self.LEDstate
            #self.MostRecentDataDict["RealTimeTicksMillisec"] = self.RealTimeTicksMillisec
            #self.MostRecentDataDict["DataStreamingFrequency_RealTimeTicksMillisecFromDynamixel"] = self.DataStreamingFrequency_RealTimeTicksMillisecFromDynamixel
            #Watchdog

            ##############################################################################################
            ##############################################################################################
            if "PresentPosition" in self.MostRecentDataDict:
                for MotorIndex in range(0, self.NumberOfMotors):

                    ##############################################################################################
                    if "PresentPosition_Degrees" not in self.MostRecentDataDict:
                        self.MostRecentDataDict["PresentPosition_Degrees"] = [0.0]*self.NumberOfMotors

                    try:
                        self.MostRecentDataDict["PresentPosition_Degrees"][MotorIndex] = self.ConversionFactorFromDynamixelUnitsToDegrees[MotorIndex]*self.MostRecentDataDict["PresentPosition"][MotorIndex]
                    except:
                        pass
                    ##############################################################################################

                    ##############################################################################################
                    if self.MostRecentDataDict["PresentPosition"][MotorIndex] == "error_serial_dxl_error":
                        self.ErrorFlag_SerialCommunication[MotorIndex] = 1
                    else:
                        self.ErrorFlag_SerialCommunication[MotorIndex] = 0
                    ##############################################################################################


                self.MostRecentDataDict["ErrorFlag_SerialCommunication"] = self.ErrorFlag_SerialCommunication
            ##############################################################################################
            ##############################################################################################

            self.MostRecentDataDict_ForExternalQueryAndGUIdisplayOnly_BlockExternalCopyingFlag = 1

            self.MostRecentDataDict_ForExternalQueryAndGUIdisplayOnly = deepcopy(self.MostRecentDataDict)

            self.MostRecentDataDict_ForExternalQueryAndGUIdisplayOnly_BlockExternalCopyingFlag = 0
            ##############################################################################################
            ##############################################################################################
            ##############################################################################################

            ##############################################################################################
            ##############################################################################################
            ##############################################################################################
            self.UpdateFrequencyCalculation_Tx_CalculatedFromMainThread_Filtered()
            self.MostRecentDataDict["DataStreamingFrequency_Tx_CalculatedFromMainThread"] = self.DataStreamingFrequency_Tx_CalculatedFromMainThread

            if self.MainThread_TimeToSleepEachLoop > 0.0:
                if self.MainThread_TimeToSleepEachLoop > 0.001:
                    time.sleep(self.MainThread_TimeToSleepEachLoop - 0.001) #The "- 0.001" corrects for slight deviation from intended frequency due to other functions being called.
                else:
                    time.sleep(self.MainThread_TimeToSleepEachLoop)
            ##############################################################################################
            ##############################################################################################
            ##############################################################################################

            #############################################################################################################################################
            #############################################################################################################################################
            #############################################################################################################################################

        self.CloseSerialPort()

        self.MyPrint_WithoutLogFile("Finished the MainThread for DynamixelProtocol2Xseries_ReubenPython3Class object.")
        return
    #######################################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    def ExitProgram_Callback(self):

        print("Exiting all threads for DynamixelProtocol2Xseries_ReubenPython3Class object")

        self.EXIT_PROGRAM_FLAG = 1
    ##########################################################################################################
    ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    def StartGUI(self, GuiParent):

        self.GUI_Thread(GuiParent)
    ##########################################################################################################
    ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    def GUI_Thread(self, parent):

        try:
            print("Starting the GUI_Thread for DynamixelProtocol2Xseries_ReubenPython3Class object.")

            ##########################################################################################################
            ##########################################################################################################
            self.root = parent
            self.parent = parent
            ##########################################################################################################
            ##########################################################################################################

            ##########################################################################################################
            ##########################################################################################################
            self.myFrame = Frame(self.root)

            if self.UseBorderAroundThisGuiObjectFlag == 1:
                self.myFrame["borderwidth"] = 2
                self.myFrame["relief"] = "ridge"

            self.myFrame.grid(row = self.GUI_ROW,
                              column = self.GUI_COLUMN,
                              padx = self.GUI_PADX,
                              pady = self.GUI_PADY,
                              rowspan = self.GUI_ROWSPAN,
                              columnspan= self.GUI_COLUMNSPAN,
                              sticky = self.GUI_STICKY)
            ##########################################################################################################
            ##########################################################################################################

            ##########################################################################################################
            ##########################################################################################################
            self.TKinter_LightGreenColor = '#%02x%02x%02x' % (150, 255, 150) #RGB
            self.TKinter_LightRedColor = '#%02x%02x%02x' % (255, 150, 150) #RGB
            self.TKinter_LightYellowColor = '#%02x%02x%02x' % (255, 255, 150)  # RGB
            self.TKinter_DefaultGrayColor = '#%02x%02x%02x' % (240, 240, 240)  # RGB
            ##########################################################################################################
            ##########################################################################################################

            ########################################################################################################## SET THE DEFAULT FONT FOR ALL WIDGETS CREATED AFTTER/BELOW THIS CALL
            ##########################################################################################################
            default_font = tkFont.nametofont("TkDefaultFont") #TkTextFont, TkFixedFont
            default_font.configure(size=8)
            self.root.option_add("*Font", default_font)
            ##########################################################################################################
            ##########################################################################################################

            ##############################################################################################################
            ##############################################################################################################

            #################################################
            self.LabelsFrame = Frame(self.myFrame)
            self.LabelsFrame["borderwidth"] = 2
            self.LabelsFrame.grid(row=0, column=0, padx=self.GUI_PADX, pady=self.GUI_PADY, columnspan=1, rowspan=1)
            #################################################

            #################################################
            self.DeviceInfoLabel = Label(self.LabelsFrame, text="Device Info", width=50)
            self.DeviceInfoLabel.grid(row=0, column=0, padx=self.GUI_PADX, pady=self.GUI_PADY, columnspan=1, rowspan=1)
            #################################################

            #################################################
            self.DataLabel = Label(self.LabelsFrame, text="DataLabel", width=100)
            self.DataLabel.grid(row=1, column=0, padx=self.GUI_PADX, pady=self.GUI_PADY, columnspan=1, rowspan=1)
            #################################################

            #################################################
            self.ErrorLabel = Label(self.LabelsFrame, text="ErrorLabel", width=50)
            self.ErrorLabel.grid(row=2, column=0, padx=self.GUI_PADX, pady=self.GUI_PADY, columnspan=5, rowspan=1)
            #################################################

            #################################################
            self.ButtonsFrame = Frame(self.myFrame)
            self.ButtonsFrame["borderwidth"] = 2
            self.ButtonsFrame.grid(row=1, column=0, padx=self.GUI_PADX, pady=self.GUI_PADY, columnspan=1, rowspan=1)
            #################################################

            #################################################
            self.DisengageAllMotorsButton = Button(self.ButtonsFrame, text="Disengage All Motors", state="normal", bg = "red", width=20,command=lambda i=1: self.DisengageAllMotorsButtonResponse())
            self.DisengageAllMotorsButton.grid(row=0, column=0, padx=self.GUI_PADX, pady=self.GUI_PADY, columnspan=1, rowspan=1)
            #################################################

            #################################################
            self.EngageAllMotorsButton = Button(self.ButtonsFrame, text="Engage All Motors", state="normal", bg = "green", width=20,command=lambda i=1: self.EngageAllMotorsButtonResponse())
            self.EngageAllMotorsButton.grid(row=0, column=1, padx=self.GUI_PADX, pady=self.GUI_PADY, columnspan=1, rowspan=1)
            #################################################

            #################################################
            #################################################
            #################################################

            #################################################
            self.ScalesFrame = Frame(self.myFrame)
            self.ScalesFrame["borderwidth"] = 2
            self.ScalesFrame.grid(row=2, column=0, padx=self.GUI_PADX, pady=self.GUI_PADY, columnspan=1, rowspan=1)
            #################################################

            self.Position_DynamixelUnits_ScaleLabel = []
            self.Position_DynamixelUnits_ScaleValue = []
            self.Position_DynamixelUnits_Scale = []

            self.Velocity_DynamixelUnits_ScaleLabel = []
            self.Velocity_DynamixelUnits_ScaleValue = []
            self.Velocity_DynamixelUnits_Scale = []

            self.Current_DynamixelUnits_ScaleLabel = []
            self.Current_DynamixelUnits_ScaleValue = []
            self.Current_DynamixelUnits_Scale = []

            self.PWM_DynamixelUnits_ScaleLabel = []
            self.PWM_DynamixelUnits_ScaleValue = []
            self.PWM_DynamixelUnits_Scale = []

            self.TkinterScaleWidth = 10
            self.TkinterScaleLength = 200

            for MotorIndex in range(0, self.NumberOfMotors):
                #################################################
                self.Position_DynamixelUnits_ScaleLabel.append(Label(self.ScalesFrame, text="Pos (DU) M" + str(MotorIndex) + "\n" + self.MotorName_StringList[MotorIndex], width=self.TkinterScaleWidth))
                self.Position_DynamixelUnits_ScaleLabel[MotorIndex].grid(row=3 + MotorIndex*2, column=0, padx=self.GUI_PADX, pady=self.GUI_PADY, columnspan=1, rowspan=1)

                self.Position_DynamixelUnits_ScaleValue.append(DoubleVar())
                self.Position_DynamixelUnits_Scale.append(Scale(self.ScalesFrame,\
                                                from_=self.Position_DynamixelUnits_Min_UserSet[MotorIndex],\
                                                to=self.Position_DynamixelUnits_Max_UserSet[MotorIndex],\
                                                #tickinterval=(self.Position_DynamixelUnits_Max_UserSet[MotorIndex] - self.Position_DynamixelUnits_Min_UserSet[MotorIndex]) / 2.0,\
                                                orient=HORIZONTAL,\
                                                borderwidth=2,\
                                                showvalue=1,\
                                                width=self.TkinterScaleWidth,\
                                                length=self.TkinterScaleLength,\
                                                resolution=1,\
                                                variable=self.Position_DynamixelUnits_ScaleValue[MotorIndex]))
                self.Position_DynamixelUnits_Scale[MotorIndex].bind('<Button-1>', lambda event, name=MotorIndex: self.Position_DynamixelUnits_ScaleResponse(event, name))
                self.Position_DynamixelUnits_Scale[MotorIndex].bind('<B1-Motion>', lambda event, name=MotorIndex: self.Position_DynamixelUnits_ScaleResponse(event, name))
                self.Position_DynamixelUnits_Scale[MotorIndex].bind('<ButtonRelease-1>', lambda event, name=MotorIndex: self.Position_DynamixelUnits_ScaleResponse(event, name))
                self.Position_DynamixelUnits_Scale[MotorIndex].set(self.Position_DynamixelUnits_StartingValueList[MotorIndex])
                self.Position_DynamixelUnits_Scale[MotorIndex].grid(row=3 + MotorIndex*2, column=1, padx=self.GUI_PADX, pady=self.GUI_PADY, columnspan=1, rowspan=1)
                #################################################

                #################################################
                self.Velocity_DynamixelUnits_ScaleLabel.append(Label(self.ScalesFrame, text="Vel M" + str(MotorIndex) + "\n" + self.MotorName_StringList[MotorIndex], width=self.TkinterScaleWidth))
                self.Velocity_DynamixelUnits_ScaleLabel[MotorIndex].grid(row=3 + MotorIndex*2, column=2, padx=self.GUI_PADX, pady=self.GUI_PADY, columnspan=1, rowspan=1)

                self.Velocity_DynamixelUnits_ScaleValue.append(DoubleVar())
                self.Velocity_DynamixelUnits_Scale.append(Scale(self.ScalesFrame,\
                                                            from_=self.Velocity_DynamixelUnits_Min_UserSet[MotorIndex],\
                                                            to=self.Velocity_DynamixelUnits_Max_UserSet[MotorIndex],\
                                                            #tickinterval=(self.Velocity_DynamixelUnits_Max_UserSet[MotorIndex] - self.Velocity_DynamixelUnits_Min_UserSet[MotorIndex]) / 2.0,\
                                                            orient=HORIZONTAL,\
                                                            showvalue=1,\
                                                            width=self.TkinterScaleWidth,\
                                                            length=self.TkinterScaleLength,\
                                                            resolution=1,\
                                                            variable=self.Velocity_DynamixelUnits_ScaleValue[MotorIndex]))
                self.Velocity_DynamixelUnits_Scale[MotorIndex].bind('<Button-1>', lambda event, name=MotorIndex: self.Velocity_DynamixelUnits_ScaleResponse(event, name))
                self.Velocity_DynamixelUnits_Scale[MotorIndex].bind('<B1-Motion>', lambda event, name=MotorIndex: self.Velocity_DynamixelUnits_ScaleResponse(event, name))
                self.Velocity_DynamixelUnits_Scale[MotorIndex].bind('<ButtonRelease-1>', lambda event, name=MotorIndex: self.Velocity_DynamixelUnits_ScaleResponse(event, name))
                self.Velocity_DynamixelUnits_Scale[MotorIndex].set(self.Velocity_DynamixelUnits_StartingValueList[MotorIndex])
                self.Velocity_DynamixelUnits_Scale[MotorIndex].grid(row=3 + MotorIndex*2, column=3, padx=self.GUI_PADX, pady=self.GUI_PADY, columnspan=1, rowspan=1)
                #################################################

                #################################################
                self.Current_DynamixelUnits_ScaleLabel.append(Label(self.ScalesFrame, text="Cur M" + str(MotorIndex) + "\n" + self.MotorName_StringList[MotorIndex], width=self.TkinterScaleWidth))
                self.Current_DynamixelUnits_ScaleLabel[MotorIndex].grid(row=4 + MotorIndex*2, column=0, padx=self.GUI_PADX, pady=self.GUI_PADY, columnspan=1, rowspan=1)

                self.Current_DynamixelUnits_ScaleValue.append(DoubleVar())
                self.Current_DynamixelUnits_Scale.append(Scale(self.ScalesFrame, \
                                                from_=self.Current_DynamixelUnits_min[MotorIndex],\
                                                to=self.Current_DynamixelUnits_max[MotorIndex],\
                                                #tickinterval=(self.Current_DynamixelUnits_max[MotorIndex] - self.Current_DynamixelUnits_min[MotorIndex]) / 2.0,\
                                                orient=HORIZONTAL,\
                                                borderwidth=2,\
                                                showvalue=1,\
                                                width=self.TkinterScaleWidth,\
                                                length=self.TkinterScaleLength,\
                                                resolution=1,\
                                                variable=self.Current_DynamixelUnits_ScaleValue[MotorIndex]))
                self.Current_DynamixelUnits_Scale[MotorIndex].bind('<Button-1>', lambda event, name=MotorIndex: self.Current_DynamixelUnits_ScaleResponse(event, name))
                self.Current_DynamixelUnits_Scale[MotorIndex].bind('<B1-Motion>', lambda event, name=MotorIndex: self.Current_DynamixelUnits_ScaleResponse(event, name))
                self.Current_DynamixelUnits_Scale[MotorIndex].bind('<ButtonRelease-1>', lambda event, name=MotorIndex: self.Current_DynamixelUnits_ScaleResponse(event, name))
                self.Current_DynamixelUnits_Scale[MotorIndex].set(self.Current_DynamixelUnits_StartingValueList[MotorIndex])
                self.Current_DynamixelUnits_Scale[MotorIndex].grid(row=4 + MotorIndex*2, column=1, padx=self.GUI_PADX, pady=self.GUI_PADY, columnspan=1, rowspan=1)
                #################################################

                #################################################
                self.PWM_DynamixelUnits_ScaleLabel.append(Label(self.ScalesFrame, text="PWM M" + str(MotorIndex) + "\n" + self.MotorName_StringList[MotorIndex], width=self.TkinterScaleWidth))
                self.PWM_DynamixelUnits_ScaleLabel[MotorIndex].grid(row=4 + MotorIndex*2, column=2, padx=self.GUI_PADX, pady=self.GUI_PADY, columnspan=1, rowspan=1)

                self.PWM_DynamixelUnits_ScaleValue.append(DoubleVar())
                self.PWM_DynamixelUnits_Scale.append(Scale(self.ScalesFrame, \
                                                from_=self.PWM_DynamixelUnits_min[MotorIndex],\
                                                to=self.PWM_DynamixelUnits_max[MotorIndex],\
                                                #tickinterval=(self.PWM_DynamixelUnits_max[MotorIndex] - self.PWM_DynamixelUnits_min[MotorIndex]) / 2.0,\
                                                orient=HORIZONTAL,\
                                                borderwidth=2,\
                                                showvalue=1,\
                                                width=self.TkinterScaleWidth,\
                                                length=self.TkinterScaleLength,\
                                                resolution=1,\
                                                variable=self.PWM_DynamixelUnits_ScaleValue[MotorIndex]))
                self.PWM_DynamixelUnits_Scale[MotorIndex].bind('<Button-1>', lambda event, name=MotorIndex: self.PWM_DynamixelUnits_ScaleResponse(event, name))
                self.PWM_DynamixelUnits_Scale[MotorIndex].bind('<B1-Motion>', lambda event, name=MotorIndex: self.PWM_DynamixelUnits_ScaleResponse(event, name))
                self.PWM_DynamixelUnits_Scale[MotorIndex].bind('<ButtonRelease-1>', lambda event, name=MotorIndex: self.PWM_DynamixelUnits_ScaleResponse(event, name))
                self.PWM_DynamixelUnits_Scale[MotorIndex].set(self.PWM_DynamixelUnits_StartingValueList[MotorIndex])
                self.PWM_DynamixelUnits_Scale[MotorIndex].grid(row=4 + MotorIndex*2, column=3, padx=self.GUI_PADX, pady=self.GUI_PADY, columnspan=1, rowspan=1)
                #################################################

            #################################################
            #################################################
            #################################################

            #################################################
            #################################################
            #################################################

            #################################################
            self.IndividualMotorWidgetsFrame = Frame(self.myFrame)
            self.IndividualMotorWidgetsFrame["borderwidth"] = 2
            self.IndividualMotorWidgetsFrame.grid(row=3, column=0, padx=self.GUI_PADX, pady=self.GUI_PADY, columnspan=1, rowspan=1)
            #################################################

            self.EngagedState_Checkbutton = []
            self.EngagedState_Checkbutton_Value = []

            self.LEDstate_Checkbutton = []
            self.LEDstate_Checkbutton_Value = []

            self.ToggleMinMax_Button = []
            self.ResetSerial_Button = []
            self.Reboot_Button = []

            self.OperatingModeRadiobutton_SelectionVar = []
            self.OperatingModeRadiobutton_CurrentControl = []
            self.OperatingModeRadiobutton_VelocityControl = []
            self.OperatingModeRadiobutton_PositionControl = []
            self.OperatingModeRadiobutton_ExtendedPositionControlMultiTurn = []
            self.OperatingModeRadiobutton_CurrentBasedPositionControl = []
            self.OperatingModeRadiobutton_PWMcontrol = []

            self.TkinterScaleWidth = 10
            self.TkinterScaleLength = 200
            
            self.TkinterButtonWidth = 15
            self.TkinterCheckbuttonWidth = 20
            self.TkinterOperatingModeRadiobuttonWidth = 25

            for MotorIndex in range(0, self.NumberOfMotors):
            #################################################

                ###########################################################
                self.ToggleMinMax_Button.append(Button(self.IndividualMotorWidgetsFrame,
                                                width=self.TkinterButtonWidth,
                                                text='ToggleMinMax M' + str(MotorIndex),
                                                state="normal"))
                self.ToggleMinMax_Button[MotorIndex].bind('<ButtonRelease-1>', lambda event, name=MotorIndex: self.ToggleMinMax_ButtonResponse(event, name))
                self.ToggleMinMax_Button[MotorIndex].grid(row=3 + MotorIndex*2, column=5, padx=self.GUI_PADX, pady=self.GUI_PADY, columnspan=1, rowspan=1)
                ###########################################################

                ###########################################################
                self.ResetSerial_Button.append(Button(self.IndividualMotorWidgetsFrame,
                                                width=self.TkinterButtonWidth,
                                                text='ResetSerial M' + str(MotorIndex),
                                                state="normal"))
                self.ResetSerial_Button[MotorIndex].bind('<ButtonRelease-1>', lambda event, name=MotorIndex: self.ResetSerial_ButtonResponse(event, name))
                self.ResetSerial_Button[MotorIndex].grid(row=3 + MotorIndex*2, column=6, padx=self.GUI_PADX, pady=self.GUI_PADY, columnspan=1, rowspan=1)
                ###########################################################

                ###########################################################
                self.Reboot_Button.append(Button(self.IndividualMotorWidgetsFrame,
                                                width=self.TkinterButtonWidth,
                                                text='Reboot M' + str(MotorIndex),
                                                state="normal"))
                self.Reboot_Button[MotorIndex].bind('<ButtonRelease-1>', lambda event, name=MotorIndex: self.Reboot_ButtonResponse(event, name))
                self.Reboot_Button[MotorIndex].grid(row=3 + MotorIndex*2, column=7, padx=self.GUI_PADX, pady=self.GUI_PADY, columnspan=1, rowspan=1)
                ###########################################################

                ###########################################################
                self.EngagedState_Checkbutton_Value.append(DoubleVar())

                if self.EngagedState[MotorIndex] == 1:
                    self.EngagedState_Checkbutton_Value[MotorIndex].set(1)
                else:
                    self.EngagedState_Checkbutton_Value[MotorIndex].set(0)

                self.EngagedState_Checkbutton.append(Checkbutton(self.IndividualMotorWidgetsFrame,
                                                                width=self.TkinterCheckbuttonWidth,
                                                                text='Engage M' + str(MotorIndex),
                                                                state="normal",
                                                                variable=self.EngagedState_Checkbutton_Value[MotorIndex]))
                self.EngagedState_Checkbutton[MotorIndex].bind('<ButtonRelease-1>', lambda event, name=MotorIndex: self.EngagedState_CheckbuttonResponse(event, name))
                self.EngagedState_Checkbutton[MotorIndex].grid(row=3 + MotorIndex*2, column=8, padx=self.GUI_PADX, pady=self.GUI_PADY, columnspan=1, rowspan=1)
                ###########################################################

                ###########################################################
                self.LEDstate_Checkbutton_Value.append(DoubleVar())
                self.LEDstate_Checkbutton_Value[MotorIndex].set(0)
                self.LEDstate_Checkbutton.append(Checkbutton(self.IndividualMotorWidgetsFrame,
                                                                width=self.TkinterCheckbuttonWidth,
                                                                text='LED M' + str(MotorIndex),
                                                                state="normal",
                                                                variable=self.LEDstate_Checkbutton_Value[MotorIndex]))
                self.LEDstate_Checkbutton[MotorIndex].bind('<ButtonRelease-1>', lambda event, name=MotorIndex: self.LEDstate_CheckbuttonResponse(event, name))
                self.LEDstate_Checkbutton[MotorIndex].grid(row=3 + MotorIndex*2, column=9, padx=self.GUI_PADX, pady=self.GUI_PADY, columnspan=1, rowspan=1)
                ###########################################################

                ###########################################################
                self.OperatingModeRadiobutton_SelectionVar.append(StringVar())
                ###########################################################

                ###########################################################
                self.OperatingModeRadiobutton_CurrentControl.append(Radiobutton(self.IndividualMotorWidgetsFrame,
                                                              text="CurrentControl",
                                                              state="normal",
                                                              width=self.TkinterOperatingModeRadiobuttonWidth,
                                                              anchor="w",
                                                              variable=self.OperatingModeRadiobutton_SelectionVar[MotorIndex],
                                                              value="CurrentControl",
                                                              command=lambda name=MotorIndex: self.OperatingModeRadiobutton_Response(name)))
                self.OperatingModeRadiobutton_CurrentControl[MotorIndex].grid(row=4 + MotorIndex*2, column=5, padx=self.GUI_PADX, pady=self.GUI_PADY, columnspan=1, rowspan=1)
                if self.OperatingMode_StartingValueList[MotorIndex] == "CurrentControl":
                    self.OperatingModeRadiobutton_CurrentControl[MotorIndex].select()
                    #self.OperatingModeRadiobutton_CurrentControl[MotorIndex].invoke()
                ###########################################################

                ###########################################################
                self.OperatingModeRadiobutton_VelocityControl.append(Radiobutton(self.IndividualMotorWidgetsFrame,
                                                              text="VelocityControl",
                                                              state="normal",
                                                              width=self.TkinterOperatingModeRadiobuttonWidth,
                                                              anchor="w",
                                                              variable=self.OperatingModeRadiobutton_SelectionVar[MotorIndex],
                                                              value="VelocityControl",
                                                              command=lambda name=MotorIndex: self.OperatingModeRadiobutton_Response(name)))
                self.OperatingModeRadiobutton_VelocityControl[MotorIndex].grid(row=4 + MotorIndex*2, column=6, padx=self.GUI_PADX, pady=self.GUI_PADY, columnspan=1, rowspan=1)
                if self.OperatingMode_StartingValueList[MotorIndex] == "VelocityControl":
                    self.OperatingModeRadiobutton_VelocityControl[MotorIndex].select()
                    #self.OperatingModeRadiobutton_VelocityControl[MotorIndex].invoke()
                ###########################################################

                ###########################################################
                self.OperatingModeRadiobutton_PositionControl.append(Radiobutton(self.IndividualMotorWidgetsFrame,
                                                              text="PositionControl",
                                                              state="normal",
                                                              width=self.TkinterOperatingModeRadiobuttonWidth,
                                                              anchor="w",
                                                              variable=self.OperatingModeRadiobutton_SelectionVar[MotorIndex],
                                                              value="PositionControl",
                                                              command=lambda name=MotorIndex: self.OperatingModeRadiobutton_Response(name)))
                self.OperatingModeRadiobutton_PositionControl[MotorIndex].grid(row=4 + MotorIndex*2, column=7, padx=self.GUI_PADX, pady=self.GUI_PADY, columnspan=1, rowspan=1)
                if self.OperatingMode_StartingValueList[MotorIndex] == "PositionControl":
                    self.OperatingModeRadiobutton_PositionControl[MotorIndex].select()
                    #self.OperatingModeRadiobutton_PositionControl[MotorIndex].invoke()
                ###########################################################

                ###########################################################
                self.OperatingModeRadiobutton_ExtendedPositionControlMultiTurn.append(Radiobutton(self.IndividualMotorWidgetsFrame,
                                                              text="ExtendedPositionControlMultiTurn",
                                                              state="normal",
                                                              width=self.TkinterOperatingModeRadiobuttonWidth,
                                                              anchor="w",
                                                              variable=self.OperatingModeRadiobutton_SelectionVar[MotorIndex],
                                                              value="ExtendedPositionControlMultiTurn",
                                                              command=lambda name=MotorIndex: self.OperatingModeRadiobutton_Response(name)))
                self.OperatingModeRadiobutton_ExtendedPositionControlMultiTurn[MotorIndex].grid(row=4 + MotorIndex*2, column=8, padx=self.GUI_PADX, pady=self.GUI_PADY, columnspan=1, rowspan=1)
                if self.OperatingMode_StartingValueList[MotorIndex] == "ExtendedPositionControlMultiTurn":
                    self.OperatingModeRadiobutton_ExtendedPositionControlMultiTurn[MotorIndex].select()
                    #self.OperatingModeRadiobutton_ExtendedPositionControlMultiTurn[MotorIndex].invoke()
                ###########################################################

                ###########################################################
                self.OperatingModeRadiobutton_CurrentBasedPositionControl.append(Radiobutton(self.IndividualMotorWidgetsFrame,
                                                              text="CurrentBasedPositionControl",
                                                              state="normal",
                                                              width=self.TkinterOperatingModeRadiobuttonWidth,
                                                              anchor="w",
                                                              variable=self.OperatingModeRadiobutton_SelectionVar[MotorIndex],
                                                              value="CurrentBasedPositionControl",
                                                              command=lambda name=MotorIndex: self.OperatingModeRadiobutton_Response(name)))
                self.OperatingModeRadiobutton_CurrentBasedPositionControl[MotorIndex].grid(row=4 + MotorIndex*2, column=9, padx=self.GUI_PADX, pady=self.GUI_PADY, columnspan=1, rowspan=1)
                if self.OperatingMode_StartingValueList[MotorIndex] == "CurrentBasedPositionControl":
                    self.OperatingModeRadiobutton_CurrentBasedPositionControl[MotorIndex].select()
                    #self.OperatingModeRadiobutton_CurrentBasedPositionControl[MotorIndex].invoke()
                ###########################################################

                ###########################################################
                self.OperatingModeRadiobutton_PWMcontrol.append(Radiobutton(self.IndividualMotorWidgetsFrame,
                                                              text="PWMcontrol",
                                                              state="normal",
                                                              width=self.TkinterOperatingModeRadiobuttonWidth,
                                                              anchor="w",
                                                              variable=self.OperatingModeRadiobutton_SelectionVar[MotorIndex],
                                                              value="PWMcontrol",
                                                              command=lambda name=MotorIndex: self.OperatingModeRadiobutton_Response(name)))
                self.OperatingModeRadiobutton_PWMcontrol[MotorIndex].grid(row=4 + MotorIndex*2, column=10, padx=self.GUI_PADX, pady=self.GUI_PADY, columnspan=10, rowspan=1)
                if self.OperatingMode_StartingValueList[MotorIndex] == "PWMcontrol":
                    self.OperatingModeRadiobutton_PWMcontrol[MotorIndex].select()
                    #self.OperatingModeRadiobutton_PWMcontrol[MotorIndex].invoke()
                ###########################################################

            #################################################
            #################################################
            #################################################

            ##########################################################################################################
            ##########################################################################################################
            self.PrintToGui_Label = Label(self.myFrame, text="PrintToGui_Label", width=75)
            if self.EnableInternal_MyPrint_Flag == 1:
                self.PrintToGui_Label.grid(row=50, column=0, padx=self.GUI_PADX, pady=self.GUI_PADY, columnspan=4, rowspan=10)
            ##########################################################################################################
            ##########################################################################################################

            ##########################################################################################################
            ##########################################################################################################
            self.GUI_ready_to_be_updated_flag = 1
            ##########################################################################################################
            ##########################################################################################################

        except:
            exceptions = sys.exc_info()[0]
            print("DynamixelProtocol2Xseries_ReubenPython3Class __init__: DataStreamingFrequency_CalculatedFromMainThread_LowPassFilter_ReubenPython2and3ClassObject, Exceptions: %s" % exceptions)
            traceback.print_exc()

    ##########################################################################################################
    ##########################################################################################################

    ##########################################################################################################
    def OperatingModeRadiobutton_Response(self, name):

        MotorNumber = int(name)

        self.OperatingMode_TO_BE_SET[MotorNumber] = self.OperatingModeRadiobutton_SelectionVar[MotorNumber].get()
        self.OperatingMode_NEEDS_TO_BE_CHANGED_FLAG[MotorNumber] = 1
        self.MyPrint_WithoutLogFile("Motor " + str(MotorNumber) + " OperatingMode set to: " + self.OperatingMode_TO_BE_SET[MotorNumber])
    ##########################################################################################################

    #######################################################################################################################
    #######################################################################################################################
    def ToggleMinMax_ButtonResponse(self, event = None, name = "default"):

        MotorNumber = int(name)

        if self.ToggleMinMax_state[MotorNumber] == -1:
            self.ToggleMinMax_TO_BE_SET[MotorNumber] = 0
        elif self.ToggleMinMax_state[MotorNumber] == 0:
            self.ToggleMinMax_TO_BE_SET[MotorNumber] = 1
        elif self.ToggleMinMax_state[MotorNumber] == 1:
            self.ToggleMinMax_TO_BE_SET[MotorNumber] = -1
            
        self.ToggleMinMax_NeedsToTakePlaceFlag[MotorNumber] = 1

        self.MyPrint_WithoutLogFile("ToggleMinMax_ButtonResponse event fired for motor: " + str(MotorNumber) + ", state = " + str(self.ToggleMinMax_TO_BE_SET[MotorNumber]))
    #######################################################################################################################
    #######################################################################################################################

    #######################################################################################################################
    #######################################################################################################################
    def ResetSerial_ButtonResponse(self, event = None, name = "default"):

        MotorNumber = int(name)

        self.ResetSerial_NeedsToTakePlaceFlag[MotorNumber] = 1

        self.MyPrint_WithoutLogFile("Reset Serial event fired for motor : " + str(MotorNumber))
    #######################################################################################################################
    #######################################################################################################################

    #######################################################################################################################
    #######################################################################################################################
    def Reboot_ButtonResponse(self, event = None, name = "default"):

        MotorNumber = int(name)

        self.Reboot_NeedsToTakePlaceFlag[MotorNumber] = 1

        self.MyPrint_WithoutLogFile("Reboot Button event fired for motor : " + str(MotorNumber))
    #######################################################################################################################
    #######################################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    ##########################################################################################################
    ##########################################################################################################
    ##########################################################################################################
    ##########################################################################################################
    def GUI_update_clock(self):

        ##########################################################################################################
        ##########################################################################################################
        ##########################################################################################################
        ##########################################################################################################
        ##########################################################################################################
        if self.USE_GUI_FLAG == 1 and self.EXIT_PROGRAM_FLAG == 0:

            ##########################################################################################################
            ##########################################################################################################
            ##########################################################################################################
            ##########################################################################################################
            if self.GUI_ready_to_be_updated_flag == 1:

                ##########################################################################################################
                ##########################################################################################################
                ##########################################################################################################
                try:

                    #######################################################
                    #######################################################
                    self.DataLabel["text"] = self.ConvertDictToProperlyFormattedStringForPrinting(self.MostRecentDataDict_ForExternalQueryAndGUIdisplayOnly,
                                                                                                    NumberOfDecimalsPlaceToUse = 2,
                                                                                                    NumberOfEntriesPerLine = 1,
                                                                                                    NumberOfTabsBetweenItems = 3)
                    #######################################################
                    #######################################################

                    #######################################################
                    self.DeviceInfoLabel["text"] = "NameToDisplay_UserSet: " + str(self.NameToDisplay_UserSet)
                    #######################################################

                    #######################################################
                    #######################################################

                    #######################################################
                    self.ErrorLabel["text"] = "ErrorFlag_BYTE: " + str(self.ErrorFlag_BYTE) + \
                                            "\nErrorFlag_Overload_Received: " + str(self.ErrorFlag_Overload_Received) + \
                                            "\nErrorFlag_ElectricalShock_Received: " + str(self.ErrorFlag_ElectricalShock_Received) + \
                                            "\nErrorFlag_MotorEncoder_Received: " + str(self.ErrorFlag_MotorEncoder_Received) + \
                                            "\nErrorFlag_Overheating_Received: " + str(self.ErrorFlag_Overheating_Received) + \
                                            "\nErrorFlag_InputVoltage_Received: " + str(self.ErrorFlag_InputVoltage_Received) + \
                                            "\nErrorFlag_SerialCommunication: " + str(self.ErrorFlag_SerialCommunication) + \
                                            "\nErrorFlag_OperatingModeMismatch: " + str(self.ErrorFlag_OperatingModeMismatch)
                    #######################################################

                    #######################################################
                    ErrorFlagTripped = 0
                    for MotorIndex in range(0, self.NumberOfMotors):
                        if self.ErrorFlag_Overload_Received[MotorIndex] != 0 or self.ErrorFlag_ElectricalShock_Received[MotorIndex] != 0 or self.ErrorFlag_Overheating_Received[MotorIndex] != 0 or self.ErrorFlag_InputVoltage_Received[MotorIndex] != 0 or self.ErrorFlag_SerialCommunication[MotorIndex] != 0 or self.ErrorFlag_OperatingModeMismatch[MotorIndex] != 0:
                            ErrorFlagTripped = 1

                    if ErrorFlagTripped == 0:
                        self.ErrorLabel["bg"] = self.TKinter_DefaultGrayColor
                    else:
                        self.ErrorLabel["bg"] = self.TKinter_LightRedColor
                    #######################################################

                    #######################################################
                    #######################################################

                    #######################################################
                    #######################################################
                    for MotorIndex in range(0, self.NumberOfMotors):

                        #########################################################
                        if self.Position_DynamixelUnits_GUI_NeedsToBeChangedFlag[MotorIndex] == 1:
                            self.Position_DynamixelUnits_Scale[MotorIndex].set(self.Position_DynamixelUnits_TO_BE_SET[MotorIndex])
                            self.Position_DynamixelUnits_GUI_NeedsToBeChangedFlag[MotorIndex] = 0
                        #########################################################

                        #########################################################
                        if self.Velocity_DynamixelUnits_GUI_NeedsToBeChangedFlag[MotorIndex] == 1:
                            self.Velocity_DynamixelUnits_Scale[MotorIndex].set(self.Velocity_DynamixelUnits_TO_BE_SET[MotorIndex])
                            self.Velocity_DynamixelUnits_GUI_NeedsToBeChangedFlag[MotorIndex] = 0
                        #########################################################

                        #########################################################
                        if self.Current_DynamixelUnits_GUI_NeedsToBeChangedFlag[MotorIndex] == 1:
                            self.Current_DynamixelUnits_Scale[MotorIndex].set(self.Current_DynamixelUnits_TO_BE_SET[MotorIndex])
                            self.Current_DynamixelUnits_GUI_NeedsToBeChangedFlag[MotorIndex] = 0
                        #########################################################

                        #########################################################
                        if self.PWM_DynamixelUnits_GUI_NeedsToBeChangedFlag[MotorIndex] == 1:
                            self.PWM_DynamixelUnits_Scale[MotorIndex].set(self.PWM_DynamixelUnits_TO_BE_SET[MotorIndex])
                            self.PWM_DynamixelUnits_GUI_NeedsToBeChangedFlag[MotorIndex] = 0
                        #########################################################

                        #########################################################
                        if self.OperatingMode_GUI_NEEDS_TO_BE_CHANGED_FLAG[MotorIndex] == 1:

                            if self.OperatingMode_TO_BE_SET[MotorIndex] == "CurrentControl":
                                self.OperatingModeRadiobutton_CurrentControl[MotorIndex].select()

                            elif self.OperatingMode_TO_BE_SET[MotorIndex] == "VelocityControl":
                                self.OperatingModeRadiobutton_VelocityControl[MotorIndex].select()

                            elif self.OperatingMode_TO_BE_SET[MotorIndex] == "PositionControl":
                                self.OperatingModeRadiobutton_PositionControl[MotorIndex].select()

                            elif self.OperatingMode_TO_BE_SET[MotorIndex] == "ExtendedPositionControlMultiTurnControl":
                                self.OperatingModeRadiobutton_ExtendedPositionControlMultiTurn[MotorIndex].select()

                            elif self.OperatingMode_TO_BE_SET[MotorIndex] == "CurrentBasedPositionControl":
                                self.OperatingModeRadiobutton_CurrentBasedPositionControl[MotorIndex].select()

                            elif self.OperatingMode_TO_BE_SET[MotorIndex] == "PWMcontrol":
                                self.OperatingModeRadiobutton_PWMcontrol[MotorIndex].select()

                            self.OperatingMode_GUI_NEEDS_TO_BE_CHANGED_FLAG[MotorIndex] = 0
                        #########################################################

                        #########################################################
                        if self.EngagedState_GUI_NeedsToBeChangedFlag[MotorIndex] == 1:

                            if self.EngagedState_TO_BE_SET[MotorIndex] == 1: #This actually changes how the widget looks
                                self.EngagedState_Checkbutton[MotorIndex].select()
                            elif self.EngagedState_TO_BE_SET[MotorIndex] == 0:
                                self.EngagedState_Checkbutton[MotorIndex].deselect()

                            self.EngagedState_GUI_NeedsToBeChangedFlag[MotorIndex] = 0
                        #########################################################

                        #########################################################
                        if self.LEDstate_GUI_NeedsToBeChangedFlag[MotorIndex] == 1:

                            if self.LEDstate_TO_BE_SET[MotorIndex] == 1: #This actually changes how the widget looks
                                self.LEDstate_Checkbutton[MotorIndex].select()
                            elif self.LEDstate_TO_BE_SET[MotorIndex] == 0:
                                self.LEDstate_Checkbutton[MotorIndex].deselect()

                            self.LEDstate_GUI_NeedsToBeChangedFlag[MotorIndex] = 0
                        #########################################################

                        #########################################################
                        if self.EngagedState[MotorIndex] == 1:
                            self.Position_DynamixelUnits_Scale[MotorIndex]["troughcolor"] = self.TKinter_LightGreenColor
                            self.Velocity_DynamixelUnits_Scale[MotorIndex]["troughcolor"] = self.TKinter_LightGreenColor
                            self.Current_DynamixelUnits_Scale[MotorIndex]["troughcolor"] = self.TKinter_LightGreenColor
                            self.PWM_DynamixelUnits_Scale[MotorIndex]["troughcolor"] = self.TKinter_LightGreenColor
                        else:
                            self.Position_DynamixelUnits_Scale[MotorIndex]["troughcolor"] = self.TKinter_LightRedColor
                            self.Velocity_DynamixelUnits_Scale[MotorIndex]["troughcolor"] = self.TKinter_LightRedColor
                            self.Current_DynamixelUnits_Scale[MotorIndex]["troughcolor"] = self.TKinter_LightRedColor
                            self.PWM_DynamixelUnits_Scale[MotorIndex]["troughcolor"] = self.TKinter_LightRedColor
                        #########################################################

                    ##########################################################################################################
                    self.PrintToGui_Label.config(text=self.PrintToGui_Label_TextInput_Str)
                    ##########################################################################################################

                ##########################################################################################################
                ##########################################################################################################
                ##########################################################################################################

                ##########################################################################################################
                ##########################################################################################################
                ##########################################################################################################
                except:
                    exceptions = sys.exc_info()[0]
                    print("DynamixelProtocol2Xseries_ReubenPython3Class GUI_update_clock, exceptions: %s" % exceptions)
                    traceback.print_exc()
                ##########################################################################################################
                ##########################################################################################################
                ##########################################################################################################

            ##########################################################################################################
            ##########################################################################################################
            ##########################################################################################################
            ##########################################################################################################

        ##########################################################################################################
        ##########################################################################################################
        ##########################################################################################################
        ##########################################################################################################
        ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    ##########################################################################################################
    ##########################################################################################################
    ##########################################################################################################
    ##########################################################################################################

    #######################################################################################################################
    def Position_DynamixelUnits_ScaleResponse(self, event, name):

        MotorIndex = name
        self.Position_DynamixelUnits_TO_BE_SET[MotorIndex] = self.Position_DynamixelUnits_ScaleValue[MotorIndex].get()
        self.Position_DynamixelUnits_NeedsToBeChangedFlag[MotorIndex] = 1

        #self.MyPrint_WithoutLogFile("ScaleResponse: Position set to: " + str(self.Position_DynamixelUnits_TO_BE_SET[MotorIndex]) + " on motor " + str(MotorIndex))
    #######################################################################################################################

    #######################################################################################################################
    def Velocity_DynamixelUnits_ScaleResponse(self, event, name):

        MotorIndex = name
        self.Velocity_DynamixelUnits_TO_BE_SET[MotorIndex] = self.Velocity_DynamixelUnits_ScaleValue[MotorIndex].get()
        self.Velocity_DynamixelUnits_NeedsToBeChangedFlag[MotorIndex] = 1

        #self.MyPrint_WithoutLogFile("ScaleResponse: Velocity set to: " + str(self.Velocity_DynamixelUnits_TO_BE_SET[MotorIndex]) + " on motor " + str(MotorIndex))
    #######################################################################################################################

    #######################################################################################################################
    def Current_DynamixelUnits_ScaleResponse(self, event, name):

        MotorIndex = name
        self.Current_DynamixelUnits_TO_BE_SET[MotorIndex] = self.Current_DynamixelUnits_ScaleValue[MotorIndex].get()
        self.Current_DynamixelUnits_NeedsToBeChangedFlag[MotorIndex] = 1

        #self.MyPrint_WithoutLogFile("ScaleResponse: Current set to: " + str(self.Current_DynamixelUnits_TO_BE_SET[MotorIndex]) + " on motor " + str(MotorIndex))
    #######################################################################################################################
    
    #######################################################################################################################
    def PWM_DynamixelUnits_ScaleResponse(self, event, name):

        MotorIndex = name
        self.PWM_DynamixelUnits_TO_BE_SET[MotorIndex] = self.PWM_DynamixelUnits_ScaleValue[MotorIndex].get()
        self.PWM_DynamixelUnits_NeedsToBeChangedFlag[MotorIndex] = 1

        #self.MyPrint_WithoutLogFile("ScaleResponse: PWM set to: " + str(self.PWM_DynamixelUnits_TO_BE_SET[MotorIndex]) + " on motor " + str(MotorIndex))
    #######################################################################################################################

    #######################################################################################################################
    def EngagedState_CheckbuttonResponse(self, event, name):

        MotorIndex = name
        temp_value = self.EngagedState_Checkbutton_Value[MotorIndex].get()

        if temp_value == 0:
            self.EngagedState_TO_BE_SET[MotorIndex] = 1 ########## This reversal is needed for the variable state to match the checked state, but we don't know why
        elif temp_value == 1:
            self.EngagedState_TO_BE_SET[MotorIndex] = 0

        self.EngagedState_NeedsToBeChangedFlag[MotorIndex] = 1
        self.MyPrint_WithoutLogFile("EngagedState_CheckbuttonResponse: EngagedState changed to " + str(self.EngagedState_TO_BE_SET[MotorIndex]) + " on motor " + str(MotorIndex))
    #######################################################################################################################

    #######################################################################################################################
    def LEDstate_CheckbuttonResponse(self, event, name):

        MotorIndex = name
        temp_value = self.LEDstate_Checkbutton_Value[MotorIndex].get()

        if temp_value == 0:
            self.LEDstate_TO_BE_SET[MotorIndex] = 1 ########## This reversal is needed for the variable state to match the checked state, but we don't know why
        elif temp_value == 1:
            self.LEDstate_TO_BE_SET[MotorIndex] = 0

        self.LEDstate_NeedsToBeChangedFlag[MotorIndex] = 1
        #self.MyPrint_WithoutLogFile("LEDstate_CheckbuttonResponse: LEDstate changed to " + str(self.LEDstate_TO_BE_SET[MotorIndex]) + " on motor " + str(MotorIndex))
    #######################################################################################################################

    #######################################################################################################################
    def DisengageAllMotorsButtonResponse(self):

        for MotorIndex in range(0, len(self.EngagedState_TO_BE_SET)):
            if self.MotorType_StringList[MotorIndex] != "None":
                #print("DisengageAllMotorsButtonResponse MotorIndex: " + str(MotorIndex))
                self.EngagedState_TO_BE_SET[MotorIndex] = 0
                self.EngagedState_NeedsToBeChangedFlag[MotorIndex] = 1

        self.MyPrint_WithoutLogFile("DisengageAllMotorsButtonResponse")
    #######################################################################################################################

    #######################################################################################################################
    def EngageAllMotorsButtonResponse(self):

        for MotorIndex in range(0, len(self.EngagedState_TO_BE_SET)):
            if self.MotorType_StringList[MotorIndex]  != "None":
                #print("EngageAllMotorsButtonResponse MotorIndex: " + str(MotorIndex))
                self.EngagedState_TO_BE_SET[MotorIndex] = 1
                self.EngagedState_NeedsToBeChangedFlag[MotorIndex] = 1

        self.MyPrint_WithoutLogFile("EngageAllMotorsButtonResponse")
    #######################################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    def LimitNumber_IntOutputOnly(self, min_val, max_val, test_val):
        if test_val > max_val:
            test_val = max_val

        elif test_val < min_val:
            test_val = min_val

        else:
            test_val = test_val

        test_val = int(test_val)

        return test_val
    ##########################################################################################################
    ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    def LimitNumber_FloatOutputOnly(self, min_val, max_val, test_val):
        if test_val > max_val:
            test_val = max_val

        elif test_val < min_val:
            test_val = min_val

        else:
            test_val = test_val

        test_val = float(test_val)

        return test_val
    ##########################################################################################################
    ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    def LimitTextEntryInput(self, min_val, max_val, test_val, TextEntryObject):

        try:
            test_val = float(test_val)  # MUST HAVE THIS LINE TO CATCH STRINGS PASSED INTO THE FUNCTION

            if test_val > max_val:
                test_val = max_val
            elif test_val < min_val:
                test_val = min_val
            else:
                test_val = test_val

        except:
            pass

        try:
            if TextEntryObject != "":
                if isinstance(TextEntryObject, list) == 1:  # Check if the input 'TextEntryObject' is a list or not
                    TextEntryObject[0].set(str(test_val))  # Reset the text, overwriting the bad value that was entered.
                else:
                    TextEntryObject.set(str(test_val))  # Reset the text, overwriting the bad value that was entered.
        except:
            pass

        return test_val
    ##########################################################################################################
    ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    def MyPrint_WithoutLogFile(self, input_string):

        input_string = str(input_string)

        if input_string != "":

            #input_string = input_string.replace("\n", "").replace("\r", "")

            ################################ Write to console
            # Some people said that print crashed for pyinstaller-built-applications and that sys.stdout.write fixed this.
            # http://stackoverflow.com/questions/13429924/pyinstaller-packaged-application-works-fine-in-console-mode-crashes-in-window-m
            if self.PrintToConsoleFlag == 1:
                sys.stdout.write(input_string + "\n")
            ################################

            ################################ Write to GUI
            self.PrintToGui_Label_TextInputHistory_List.append(self.PrintToGui_Label_TextInputHistory_List.pop(0)) #Shift the list
            self.PrintToGui_Label_TextInputHistory_List[-1] = str(input_string) #Add the latest value

            self.PrintToGui_Label_TextInput_Str = ""
            for Counter, Line in enumerate(self.PrintToGui_Label_TextInputHistory_List):
                self.PrintToGui_Label_TextInput_Str = self.PrintToGui_Label_TextInput_Str + Line

                if Counter < len(self.PrintToGui_Label_TextInputHistory_List) - 1:
                    self.PrintToGui_Label_TextInput_Str = self.PrintToGui_Label_TextInput_Str + "\n"
            ################################

    ##########################################################################################################
    ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    ##########################################################################################################
    ##########################################################################################################
    def ConvertFloatToStringWithNumberOfLeadingNumbersAndDecimalPlaces_NumberOrListInput(self, input, number_of_leading_numbers = 4, number_of_decimal_places = 3):

        number_of_decimal_places = max(1, number_of_decimal_places) #Make sure we're above 1

        ListOfStringsToJoin = []

        ##########################################################################################################
        ##########################################################################################################
        ##########################################################################################################
        if isinstance(input, str) == 1:
            ListOfStringsToJoin.append(input)
        ##########################################################################################################
        ##########################################################################################################
        ##########################################################################################################

        ##########################################################################################################
        ##########################################################################################################
        ##########################################################################################################
        elif isinstance(input, int) == 1 or isinstance(input, float) == 1:
            element = float(input)
            prefix_string = "{:." + str(number_of_decimal_places) + "f}"
            element_as_string = prefix_string.format(element)

            ##########################################################################################################
            ##########################################################################################################
            if element >= 0:
                element_as_string = element_as_string.zfill(number_of_leading_numbers + number_of_decimal_places + 1 + 1)  # +1 for sign, +1 for decimal place
                element_as_string = "+" + element_as_string  # So that our strings always have either + or - signs to maintain the same string length
            else:
                element_as_string = element_as_string.zfill(number_of_leading_numbers + number_of_decimal_places + 1 + 1 + 1)  # +1 for sign, +1 for decimal place
            ##########################################################################################################
            ##########################################################################################################

            ListOfStringsToJoin.append(element_as_string)
        ##########################################################################################################
        ##########################################################################################################
        ##########################################################################################################

        ##########################################################################################################
        ##########################################################################################################
        ##########################################################################################################
        elif isinstance(input, list) == 1:

            if len(input) > 0:
                for element in input: #RECURSION
                    ListOfStringsToJoin.append(self.ConvertFloatToStringWithNumberOfLeadingNumbersAndDecimalPlaces_NumberOrListInput(element, number_of_leading_numbers, number_of_decimal_places))

            else: #Situation when we get a list() or []
                ListOfStringsToJoin.append(str(input))

        ##########################################################################################################
        ##########################################################################################################
        ##########################################################################################################

        ##########################################################################################################
        ##########################################################################################################
        ##########################################################################################################
        elif isinstance(input, tuple) == 1:

            if len(input) > 0:
                for element in input: #RECURSION
                    ListOfStringsToJoin.append("TUPLE" + self.ConvertFloatToStringWithNumberOfLeadingNumbersAndDecimalPlaces_NumberOrListInput(element, number_of_leading_numbers, number_of_decimal_places))

            else: #Situation when we get a list() or []
                ListOfStringsToJoin.append(str(input))

        ##########################################################################################################
        ##########################################################################################################
        ##########################################################################################################

        ##########################################################################################################
        ##########################################################################################################
        ##########################################################################################################
        elif isinstance(input, dict) == 1:

            if len(input) > 0:
                for Key in input: #RECURSION
                    ListOfStringsToJoin.append(str(Key) + ": " + self.ConvertFloatToStringWithNumberOfLeadingNumbersAndDecimalPlaces_NumberOrListInput(input[Key], number_of_leading_numbers, number_of_decimal_places))

            else: #Situation when we get a dict()
                ListOfStringsToJoin.append(str(input))

        ##########################################################################################################
        ##########################################################################################################
        ##########################################################################################################
        else:
            ListOfStringsToJoin.append(str(input))
        ##########################################################################################################
        ##########################################################################################################
        ##########################################################################################################

        ##########################################################################################################
        ##########################################################################################################
        ##########################################################################################################

        ##########################################################################################################
        ##########################################################################################################
        ##########################################################################################################
        if len(ListOfStringsToJoin) > 1:

            ##########################################################################################################
            ##########################################################################################################

            ##########################################################################################################
            StringToReturn = ""
            for Index, StringToProcess in enumerate(ListOfStringsToJoin):

                ################################################
                if Index == 0: #The first element
                    if StringToProcess.find(":") != -1 and StringToProcess[0] != "{": #meaning that we're processing a dict()
                        StringToReturn = "{"
                    elif StringToProcess.find("TUPLE") != -1 and StringToProcess[0] != "(":  # meaning that we're processing a tuple
                        StringToReturn = "("
                    else:
                        StringToReturn = "["

                    StringToReturn = StringToReturn + StringToProcess.replace("TUPLE","") + ", "
                ################################################

                ################################################
                elif Index < len(ListOfStringsToJoin) - 1: #The middle elements
                    StringToReturn = StringToReturn + StringToProcess + ", "
                ################################################

                ################################################
                else: #The last element
                    StringToReturn = StringToReturn + StringToProcess

                    if StringToProcess.find(":") != -1 and StringToProcess[-1] != "}":  # meaning that we're processing a dict()
                        StringToReturn = StringToReturn + "}"
                    elif StringToProcess.find("TUPLE") != -1 and StringToProcess[-1] != ")":  # meaning that we're processing a tuple
                        StringToReturn = StringToReturn + ")"
                    else:
                        StringToReturn = StringToReturn + "]"

                ################################################

            ##########################################################################################################

            ##########################################################################################################
            ##########################################################################################################

        elif len(ListOfStringsToJoin) == 1:
            StringToReturn = ListOfStringsToJoin[0]

        else:
            StringToReturn = ListOfStringsToJoin

        return StringToReturn
        ##########################################################################################################
        ##########################################################################################################
        ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    ##########################################################################################################
    ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    def ConvertDictToProperlyFormattedStringForPrinting(self, DictToPrint, NumberOfDecimalsPlaceToUse = 3, NumberOfEntriesPerLine = 1, NumberOfTabsBetweenItems = 3):

        ProperlyFormattedStringForPrinting = ""
        ItemsPerLineCounter = 0

        for Key in DictToPrint:

            ##########################################################################################################
            if isinstance(DictToPrint[Key], dict): #RECURSION
                ProperlyFormattedStringForPrinting = ProperlyFormattedStringForPrinting + \
                                                     str(Key) + ":\n" + \
                                                     self.ConvertDictToProperlyFormattedStringForPrinting(DictToPrint[Key], NumberOfDecimalsPlaceToUse, NumberOfEntriesPerLine, NumberOfTabsBetweenItems)

            else:
                ProperlyFormattedStringForPrinting = ProperlyFormattedStringForPrinting + \
                                                     str(Key) + ": " + \
                                                     self.ConvertFloatToStringWithNumberOfLeadingNumbersAndDecimalPlaces_NumberOrListInput(DictToPrint[Key], 0, NumberOfDecimalsPlaceToUse)
            ##########################################################################################################

            ##########################################################################################################
            if ItemsPerLineCounter < NumberOfEntriesPerLine - 1:
                ProperlyFormattedStringForPrinting = ProperlyFormattedStringForPrinting + "\t"*NumberOfTabsBetweenItems
                ItemsPerLineCounter = ItemsPerLineCounter + 1
            else:
                ProperlyFormattedStringForPrinting = ProperlyFormattedStringForPrinting + "\n"
                ItemsPerLineCounter = 0
            ##########################################################################################################

        return ProperlyFormattedStringForPrinting
    ##########################################################################################################
    ##########################################################################################################
