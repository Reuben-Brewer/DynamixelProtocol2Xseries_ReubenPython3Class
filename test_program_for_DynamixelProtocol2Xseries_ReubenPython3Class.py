# -*- coding: utf-8 -*-

'''
Reuben Brewer, Ph.D.
reuben.brewer@gmail.com
www.reubotics.com

Apache 2 License
Software Revision K, 09/12/2024

Verified working on: Python 3.11 for Windows 11 64-bit and Raspberry Pi Buster (no Mac testing yet).
'''

__author__ = 'reuben.brewer'

##########################################
from DynamixelProtocol2Xseries_ReubenPython3Class import *
from MyPrint_ReubenPython2and3Class import *
from MyPlotterPureTkinterStandAloneProcess_ReubenPython2and3Class import *
##########################################

##########################################
import os
import sys
from sys import platform as _platform
import time
import datetime
import collections
import traceback
import re
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

##########################################################################################################
##########################################################################################################
def getPreciseSecondsTimeStampString():
    ts = time.time()

    return ts
##########################################################################################################
##########################################################################################################

##########################################################################################################
##########################################################################################################
def IsInputList(input, print_result_flag = 0):

    result = isinstance(input, list)

    if print_result_flag == 1:
        print("IsInputList: " + str(result))

    return result
##########################################################################################################
##########################################################################################################

##########################################################################################################
##########################################################################################################
##########################################################################################################
##########################################################################################################
def ConvertFloatToStringWithNumberOfLeadingNumbersAndDecimalPlaces_NumberOrListInput(input, number_of_leading_numbers = 4, number_of_decimal_places = 3):

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
                ListOfStringsToJoin.append(ConvertFloatToStringWithNumberOfLeadingNumbersAndDecimalPlaces_NumberOrListInput(element, number_of_leading_numbers, number_of_decimal_places))

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
                ListOfStringsToJoin.append("TUPLE" + ConvertFloatToStringWithNumberOfLeadingNumbersAndDecimalPlaces_NumberOrListInput(element, number_of_leading_numbers, number_of_decimal_places))

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
                ListOfStringsToJoin.append(str(Key) + ": " + ConvertFloatToStringWithNumberOfLeadingNumbersAndDecimalPlaces_NumberOrListInput(input[Key], number_of_leading_numbers, number_of_decimal_places))

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
def ConvertDictToProperlyFormattedStringForPrinting(DictToPrint, NumberOfDecimalsPlaceToUse = 3, NumberOfEntriesPerLine = 1, NumberOfTabsBetweenItems = 3):

    ProperlyFormattedStringForPrinting = ""
    ItemsPerLineCounter = 0

    for Key in DictToPrint:

        if isinstance(DictToPrint[Key], dict): #RECURSION
            ProperlyFormattedStringForPrinting = ProperlyFormattedStringForPrinting + \
                                                 str(Key) + ":\n" + \
                                                 ConvertDictToProperlyFormattedStringForPrinting(DictToPrint[Key], NumberOfDecimalsPlaceToUse, NumberOfEntriesPerLine, NumberOfTabsBetweenItems)

        else:
            ProperlyFormattedStringForPrinting = ProperlyFormattedStringForPrinting + \
                                                 str(Key) + ": " + \
                                                 ConvertFloatToStringWithNumberOfLeadingNumbersAndDecimalPlaces_NumberOrListInput(DictToPrint[Key], 0, NumberOfDecimalsPlaceToUse)

        if ItemsPerLineCounter < NumberOfEntriesPerLine - 1:
            ProperlyFormattedStringForPrinting = ProperlyFormattedStringForPrinting + "\t"*NumberOfTabsBetweenItems
            ItemsPerLineCounter = ItemsPerLineCounter + 1
        else:
            ProperlyFormattedStringForPrinting = ProperlyFormattedStringForPrinting + "\n"
            ItemsPerLineCounter = 0

    return ProperlyFormattedStringForPrinting
##########################################################################################################
##########################################################################################################

##########################################################################################################
##########################################################################################################
##########################################################################################################
##########################################################################################################
def GUI_update_clock():
    global root
    global EXIT_PROGRAM_FLAG
    global GUI_RootAfterCallbackInterval_Milliseconds
    global USE_GUI_FLAG

    global DynamixelProtocol2Xseries_OPEN_FLAG
    global DynamixelProtocol2Xseries_Object
    global DynamixelProtocol2Xseries_MostRecentDict
    global USE_DynamixelProtocol2Xseries_FLAG
    global SHOW_IN_GUI_DynamixelProtocol2Xseries_FLAG

    global Data_Label

    ##########################################################################################################
    ##########################################################################################################
    ##########################################################################################################
    if USE_GUI_FLAG == 1:

        ##########################################################################################################
        ##########################################################################################################
        if EXIT_PROGRAM_FLAG == 0:

            ##########################################################################################################
            Data_Label_TextToDisplay = ConvertDictToProperlyFormattedStringForPrinting(DynamixelProtocol2Xseries_MostRecentDict,
                                                                                       NumberOfDecimalsPlaceToUse=5,
                                                                                       NumberOfEntriesPerLine=1,
                                                                                       NumberOfTabsBetweenItems=1)
            Data_Label["text"] = Data_Label_TextToDisplay
            ##########################################################################################################

            ##########################################################################################################
            if USE_DynamixelProtocol2Xseries_FLAG == 1 and SHOW_IN_GUI_DynamixelProtocol2Xseries_FLAG == 1 and DynamixelProtocol2Xseries_OPEN_FLAG == 1:
                DynamixelProtocol2Xseries_Object.GUI_update_clock()
            ##########################################################################################################

            ##########################################################################################################
            if MYPRINT_OPEN_FLAG == 1 and SHOW_IN_GUI_MYPRINT_FLAG == 1:
                MyPrint_ReubenPython2and3ClassObject.GUI_update_clock()
            ##########################################################################################################

            ##########################################################################################################
            root.after(GUI_RootAfterCallbackInterval_Milliseconds, GUI_update_clock)
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
def ExitProgram_Callback():
    global EXIT_PROGRAM_FLAG

    print("ExitProgram_Callback event fired!")

    EXIT_PROGRAM_FLAG = 1
##########################################################################################################
##########################################################################################################

##########################################################################################################
##########################################################################################################
##########################################################################################################
def GUI_Thread():
    global root
    global root_Xpos
    global root_Ypos
    global root_width
    global root_height
    global GUI_RootAfterCallbackInterval_Milliseconds
    global USE_TABS_IN_GUI_FLAG

    ########################################################################################################## KEY GUI LINE
    ##########################################################################################################
    root = Tk()
    ##########################################################################################################
    ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    global TabControlObject
    global Tab_MainControls
    global Tab_DynamixelProtocol2Xseries
    global Tab_MyPrint

    if USE_TABS_IN_GUI_FLAG == 1:
        ##########################################################################################################
        TabControlObject = ttk.Notebook(root)

        Tab_DynamixelProtocol2Xseries = ttk.Frame(TabControlObject)
        TabControlObject.add(Tab_DynamixelProtocol2Xseries, text='   Dynamixel   ')

        Tab_MainControls = ttk.Frame(TabControlObject)
        TabControlObject.add(Tab_MainControls, text='   Main Controls   ')

        Tab_MyPrint = ttk.Frame(TabControlObject)
        TabControlObject.add(Tab_MyPrint, text='   MyPrint Terminal   ')

        TabControlObject.pack(expand=1, fill="both")  # CANNOT MIX PACK AND GRID IN THE SAME FRAME/TAB, SO ALL .GRID'S MUST BE CONTAINED WITHIN THEIR OWN FRAME/TAB.

        ############# #Set the tab header font
        TabStyle = ttk.Style()
        TabStyle.configure('TNotebook.Tab', font=('Helvetica', '12', 'bold'))
        #############

        ##########################################################################################################
    else:
        ##########################################################################################################
        Tab_MainControls = root
        Tab_DynamixelProtocol2Xseries = root
        Tab_MyPrint = root
        ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    global Data_Label
    Data_Label = Label(Tab_MainControls, text="Data_Label", width=150)
    Data_Label.grid(row=1, column=0, padx=1, pady=1, columnspan=1, rowspan=1)
    ##########################################################################################################
    ##########################################################################################################

    ########################################################################################################## THIS BLOCK MUST COME 2ND-TO-LAST IN def GUI_Thread() IF USING TABS.
    ##########################################################################################################
    root.protocol("WM_DELETE_WINDOW", ExitProgram_Callback)  # Set the callback function for when the window's closed.
    root.title("test_program_for_DynamixelProtocol2Xseries_ReubenPython3Class")
    root.geometry('%dx%d+%d+%d' % (root_width, root_height, root_Xpos, root_Ypos)) # set the dimensions of the screen and where it is placed
    root.after(GUI_RootAfterCallbackInterval_Milliseconds, GUI_update_clock)
    root.mainloop()
    ##########################################################################################################
    ##########################################################################################################

    ##########################################################################################################  THIS BLOCK MUST COME LAST IN def GUI_Thread() REGARDLESS OF CODE.
    ##########################################################################################################
    root.quit() #Stop the GUI thread, MUST BE CALLED FROM GUI_Thread
    root.destroy() #Close down the GUI thread, MUST BE CALLED FROM GUI_Thread
    ##########################################################################################################
    ##########################################################################################################

##########################################################################################################
##########################################################################################################

##########################################################################################################
##########################################################################################################
##########################################################################################################
##########################################################################################################
if __name__ == '__main__':

    ##########################################################################################################
    ##########################################################################################################
    ##########################################################################################################
    global my_platform

    if platform.system() == "Linux":

        if "raspberrypi" in platform.uname():  # os.uname() doesn't work in windows
            my_platform = "pi"
        else:
            my_platform = "linux"

    elif platform.system() == "Windows":
        my_platform = "windows"

    elif platform.system() == "Darwin":
        my_platform = "mac"

    else:
        my_platform = "other"

    print("The OS platform is: " + my_platform)
    ##########################################################################################################
    ##########################################################################################################
    ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    ##########################################################################################################
    global USE_GUI_FLAG
    USE_GUI_FLAG = 1

    global USE_TABS_IN_GUI_FLAG
    USE_TABS_IN_GUI_FLAG = 1

    global USE_DynamixelProtocol2Xseries_FLAG
    USE_DynamixelProtocol2Xseries_FLAG = 1

    global USE_MyPrint_FLAG
    USE_MyPrint_FLAG = 1

    global USE_PLOTTER_FLAG
    USE_PLOTTER_FLAG = 1

    global USE_SINUSOIDAL_POS_CONTROL_INPUT_FLAG
    USE_SINUSOIDAL_POS_CONTROL_INPUT_FLAG = 0 #unicorn
    ##########################################################################################################
    ##########################################################################################################
    ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    ##########################################################################################################
    global SHOW_IN_GUI_DynamixelProtocol2Xseries_FLAG
    SHOW_IN_GUI_DynamixelProtocol2Xseries_FLAG = 1

    global SHOW_IN_GUI_MYPRINT_FLAG
    SHOW_IN_GUI_MYPRINT_FLAG = 1
    ##########################################################################################################
    ##########################################################################################################
    ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    ##########################################################################################################
    global GUI_ROW_DynamixelProtocol2Xseries
    global GUI_COLUMN_DynamixelProtocol2Xseries
    global GUI_PADX_DynamixelProtocol2Xseries
    global GUI_PADY_DynamixelProtocol2Xseries
    global GUI_ROWSPAN_DynamixelProtocol2Xseries
    global GUI_COLUMNSPAN_DynamixelProtocol2Xseries
    GUI_ROW_DynamixelProtocol2Xseries = 0

    GUI_COLUMN_DynamixelProtocol2Xseries = 0
    GUI_PADX_DynamixelProtocol2Xseries = 1
    GUI_PADY_DynamixelProtocol2Xseries = 1
    GUI_ROWSPAN_DynamixelProtocol2Xseries = 1
    GUI_COLUMNSPAN_DynamixelProtocol2Xseries = 1

    global GUI_ROW_MYPRINT
    global GUI_COLUMN_MYPRINT
    global GUI_PADX_MYPRINT
    global GUI_PADY_MYPRINT
    global GUI_ROWSPAN_MYPRINT
    global GUI_COLUMNSPAN_MYPRINT
    GUI_ROW_MYPRINT = 1

    GUI_COLUMN_MYPRINT = 0
    GUI_PADX_MYPRINT = 1
    GUI_PADY_MYPRINT = 1
    GUI_ROWSPAN_MYPRINT = 1
    GUI_COLUMNSPAN_MYPRINT = 1
    ##########################################################################################################
    ##########################################################################################################
    ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    ##########################################################################################################
    global EXIT_PROGRAM_FLAG
    EXIT_PROGRAM_FLAG = 0

    global CurrentTime_MainLoopThread
    CurrentTime_MainLoopThread = -11111.0

    global StartingTime_MainLoopThread
    StartingTime_MainLoopThread = -11111.0

    global root

    global root_Xpos
    root_Xpos = 900

    global root_Ypos
    root_Ypos = 0

    global root_width
    root_width = 1920 - root_Xpos

    global root_height
    root_height = 1020 - root_Ypos

    global TabControlObject
    global Tab_MainControls
    global Tab_DynamixelProtocol2Xseries
    global Tab_MyPrint

    global GUI_RootAfterCallbackInterval_Milliseconds
    GUI_RootAfterCallbackInterval_Milliseconds = 30

    global SINUSOIDAL_MOTION_INPUT_ROMtestTimeToPeakAngle
    SINUSOIDAL_MOTION_INPUT_ROMtestTimeToPeakAngle = 3.0

    global SINUSOIDAL_MOTION_INPUT_MinValue_PositionControl_0  # unicorn
    SINUSOIDAL_MOTION_INPUT_MinValue_PositionControl_0 = -400.0

    global SINUSOIDAL_MOTION_INPUT_MaxValue_PositionControl_0
    SINUSOIDAL_MOTION_INPUT_MaxValue_PositionControl_0 = 800.0

    global SINUSOIDAL_MOTION_INPUT_MinValue_PositionControl_1
    SINUSOIDAL_MOTION_INPUT_MinValue_PositionControl_1 = -400.0

    global SINUSOIDAL_MOTION_INPUT_MaxValue_PositionControl_1
    SINUSOIDAL_MOTION_INPUT_MaxValue_PositionControl_1 = 800.0

    global SINUSOIDAL_MOTION_INPUT_MinValue_VelocityControl
    SINUSOIDAL_MOTION_INPUT_MinValue_VelocityControl = -1.0

    global SINUSOIDAL_MOTION_INPUT_MaxValue_VelocityControl
    SINUSOIDAL_MOTION_INPUT_MaxValue_VelocityControl = 1.0
    ##########################################################################################################
    ##########################################################################################################
    ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    ##########################################################################################################
    global DynamixelProtocol2Xseries_Object

    global DynamixelProtocol2Xseries_OPEN_FLAG
    DynamixelProtocol2Xseries_OPEN_FLAG = -1

    global DynamixelProtocol2Xseries_TestChannelsList
    DynamixelProtocol2Xseries_TestChannelsList = [0, 1] #Set this list to whichever motor ID's you want to test.

    global DynamixelProtocol2Xseries_SinusoidalMotionChannelsList
    DynamixelProtocol2Xseries_SinusoidalMotionChannelsList = [0, 1],

    global DynamixelProtocol2Xseries_MostRecentDict
    DynamixelProtocol2Xseries_MostRecentDict = dict()

    global DynamixelProtocol2Xseries_PresentPositionDegreesList
    DynamixelProtocol2Xseries_PresentPositionDegreesList = [0.0 ,0.0]

    global DynamixelProtocol2Xseries_PresentCurrentList
    DynamixelProtocol2Xseries_PresentCurrentList = [0.0 ,0.0]
    ##########################################################################################################
    ##########################################################################################################
    ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    ##########################################################################################################
    global MyPrint_ReubenPython2and3ClassObject

    global MYPRINT_OPEN_FLAG
    MYPRINT_OPEN_FLAG = -1
    ##########################################################################################################
    ##########################################################################################################
    ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    ##########################################################################################################
    global MyPlotterPureTkinterStandAloneProcess_ReubenPython2and3ClassObject

    global PLOTTER_OPEN_FLAG
    PLOTTER_OPEN_FLAG = -1

    global MyPlotterPureTkinter_MostRecentDict
    MyPlotterPureTkinter_MostRecentDict = dict()

    global MyPlotterPureTkinterStandAloneProcess_ReubenPython2and3ClassObject_MostRecentDict_StandAlonePlottingProcess_ReadyForWritingFlag
    MyPlotterPureTkinterStandAloneProcess_ReubenPython2and3ClassObject_MostRecentDict_StandAlonePlottingProcess_ReadyForWritingFlag = -1

    global LastTime_MainLoopThread_PLOTTER
    LastTime_MainLoopThread_PLOTTER = -11111.0
    ##########################################################################################################
    ##########################################################################################################
    ##########################################################################################################

    ########################################################################################################## KEY GUI LINE
    ##########################################################################################################
    ##########################################################################################################
    if USE_GUI_FLAG == 1:
        print("Starting GUI thread...")
        GUI_Thread_ThreadingObject = threading.Thread(target=GUI_Thread)
        GUI_Thread_ThreadingObject.setDaemon(True) #Should mean that the GUI thread is destroyed automatically when the main thread is destroyed.
        GUI_Thread_ThreadingObject.start()
        time.sleep(0.5)  #Allow enough time for 'root' to be created that we can then pass it into other classes.
    else:
        root = None
        Tab_MainControls = None
        Tab_DynamixelProtocol2Xseries = None
        Tab_MyPrint = None
    ##########################################################################################################
    ##########################################################################################################
    ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    ##########################################################################################################
    global DynamixelProtocol2Xseries_GUIparametersDict
    DynamixelProtocol2Xseries_GUIparametersDict = dict([("USE_GUI_FLAG", USE_GUI_FLAG and SHOW_IN_GUI_DynamixelProtocol2Xseries_FLAG),
                                    ("root", Tab_DynamixelProtocol2Xseries),
                                    ("EnableInternal_MyPrint_Flag", 1),
                                    ("NumberOfPrintLines", 10),
                                    ("UseBorderAroundThisGuiObjectFlag", 0),
                                    ("GUI_ROW", GUI_ROW_DynamixelProtocol2Xseries),
                                    ("GUI_COLUMN", GUI_COLUMN_DynamixelProtocol2Xseries),
                                    ("GUI_PADX", GUI_PADX_DynamixelProtocol2Xseries),
                                    ("GUI_PADY", GUI_PADY_DynamixelProtocol2Xseries),
                                    ("GUI_ROWSPAN", GUI_ROWSPAN_DynamixelProtocol2Xseries),
                                    ("GUI_COLUMNSPAN", GUI_COLUMNSPAN_DynamixelProtocol2Xseries)])

    global DynamixelProtocol2Xseries_setup_dict
    DynamixelProtocol2Xseries_setup_dict = dict([("GUIparametersDict", DynamixelProtocol2Xseries_GUIparametersDict),
                                ("DesiredSerialNumber_USBtoSerialConverter", "FT94VSKV"), #Change to the serial number of your unique device, FT89FE7OA FT891KTUA
                                ("NameToDisplay_UserSet", "Example Name U2D2"),
                                ("SerialBaudRate", 4000000),
                                ("WatchdogTimeIntervalMilliseconds", 100.0),
                                ("EnableSafetyShutoff", 0),
                                ("GetVariablesEveryNloopsCycles", 3),
                                ("ENABLE_SETS", 1),
                                ("MainThread_TimeToSleepEachLoop", 0.002),
                                ("MotorType_StringList", ["XC330-288-T"]*len(DynamixelProtocol2Xseries_TestChannelsList)), #EACH INPUT LIST MUST BE THE SAME LENGTH AS NUMBER OF MOTORS. XC330-288-T
                                ("MotorName_StringList", ["Large", "Small"]),
                                ("ControlType_StartingValueList", ["ExtendedPositionControlMultiTurn"]*len(DynamixelProtocol2Xseries_TestChannelsList)), #MOTOR ID'S MUST BE IN ORDER FROM 0 T0 (NumberOfMotors - 1) (E.G. FOR 3 MOTORS, THE ID'S WOULD BE 0, 1, AND 2).
                                ("Position_Deg_StartingValueList", [123.0, 456.0]),#[(SINUSOIDAL_MOTION_INPUT_MinValue_PositionControl_0 + SINUSOIDAL_MOTION_INPUT_MaxValue_PositionControl_0)/2.0, (SINUSOIDAL_MOTION_INPUT_MinValue_PositionControl_1 + SINUSOIDAL_MOTION_INPUT_MaxValue_PositionControl_1)/2.0]),
                                ("Position_Deg_min", [SINUSOIDAL_MOTION_INPUT_MinValue_PositionControl_0, SINUSOIDAL_MOTION_INPUT_MinValue_PositionControl_1]),
                                ("Position_Deg_max", [SINUSOIDAL_MOTION_INPUT_MaxValue_PositionControl_0, SINUSOIDAL_MOTION_INPUT_MaxValue_PositionControl_1]),
                                ("Velocity_DynamixelUnits_StartingValueList", [0.0] * len(DynamixelProtocol2Xseries_TestChannelsList)),
                                ("Velocity_DynamixelUnits_min", [0.0] * len(DynamixelProtocol2Xseries_TestChannelsList)),
                                ("Velocity_DynamixelUnits_max", [60.0] * len(DynamixelProtocol2Xseries_TestChannelsList)),
                                ("Current_DynamixelUnits_StartingValueList", [390.0] * len(DynamixelProtocol2Xseries_TestChannelsList)),
                                ("StartEngagedFlag", [0]*len(DynamixelProtocol2Xseries_TestChannelsList)),
                                ("ListOfVariableNameStringsToGet", ["PresentPosition", "PresentCurrent", "PresentInputVoltage", "PresentTemperature","CurrentLimit", "HardwareErrorStatus", "Shutdown"])])

    if USE_DynamixelProtocol2Xseries_FLAG == 1:
        try:
            DynamixelProtocol2Xseries_Object = DynamixelProtocol2Xseries_ReubenPython3Class(DynamixelProtocol2Xseries_setup_dict)
            DynamixelProtocol2Xseries_OPEN_FLAG = DynamixelProtocol2Xseries_Object.OBJECT_CREATED_SUCCESSFULLY_FLAG

        except:
            exceptions = sys.exc_info()[0]
            print("DynamixelProtocol2Xseries_Object, exceptions: %s" % exceptions)
            traceback.print_exc()
    ##########################################################################################################
    ##########################################################################################################
    ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    ##########################################################################################################
    if USE_MyPrint_FLAG == 1:

        MyPrint_ReubenPython2and3ClassObject_GUIparametersDict = dict([("USE_GUI_FLAG", USE_GUI_FLAG and SHOW_IN_GUI_MYPRINT_FLAG),
                                                                        ("root", Tab_MyPrint),
                                                                        ("UseBorderAroundThisGuiObjectFlag", 0),
                                                                        ("GUI_ROW", GUI_ROW_MYPRINT),
                                                                        ("GUI_COLUMN", GUI_COLUMN_MYPRINT),
                                                                        ("GUI_PADX", GUI_PADX_MYPRINT),
                                                                        ("GUI_PADY", GUI_PADY_MYPRINT),
                                                                        ("GUI_ROWSPAN", GUI_ROWSPAN_MYPRINT),
                                                                        ("GUI_COLUMNSPAN", GUI_COLUMNSPAN_MYPRINT)])

        MyPrint_ReubenPython2and3ClassObject_setup_dict = dict([("NumberOfPrintLines", 10),
                                                                ("WidthOfPrintingLabel", 200),
                                                                ("PrintToConsoleFlag", 1),
                                                                ("LogFileNameFullPath", os.getcwd() + "//TestLog.txt"),
                                                                ("GUIparametersDict", MyPrint_ReubenPython2and3ClassObject_GUIparametersDict)])

        try:
            MyPrint_ReubenPython2and3ClassObject = MyPrint_ReubenPython2and3Class(MyPrint_ReubenPython2and3ClassObject_setup_dict)
            MYPRINT_OPEN_FLAG = MyPrint_ReubenPython2and3ClassObject.OBJECT_CREATED_SUCCESSFULLY_FLAG

        except:
            exceptions = sys.exc_info()[0]
            print("MyPrint_ReubenPython2and3ClassObject __init__: Exceptions: %s" % exceptions)
            traceback.print_exc()
    ##########################################################################################################
    ##########################################################################################################
    ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    ##########################################################################################################
    global MyPlotterPureTkinterStandAloneProcess_ReubenPython2and3ClassObject_GUIparametersDict
    MyPlotterPureTkinterStandAloneProcess_ReubenPython2and3ClassObject_GUIparametersDict = dict([("EnableInternal_MyPrint_Flag", 1),
                                                                                                ("NumberOfPrintLines", 10),
                                                                                                ("UseBorderAroundThisGuiObjectFlag", 0),
                                                                                                ("GraphCanvasWidth", 890),
                                                                                                ("GraphCanvasHeight", 700),
                                                                                                ("GraphCanvasWindowStartingX", 0),
                                                                                                ("GraphCanvasWindowStartingY", 0),
                                                                                                ("GUI_RootAfterCallbackInterval_Milliseconds_IndependentOfParentRootGUIloopEvents", 20)])

    global MyPlotterPureTkinterStandAloneProcess_ReubenPython2and3ClassObject_setup_dict
    MyPlotterPureTkinterStandAloneProcess_ReubenPython2and3ClassObject_setup_dict = dict([("GUIparametersDict", MyPlotterPureTkinterStandAloneProcess_ReubenPython2and3ClassObject_GUIparametersDict),
                                                                                        ("ParentPID", os.getpid()),
                                                                                        ("WatchdogTimerExpirationDurationSeconds_StandAlonePlottingProcess", 0.0),
                                                                                        ("MarkerSize", 3),
                                                                                        ("CurvesToPlotNamesAndColorsDictOfLists", dict([("NameList", ["Channel0", "Channel1", "Channel2", "Channel3"]),("ColorList", ["Red", "Green", "Blue", "Black"])])),
                                                                                        ("NumberOfDataPointToPlot", 100),
                                                                                        ("XaxisNumberOfTickMarks", 10),
                                                                                        ("YaxisNumberOfTickMarks", 10),
                                                                                        ("XaxisNumberOfDecimalPlacesForLabels", 3),
                                                                                        ("YaxisNumberOfDecimalPlacesForLabels", 3),
                                                                                        ("XaxisAutoscaleFlag", 1),
                                                                                        ("YaxisAutoscaleFlag", 1),
                                                                                        ("X_min", 0.0),
                                                                                        ("X_max", 20.0),
                                                                                        ("Y_min", -0.0015),
                                                                                        ("Y_max", 0.0015),
                                                                                        ("XaxisDrawnAtBottomOfGraph", 0),
                                                                                        ("XaxisLabelString", "Time (sec)"),
                                                                                        ("YaxisLabelString", "Y-units (units)"),
                                                                                        ("ShowLegendFlag", 1)])

    if USE_PLOTTER_FLAG == 1:
        try:
            MyPlotterPureTkinterStandAloneProcess_ReubenPython2and3ClassObject = MyPlotterPureTkinterStandAloneProcess_ReubenPython2and3Class(MyPlotterPureTkinterStandAloneProcess_ReubenPython2and3ClassObject_setup_dict)
            PLOTTER_OPEN_FLAG = MyPlotterPureTkinterStandAloneProcess_ReubenPython2and3ClassObject.OBJECT_CREATED_SUCCESSFULLY_FLAG

        except:
            exceptions = sys.exc_info()[0]
            print("MyPlotterPureTkinterStandAloneProcess_ReubenPython2and3ClassObject, exceptions: %s" % exceptions)
            traceback.print_exc()
    ##########################################################################################################
    ##########################################################################################################
    ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    ##########################################################################################################
    if USE_DynamixelProtocol2Xseries_FLAG == 1 and DynamixelProtocol2Xseries_OPEN_FLAG != 1:
        print("Failed to open DynamixelProtocol2Xseries_ReubenPython3Class.")
        ExitProgram_Callback()
    ##########################################################################################################
    ##########################################################################################################
    ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    ##########################################################################################################
    if USE_MyPrint_FLAG == 1 and MYPRINT_OPEN_FLAG != 1:
        print("Failed to open MyPrint_ReubenPython2and3ClassObject.")
        ExitProgram_Callback()
    ##########################################################################################################
    ##########################################################################################################
    ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    ##########################################################################################################
    if USE_PLOTTER_FLAG == 1 and PLOTTER_OPEN_FLAG != 1:
        print("Failed to open MyPlotterPureTkinterClass_Object.")
        ExitProgram_Callback()
    ##########################################################################################################
    ##########################################################################################################
    ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    ##########################################################################################################
    print("Starting main loop 'test_program_for_DynamixelProtocol1AXorMXseries.py'")

    StartingTime_MainLoopThread = getPreciseSecondsTimeStampString()
    LoopCounter = 0

    while(EXIT_PROGRAM_FLAG == 0):

        ##########################################################################################################
        ##########################################################################################################
        CurrentTime_MainLoopThread = getPreciseSecondsTimeStampString() - StartingTime_MainLoopThread
        ##########################################################################################################
        ##########################################################################################################

        ########################################################################################################## GET's
        ##########################################################################################################
        if USE_DynamixelProtocol2Xseries_FLAG == 1:

            DynamixelProtocol2Xseries_MostRecentDict = DynamixelProtocol2Xseries_Object.GetMostRecentDataDict(OverrideLengthMatchingRequirementFlag=1)
            #print("DynamixelProtocol2Xseries_MostRecentDict: " + str(DynamixelProtocol2Xseries_MostRecentDict))

            if "Time" in DynamixelProtocol2Xseries_MostRecentDict:
                try:
                    DynamixelProtocol2Xseries_PresentPositionDegreesList = DynamixelProtocol2Xseries_MostRecentDict["PresentPosition_Degrees"]
                    #print("DynamixelProtocol2Xseries_PresentPositionDegreesList: " + str(DynamixelProtocol2Xseries_PresentPositionDegreesList))
                except:
                    pass

                try:
                    DynamixelProtocol2Xseries_PresentCurrentList = DynamixelProtocol2Xseries_MostRecentDict["PresentCurrent"]
                    #print("DynamixelProtocol2Xseries_PresentCurrentList: " + str(DynamixelProtocol2Xseries_PresentCurrentList))
                except:
                    pass


        ##########################################################################################################
        ##########################################################################################################

        ########################################################################################################## SET's
        ##########################################################################################################
        if USE_DynamixelProtocol2Xseries_FLAG == 1:

            time_gain = math.pi / (2.0 * SINUSOIDAL_MOTION_INPUT_ROMtestTimeToPeakAngle)

            if LoopCounter == 500:
                for DynamixelChannel in range(0, len(DynamixelProtocol2Xseries_TestChannelsList)):
                    if DynamixelChannel in DynamixelProtocol2Xseries_TestChannelsList:
                        DynamixelProtocol2Xseries_Object.SetEngagedState_FROM_EXTERNAL_PROGRAM(DynamixelChannel, 1)
                        print("Motor " + str(DynamixelChannel) + "engaged!")

            if USE_SINUSOIDAL_POS_CONTROL_INPUT_FLAG == 1:

                SINUSOIDAL_INPUT_TO_COMMAND_0 = (SINUSOIDAL_MOTION_INPUT_MaxValue_PositionControl_0 + SINUSOIDAL_MOTION_INPUT_MinValue_PositionControl_0)/2.0 + 0.5*abs(SINUSOIDAL_MOTION_INPUT_MaxValue_PositionControl_0 - SINUSOIDAL_MOTION_INPUT_MinValue_PositionControl_0)*math.sin(time_gain*CurrentTime_MainLoopThread)
                SINUSOIDAL_INPUT_TO_COMMAND_1 = (SINUSOIDAL_MOTION_INPUT_MaxValue_PositionControl_1 + SINUSOIDAL_MOTION_INPUT_MinValue_PositionControl_1)/2.0 + 0.5*abs(SINUSOIDAL_MOTION_INPUT_MaxValue_PositionControl_1 - SINUSOIDAL_MOTION_INPUT_MinValue_PositionControl_1)*math.sin(math.pi/2.0 + time_gain*CurrentTime_MainLoopThread)

                for DynamixelChannel in range(0, len(DynamixelProtocol2Xseries_TestChannelsList)):
                    if DynamixelChannel in DynamixelProtocol2Xseries_TestChannelsList:
                        if DynamixelChannel == 0:
                            DynamixelProtocol2Xseries_Object.SetPosition_FROM_EXTERNAL_PROGRAM(DynamixelChannel, SINUSOIDAL_INPUT_TO_COMMAND_0, "deg")

                        elif DynamixelChannel == 1:
                            DynamixelProtocol2Xseries_Object.SetPosition_FROM_EXTERNAL_PROGRAM(DynamixelChannel, SINUSOIDAL_INPUT_TO_COMMAND_1, "deg")

                else:
                    SINUSOIDAL_INPUT_TO_COMMAND = (SINUSOIDAL_MOTION_INPUT_MaxValue_VelocityControl + SINUSOIDAL_MOTION_INPUT_MinValue_VelocityControl)/2.0 + 0.5*abs(SINUSOIDAL_MOTION_INPUT_MaxValue_VelocityControl - SINUSOIDAL_MOTION_INPUT_MinValue_VelocityControl)*math.sin(time_gain*CurrentTime_MainLoopThread)

                for DynamixelChannel in range(0, len(DynamixelProtocol2Xseries_TestChannelsList)):
                    if DynamixelChannel in DynamixelProtocol2Xseries_TestChannelsList:
                        DynamixelProtocol2Xseries_Object.SetVelocity_FROM_EXTERNAL_PROGRAM(DynamixelChannel, SINUSOIDAL_INPUT_TO_COMMAND)

        else:
            pass
        ##########################################################################################################
        ##########################################################################################################

        ##########################################################################################################
        ##########################################################################################################
        if PLOTTER_OPEN_FLAG == 1:

            ##########################################################################################################
            MyPlotterPureTkinterStandAloneProcess_ReubenPython2and3ClassObject_MostRecentDict = MyPlotterPureTkinterStandAloneProcess_ReubenPython2and3ClassObject.GetMostRecentDataDict()

            if "StandAlonePlottingProcess_ReadyForWritingFlag" in MyPlotterPureTkinterStandAloneProcess_ReubenPython2and3ClassObject_MostRecentDict:
                MyPlotterPureTkinterStandAloneProcess_ReubenPython2and3ClassObject_MostRecentDict_StandAlonePlottingProcess_ReadyForWritingFlag = MyPlotterPureTkinterStandAloneProcess_ReubenPython2and3ClassObject_MostRecentDict["StandAlonePlottingProcess_ReadyForWritingFlag"]

                if MyPlotterPureTkinterStandAloneProcess_ReubenPython2and3ClassObject_MostRecentDict_StandAlonePlottingProcess_ReadyForWritingFlag == 1:
                    if CurrentTime_MainLoopThread - LastTime_MainLoopThread_PLOTTER >= 0.030:
                        MyPlotterPureTkinterStandAloneProcess_ReubenPython2and3ClassObject.ExternalAddPointOrListOfPointsToPlot(["Channel0"], [CurrentTime_MainLoopThread]*1, [DynamixelProtocol2Xseries_PresentPositionDegreesList[0]])
                        LastTime_MainLoopThread_PLOTTER = CurrentTime_MainLoopThread
            ##########################################################################################################

        ##########################################################################################################
        ##########################################################################################################

        ##########################################################################################################
        ##########################################################################################################
        LoopCounter = LoopCounter + 1
        time.sleep(0.005) ######## MUST HAVE AT LEAST A SMALL TIMEOUT OR ELSE THE MOTORS RUNS AWAY
        ##########################################################################################################
        ##########################################################################################################

    #################################################
    #################################################

    ########################################################################################################## THIS IS THE EXIT ROUTINE!
    ##########################################################################################################
    ##########################################################################################################

    print("Exiting main program 'test_program_for_DynamixelProtocol2Xseries_ReubenPython3Class.")

    ##########################################################################################################
    ##########################################################################################################
    if DynamixelProtocol2Xseries_OPEN_FLAG == 1:
        DynamixelProtocol2Xseries_Object.ExitProgram_Callback()
    ##########################################################################################################
    ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    if MYPRINT_OPEN_FLAG == 1:
        MyPrint_ReubenPython2and3ClassObject.ExitProgram_Callback()
    ##########################################################################################################
    ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    if PLOTTER_OPEN_FLAG == 1:
        MyPlotterPureTkinterStandAloneProcess_ReubenPython2and3ClassObject.ExitProgram_Callback()
    ##########################################################################################################
    ##########################################################################################################

    ##########################################################################################################
    ##########################################################################################################
    ##########################################################################################################

##########################################################################################################
##########################################################################################################
##########################################################################################################
##########################################################################################################