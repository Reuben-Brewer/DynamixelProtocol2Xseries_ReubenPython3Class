echo "Reuben Brewer, Ph.D., reuben.brewer@gmail.com, www.reubotics.com"
echo "Apache 2 License, Software Revision D, 03/13/2022"
echo "Verified working on Raspberry Pi Buster"

echo "Running script: $0" #First argv is the namne of the shell script

NumberOfARGVarguments=$#
echo "NumberOfARGVarguments = $#"

if [ $NumberOfARGVarguments -lt 1 ]; then
	echo "You should have at least 1 argument (latency_timer). Defaulting to 1ms"
	#echo "Press any key to exit"
	#read varname
	#exit
	LatencyTimerToSet=1 #Hard-code to 1ms
else
	LatencyTimerToSet=$1 #2nd argv is the first input to the shell command

fi


echo "LatencyTimerToSet = $LatencyTimerToSet"

echo "BEFORE USB0 latency_timer: $(cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer)"
echo "BEFORE USB1 latency_timer: $(cat /sys/bus/usb-serial/devices/ttyUSB1/latency_timer)"
echo "BEFORE USB2 latency_timer: $(cat /sys/bus/usb-serial/devices/ttyUSB2/latency_timer)"
echo "BEFORE USB3 latency_timer: $(cat /sys/bus/usb-serial/devices/ttyUSB3/latency_timer)"

sudo echo $LatencyTimerToSet > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
sudo echo $LatencyTimerToSet > /sys/bus/usb-serial/devices/ttyUSB1/latency_timer
sudo echo $LatencyTimerToSet > /sys/bus/usb-serial/devices/ttyUSB2/latency_timer
sudo echo $LatencyTimerToSet > /sys/bus/usb-serial/devices/ttyUSB3/latency_timer

echo "AFTER USB0 latency_timer: $(cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer)"
echo "AFTER USB1 latency_timer: $(cat /sys/bus/usb-serial/devices/ttyUSB1/latency_timer)"
echo "AFTER USB2 latency_timer: $(cat /sys/bus/usb-serial/devices/ttyUSB2/latency_timer)"
echo "AFTER USB3 latency_timer: $(cat /sys/bus/usb-serial/devices/ttyUSB3/latency_timer)"

echo "Press any key to exit"
read varname

exit
