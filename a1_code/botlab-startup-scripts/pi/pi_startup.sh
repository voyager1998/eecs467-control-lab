#!/bin/bash

SCRIPT_DIR="/home/pi/botlab-f19"

PASS="i<3robots!"
LOG_PATH="/home/pi/log"
LOG="$LOG_PATH/pi_startup.log"
OLED_LOG="$LOG_PATH/oled.log"
LIDAR_LOG="$LOG_PATH/lidar.log"

date > $LOG
date > $OLED_LOG
date > $LIDAR_LOG

_term() {
	echo "SIGTERM!" >> $LOG
	exit
}

trap _term SIGTERM


#echo $PASS | sudo -S ifconfig lo multicast
#echo $PASS | sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo

cd /home/pi/botlab-startup-scripts/pi
./stats.py &>> $OLED_LOG &

for i in {1..10}
do


	ping -c 1 -W 1 192.168.4.2 >/dev/null
	if [ $? -eq 0 ]
	then
		echo "connection success" >>$LOG
		break
	else
		echo "waiting for beaglebone network" >>$LOG
	fi

	sleep 3
done

ping -c 1 -W 1 192.168.4.2 >/dev/null

if [ $? -ne 0 ]
then
	echo "failed to connect after 10 tries" >>$LOG
	exit
fi

#set up multicast
echo "setting up multicast with beaglebone" >>$LOG
echo $PASS | sudo -S ifconfig eth0 multicast
echo $PASS | sudo -S route add -net 224.0.0.0 netmask 240.0.0.0 dev eth0

#source setenv.sh

$SCRIPT_DIR/bin/timesync &>> $LOG &
$SCRIPT_DIR/bin/rplidar_driver &>> $LIDAR_LOG &

# Add other programs to autostart here
#$SCRIPT_DIR/bin/slam &>> $LOG &

echo "start lcm tunnel " >> $LOG &
bot-lcm-tunnel &>> /home/pi/log/tunnel_log.txt &>>$LOG &
