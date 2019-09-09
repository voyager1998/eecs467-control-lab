#!/usr/bin/env bash

BOTTOMDIR="/home/debian/mobilebot-f19"

PASS="temppwd"
LOG="/home/debian/bbstartuplog.txt"

> $LOG
date &>> $LOG

echo "configuring lcm multicast"

for i in {1..20} #attempt to ping raspberry pi 20 times
do

	ping -c 1 -W 1 192.168.4.1 >/dev/null
	if [ $? -eq 0 ]
	then 
		echo "connection success" &>>$LOG
		break
	else
		echo "waiting for raspberry pi eth0" &>>$LOG

		if [ $i -eq 20 ]
		then
			echo "unable to ping raspberry pi after 20 tries" &>>$LOG
			exit 1
		fi
	fi

	sleep 3
done

#attempt to configure multicast after a successful ping
echo $PASS | sudo -S ifconfig eth0 down $>> $LOG # restart usb0 interface
echo $PASS | sudo -S ifconfig eth0 up &>> $LOG
echo $PASS | sudo -S ifconfig eth0 multicast &>> $LOG
echo $PASS | sudo -S route add -net 224.0.0.0 netmask 240.0.0.0 dev eth0 &>> $LOG #add multicast route to eth0

#start mobilebot program
echo "starting mobilebot program" &>> $LOG
echo $PASS | sudo -S $BOTTOMDIR/bin/mobilebot &> /dev/null &
