#!/bin/bash

getdevice()
{
    networksetup -listallhardwareports | awk '
    /^Hardware Port: Wi-Fi/ {
        wifi=1
    }
    /^Device: / {
        if (wifi) {
            print $2
            wifi = 0
        }
    }
    '
}

getifstatus()
{
    ifconfig $1 | awk '/status:/{ print $2 }'
}

wait_wifi_connection()
{
    device=`getdevice`
    if [ "x$device" == x ]; then
	echo no wifi interface available
	exit 1
    fi

    while : ; do
	status=`getifstatus $device`
	if [ x$prevstatus != x$status ]; then
	    prevstatus=$status
	    if [ "x$status" == xactive ]; then
		echo wifi connected
		break
	    else
		echo wifi disconnected
	    fi
	fi

	if [ "x$status" != xactive ]; then
	    networksetup -setairportpower $device off
	    networksetup -setairportpower $device on
	fi
	sleep 2
    done
}
