#!/usr/bin/env python

import subprocess
import time

def get_device_name():
    proc = subprocess.Popen(['networksetup','-listallhardwareports'], stdout=subprocess.PIPE)
    wifi = False
    device_name = None
    while True:
        line = proc.stdout.readline()
        if line == '':
            break
        words = line.rstrip().split()
        if (2 < len(words) and
            words[0] == 'Hardware' and words[1] == 'Port:' and words[2] == 'Wi-Fi'):
            wifi = True
        elif wifi and words[0] == 'Device:':
            device_name = words[1]
        else:
            wifi = False
    if device_name == None:
        raise Exception('Wi-Fi device not found')
    return device_name

def get_status(device_name = get_device_name()):
    proc = subprocess.Popen(['ifconfig', device_name], stdout=subprocess.PIPE)
    wifi = False
    status = None
    while True:
        line = proc.stdout.readline()
        if line == '':
            break
        words = line.rstrip().split()
        if words[0] == 'status:':
            status = words[1]
    if status == None:
        raise Exception('Unknown Wi-Fi status')
    return status

def set_power(power, device_name = get_device_name()):
    proc = subprocess.check_output(['networksetup', '-setairportpower', device_name, power])

def get_ssid(device_name = get_device_name()):
    proc = subprocess.Popen(['networksetup', '-getairportnetwork', device_name],
                                stdout=subprocess.PIPE)
    ssid = None
    while True:
        line = proc.stdout.readline()
        if line == '':
            break
        words = line.rstrip().split()
        if (4 <= len(words) and
            words[0] == 'Current' and words[1] == 'Wi-Fi' and words[2] == 'Network:'):
            ssid = words[3]
    if ssid == None:
        raise Exception('Wi-Fi not connected')
    return ssid

def wait():
    prev_status = get_status()
    while True:
        status = get_status()
        if prev_status != status:
            break

        if status != 'active':
            set_power('off')
            set_power('on')
            time.sleep(2)


if __name__ == '__main__':
    print('device name = %s' % device_name())
    print('status = %s' % status())
    while True:
        wait()
	if status() == 'active':
            print('connected to %s' % ssid())
