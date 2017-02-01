#!/bin/bash

#Setting trigger and echo pins for device 1
	echo -n "Enter trigger pin for device 1 -> "
	read trigger_pin
	echo "$trigger_pin" > /sys/class/HCSR/HCSRdevice1/trigger_pin
	echo -n "Enter echo pin  for device 1 -> "
	read echo_pin
	echo "$echo_pin" > /sys/class/HCSR/HCSRdevice1/echo_pin
	
#Setting trigger and echo pins for device 2
	echo -n "Enter trigger pin for device 2 -> "
	read trigger_pin
	echo "$trigger_pin" > /sys/class/HCSR/HCSRdevice2/trigger_pin
	echo -n "Enter echo pin  for device 2 -> "
	read echo_pin
	echo "$echo_pin" > /sys/class/HCSR/HCSRdevice2/echo_pin
	
#Case 1(device 1): one-shot mode(mode=0, frequency=1, enable=0)
	echo "Case 1(device 1): one-shot mode(mode=0, frequency=0, enable=0)"
	echo "0" > /sys/class/HCSR/HCSRdevice1/mode
	echo "1" > /sys/class/HCSR/HCSRdevice1/frequency
	echo "0" > /sys/class/HCSR/HCSRdevice1/Enable
	
#Reading distance value device 1
	echo "One shot distance measurement device 1"
	cat /sys/class/HCSR/HCSRdevice1/distance
	
#Case 1(device 2): one-shot mode(mode=0, frequency=1, enable=0)
	echo "Case 1(device 2): one-shot mode(mode=0, frequency=0, enable=0)"
	echo "0" > /sys/class/HCSR/HCSRdevice2/mode
	echo "1" > /sys/class/HCSR/HCSRdevice2/frequency
	echo "0" > /sys/class/HCSR/HCSRdevice2/Enable
	
#Reading distance value device 2
	echo "One shot distance measurement device 2"
	cat /sys/class/HCSR/HCSRdevice2/distance
	
#Case 2(device 1): Periodic mode(mode=1, frequency = 10, enable=1(later 0))
	echo "Case 2(device 1): Periodic mode(mode=1, frequency = 10, enable=1(later 0))"
	echo "1" > /sys/class/HCSR/HCSRdevice1/mode
	echo "10" > /sys/class/HCSR/HCSRdevice1/frequency
	echo "1" > /sys/class/HCSR/HCSRdevice1/Enable
sleep 3
	echo "0" > /sys/class/HCSR/HCSRdevice1/Enable
	
#Reading distance value device 1
	echo "Last value of distance measured device 1"
	cat /sys/class/HCSR/HCSRdevice1/distance

#Case 2(device 2): Periodic mode(mode=1, frequency = 5, enable=1(later 0))
	echo "Case 2(device 2): Periodic mode(mode=1, frequency = 5, enable=1(later 0))"
	echo "1" > /sys/class/HCSR/HCSRdevice2/mode
	echo "5" > /sys/class/HCSR/HCSRdevice2/frequency
	echo "1" > /sys/class/HCSR/HCSRdevice2/Enable
sleep 3
	echo "0" > /sys/class/HCSR/HCSRdevice2/Enable

#Reading distance value device 2
	echo "Last value of distance measured device 2"
	cat /sys/class/HCSR/HCSRdevice2/distance
