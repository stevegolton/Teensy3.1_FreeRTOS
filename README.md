# TeensyFreeRTOS
FreeRTOS on the Teensy 3.1 board taking a "barebones" approach.

The aim of this project is to provide a baseline Free RTOS level on the Teensy
3.1 board. I could find some examples of others where they had got FreeRTOS
running using the Arduino build environment however I wanted to get it running
using a barebones environment.

Some useful links:
A great start point for baremetal coding on the Teensy:
http://www.seanet.com/~karllunt/bareteensy31.html

Guide to installing FreeRTOS on the Teensy but using the Arduino environment:
http://rishifranklin.blogspot.co.uk/2014/03/freertos-on-teensy-31.html

## Files
FreeRTOSV8.2.3
	croutine.c
	event_groups.c
	list.c
	queue.c
	tasks.c
	timers.c
MemMang
	heap_2.c
GCC/ARM_CM3
	port.c
	portmacro.h
https://github.com/circuitsenses/Teensy-3.1-FreeRTOS
	FreeRTOSConfig.h
