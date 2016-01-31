# TeensyFreeRTOS
Minimal FreeRTOS for the Teensy 3.1.

The bare metal code was taken pretty much verbatim from [this tutorial](http://www.seanet.com/~karllunt/bareteensy31.html). I strongly recommend reading this tutorial as it's very well written and contains an extraordinary amount of information. I would also recommend taking some time to read through the code as it is very well commented and provided me a great insight into how the MCU works.

The FreeRTOS files are available for download from their site and forms the bulk of the code. I used the m3 portable code and the heap_2.c memory management technique.

I used the [this tutorial](http://rishifranklin.blogspot.co.uk/2014/03/freertos-on-teensy-31.html) as a rough guide to modify the bare metal code to run FreeRTOS. It is targeted at the TeensyDuino environment but could be applied to my bare metal code pretty easily. I copied the FreeRTOSConfig.h pretty much verbatim.

## Files
 - Taken or barely modified from http://www.seanet.com/~karllunt/bareteensy31.html
  - crt0.o
  - Makefile
  - main.c
  - sysinit.c
  - common.h
  - arm_cm4.c
  - Teensy31_flash.ld
  - assosiated header files...
 - FreeRTOSV8.2.3 common code
  - croutine.c
  - event_groups.c
  - list.c
  - queue.c
  - tasks.c
  - timers.c
  - assosiated header files...
 - MemMang
  - heap_2.c
 - GCC/ARM_CM3
  - port.c
  - portmacro.h
 - https://github.com/circuitsenses/Teensy-3.1-FreeRTOS
  - FreeRTOSConfig.h
  
## Build
This build has only been tested in Ubutnu 14.04 for now but it shouldn't be too hard to fiddle with the paths in the makefile to make it work on Windows.

 - Install the gcc-arm-none-eabi toolchain with `# apt-get install gcc-arm-none-eabi` on Ubuntu, or download the [installer](https://launchpad.net/gcc-arm-embedded/+download) on Windows.
 - [Teensy bootloader](https://www.pjrc.com/teensy/loader.html) (I used the command line version on Ubuntu).
 - Navigate to the project directory and run `make`.
 - Follow the instructions on the teensy bootloader website above for programming the teensy.
