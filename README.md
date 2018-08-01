**FreeRTOS**
========

This is a port of FreeRTOS( v. 10.0.1 ) to the RISC-V based GAP8, IoT Application Processor by [GreenWaves-Technologies](https://greenwaves-technologies.com).

About FreeRTOS
--------------
FreeRTOS is an RTOS designed for small and resource constrained microcontrollers, issued under MIT license. It was developed and maintened, since 2003, by Richard Barry and his Real Time Engineers Ltd. It is now under the aegis of [Amazon Web Services](https://aws.amazon.com/blogs/opensource/announcing-freertos-kernel-v10/) and became Amazon FreeRTOS since 2017. FreeRTOS still remains available under MIT license.
Light and easily configurable, it supports multi-tasking, various synchronisation mechanism and communication between tasks, and many others features. You may refer to the official website to learn more about [FreeRTOS and its features](https://www.freertos.org/FreeRTOS_Features.html).

The port files can be found in the *lib/FreeRTOS/portable/GCC/GAP8/* folder. The port is done editing those 3 files, essentially focused on FreeRTOS and not other stacks, such as FreeRTOS-Plus.
```bash
lib/FreeRTOS/portable/GCC/GAP8/:
port_asm.S
port.c
portmacro.h
```

Getting Started
---------------

The FreeRTOS project is organised as follows :
```bash
freeRTOS
|-lib
|   |-FreeRTOS    <- Contains FreeRTOS kernel source code, and the port source files
|   |-include     <- Contains FreeRTOS kernel headers and macros
|-demos
|   |-gwt         <- Contains drivers and GAPUINO features code
```

There are several demos and examples available to present FreeRTOS' and GAPUINO's features.
Those demos can be found in the [*gap_sdk*](https://github.com/GreenWaves-Technologies/gap_sdk/tree/master/examples).


API Reference and Documentation
-------------------------------

For detailed documentation and FAQ on Amazon FreeRTOS, refer to the [Amazon FreeRTOS User Guide](https://aws.amazon.com/documentation/freertos)  or [ FreeRTOS FAQ](https://www.freertos.org/FAQ.html) about FreeRTOS Kernel.

You can find the API reference either on [Amazon FreeRTOS Kernel Reference Manual](https://docs.aws.amazon.com/freertos-kernel/latest/ref/welcome.html) or on [FreeRTOS API Reference](https://www.freertos.org/a00106.html).
