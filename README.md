# CMSIS-FreeRTOS-STM32F103

Hello there random visitor.

Here you will find a project made for SOTER (Sistemas Operativos em Tempo Real), a course that I had in my master's degree at ISEP (Instituto Superior de Engenharia do Porto) about real time operating systems using the Nucleo F103RB board with a custom made PCB that fits into the provided connectors.

The project consists of printing to a LCD the high-level FreeRTOS queues used by the tasks and all of the tasks' stack watermarks. Pressing Switch 5 in the custom board switches to a Pong game, which uses the MMA8452Q 3-axis accelerometer. Another task sends all the logs from the accelerometer to the computer via the virtual COM port in the Nucleo Board (it has an integrated TTL/USART to USB converter).

In the SOTER folder you can find the code that was developed using the System Workbench for STM32. The code uses exclusively CMSIS and FreeRTOS.

In the Presentation folder you will find a small .ppt slide-show that shows key parts of the code and how to get a project without the standard library/HAL stuff. You can also download a pdf version, although the small video showcasing the Pong game obviously won't work.

In the Custom Schematic Board folder, you will find a .pdf schematic of the "shield" board used in the project, in case you want to replicate the project.

All task stacks were chosen based on their watermark. If the watermark went below 50 bytes, we would increase the stack.

The LCD driver was ported from a Adafruit library by a teacher, and then ported by me to use CMSIS registers, so it doesn't have any useful comments or explanations about how it works. I've only added a small filled circle print function shamelessly stolen from stack overflow. Full credits are given in the comments.

The project is distributed under the Mozilla Public License Version 2.0.
This means that you are free to use, modify and even sell any part of the code available in here, as long as you fully disclose your source code and keep the same license.

This project was made in colaboration with my colleague Daniel Freire.

My name is Jose Silva and if you have any questions regarding the code in this repository you can contact me at:
1130352 at isep.ipp.pt
