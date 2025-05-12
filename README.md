An Intruder Detection System implemented on the STM32. A collaborative effort of Cindy Chen, Riya Bansal, and Joonseo Park.

All source code is in Embedded C, and the project was developed on Keil uVision.

main_project.c contains the integrated source code for:
- human detection using connected component labelling + boundary calculations
- Sobel Filtering
- Global Binary Thresholding
- Motion detection using frame differencing
- A finite state machine for keypad combinations
- A random authorization code sent over Bluetooth
- Frame compression with Run Length Encoding + Data Truncation

OV7670.c contains the source code for:
- Initializing the OV7670 camera module via I2C
- Using memory mapped I/O addresses to configure camera register settings
- Functions for starting snapshot/continuous capture mode

stm32f4xx_it.c contains the source code for:
- Interrupt request handler for a completed DMA frame transfer
- A general interrupt request handler for misc. camera related interrupts
