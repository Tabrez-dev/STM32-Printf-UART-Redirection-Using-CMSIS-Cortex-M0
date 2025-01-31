# Baremetal UART Printf Implementation for STM32 using CMSIS

# STM32-Printf-UART-Redirection-Using-CMSIS-Cortex-M0

This repository demonstrates how to use `printf` redirection over UART on STM32 Cortex-M0 with CMSIS. Follow the steps below to clone and build the project.

## Steps to Reproduce

1. **Clone the Repository:**

   Clone the repository to your local machine:
   ```bash
   git clone https://github.com/Tabrez-dev/STM32-Printf-UART-Redirection-Using-CMSIS-Cortex-M0.git
   cd STM32-Printf-UART-Redirection-Using-CMSIS-Cortex-M0
   ```

2. **Clean the Build:**

   Run the following command to clean any previous build files:
   ```bash
   make clean
   ```

3. **Build the Project:**

   Next, build the project using:
   ```bash
   make build
   ```
   This will automatically clone any necessary submodules and prepare the project for flashing. 

4. **Modify STM32 Header File:**

   After building the project, you need to modify the STM32 header file:
   - Access the `cmsis_f0/Include/stm32f0xx.h` file using the following command:
     ```bash
     vim cmsis_f0/Include/stm32f0xx.h
     ```
   - Uncomment the line for the #define STM32F0x2.h series for the board STM32F072RBT6-DISCO or the baord you are working with.

5. **Flashing the Microcontroller:**

   To flash the firmware to your STM32 microcontroller, it's recommended to use the following command:
   ```bash
   make jflash
   ```
   This will use J-Link to flash the firmware onto your STM32 device.

6. **Debugging (Optional):**

   If you encounter any issues or need to debug, I recommend using the Ozone debugger. It allows you to monitor the execution and troubleshoot any problems during flashing.

   You can start debugging by connecting your J-Link and opening the Ozone debugger with the configuration files.

## Notes

- Ensure that the STM32 header file modification is done after cloning and building the repository to ensure proper functionality.
- If you encounter issues during flashing or debugging, check your connection and ensure the correct STM32 microcontroller model is selected in the header file.

