# Baremetal UART Printf Implementation for STM32 (Using Newlib IO Retargeting)

This project demonstrates how to implement a bare-metal `printf` function to output data over UART on an embedded system, specifically for ARM-based microcontrollers.Im using STM32F072RB-DISCOVERY board in this project.

## Project Overview

In this project, we redirect the `printf()` function to UART to allow for formatted output, thus enabling printf-style debugging. This is achieved through a mechanism called "IO retargeting," which allows us to redirect standard library I/O operations (like `printf()`, `fwrite()`, etc.) to custom low-level functions.

I am using the GNU ARM toolchain, which includes the GCC compiler and the `newlib` C library. `Newlib` is specifically designed for embedded systems and includes implementations for standard C library functions. When our firmware calls a standard C function, such as `strcmp()`, the linker adds the appropriate `newlib` code to our project.

In the case of file I/O operations, such as `fopen()` and `fwrite()`, `newlib` calls a set of low-level IO functions, or "syscalls," to perform the required actions. By modifying the `_write()` syscall, we can redirect `printf()` output to UART or any other desired interface.

This project specifically focuses on retargeting `printf()` to UART. The process involves modifying the `_write()` syscall to send the data to a UART interface (UART1 in this case).

### Changes made comapred to previous projects:
1. **Move hardware abstraction layer (HAL) code to `hal.h`.**
2. **Move startup code to `startup.c`.**
3. **Create a `syscalls.c` file for newlib syscalls.**
4. **Modify the Makefile to include the new files (`syscalls.c`, `startup.c`).**

After organizing the code, we replace `uartWriteBuf()` calls with `printf()` in the main application. By modifying the `_write()` syscall, we can print the diagnostic information to UART, thus enabling easier debugging via serial output.

### Key Code Changes:
1. **`_write()` syscall implementation**: Redirects `printf()` output to UART1.
2. **Added missing syscalls**: Stub implementations for syscalls like `_fstat()`, `_close()`, `_read()`, etc., that newlib expects.
3. **Main function update**: Replaced `uartWriteBuf()` with `printf()` to print formatted messages to UART.

## Setup Instructions

### Prerequisites:
- ARM toolchain (`arm-none-eabi-gcc`)
- An ARM-based microcontroller (e.g., STM32)
- Serial terminal for viewing the output (e.g., `PuTTY`, `Tera Term`, or `minicom`)

### Steps:
1. **Install ARM Toolchain**:
   Make sure that the ARM toolchain is installed on your system. You can install it with the following command:
   ```bash
   sudo apt-get install gcc-arm-none-eabi
   ```

2. **Clone the Repository**:
   Clone this repository to your local machine.
   ```bash
   git clone https://github.com/Tabrez-dev/Baremetal-Printf-to-UART.git
   ```

3. **Build the Project**:
   Navigate to the project directory and run `make` to build the project:
   ```bash
   cd Baremetal-Printf-to-UART
   make build
   ```

4. **Flash the Firmware**:
   Flash the resulting binary (`.bin`) file to your embedded system using your preferred flashing tool (e.g., `OpenOCD`, `ST-Link`, etc.).
   ```
   make flash
   ```

6. **Open a Serial Terminal**:
   Open a serial terminal(like minicom) to observe the output of the `printf` function. The baud rate and other serial settings should match the configuration in your embedded system.

```
sudo minicom -D /dev/ttyUSB0 -b 115200

```

## Code Structure

- **`main.c`**: Contains the main program, including initialization of the system and UART, and the entry point for the application.
- **`syscalls.c`**: Contains the custom system call implementations, including the `_write` function for redirecting `printf` output to UART.
- **`startup.c`**: Handles the startup code, including the initialization of the system and peripherals before `main()` is called.
- **`hal.h`**: Header file defining hardware abstraction layer functions, typically for UART or other peripherals.
- **`Makefile`**: Defines build rules, including compilation, linking, and flashing the firmware.
- **`link.ld`**: The linker script, which specifies memory layout and sections for the application.
- **`firmware.bin`**: The compiled binary file to be flashed to the embedded system.
- **`firmware.elf`**: The ELF file containing the compiled code, useful for debugging.
- **`firmware.elf.map`**: The memory map file generated during the build process, useful for analyzing memory usage.

## Custom `_write` Implementation

In this project, the default `printf` function provided by `newlib-nano` is redirected to UART using a custom `_write` function. The `_write` function is responsible for transmitting the characters to the serial interface.

```c
int _write(int fd,char *ptr, int len){
(void) fd,(void)ptr, (void)len;
if(fd==1) uartWriteBuf(UART1, ptr, (size_t) len);
return -1;

}

```

## How It Works:
- The `_write` function is called by `printf` whenever data is ready to be printed.
- The function transmits each character in the string to the UART peripheral using the `uartWriteBuf` function.
