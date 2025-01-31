#pragma once

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <sys/stat.h> 
#include <stdlib.h>

#include "stm32f072xb.h"

#include "core_cm0.h"  // For Cortex-M0 CMSIS functions

#define FREQ 8000000  // HSI clock frequency is 8 MHz by default
#define BIT(x) (1UL << (x))
#define PIN(bank, num) ((((bank)-'A')<<8) | (num))
#define PINNO(pin) (pin & 255)
#define PINBANK(pin) (pin >> 8)
#define GPIO(bank) ((GPIO_TypeDef *) (GPIOA_BASE + 0x400U * (bank)))
enum { GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_AF, GPIO_MODE_ANALOG };
#define RCC ((RCC_TypeDef *) RCC_BASE)
#define UART1 USART1


static inline void systick_init(uint32_t ticks) {
    SysTick_Config(ticks);
}


static inline void spin(volatile uint32_t count){

while(count--) (void) 0;

}


static inline void gpio_set_mode(uint16_t pin, uint8_t mode) {
    // Get the GPIO bank (port) for the specified pin (A, B, C, etc.)
    GPIO_TypeDef *gpio = GPIO(PINBANK(pin));
    // Extract the pin number (0-15) from the pin identifier (e.g., 'A5' -> 5)
    int n = PINNO(pin);
    RCC->AHBENR |= BIT(17+PINBANK(pin));        // Enable GPIO clock
    //RCC_GPIO_CLK_ENABLE(PINBANK(pin));//verify if this works
    // Clear the existing 2-bit mode configuration for the specific pin
    // The 2 bits for each pin are at positions 2*n and 2*n+1 in MODER.
    gpio->MODER &= ~(3U << (n * 2));
    // Set the new mode by OR'ing the correct mode value at the proper position
    // (mode & 3) ensures we only take the lower 2 bits of the mode
    // and then shift them to the correct position for the pin.
    gpio->MODER |= (mode & 3U) << (n * 2);
}

static inline void gpioSetAF(uint16_t pin, uint8_t afNum){
    GPIO_TypeDef *gpio = GPIO(PINBANK(pin));
    int n = PINNO(pin);

    if(n<8){
        gpio->AFR[0] &= ~(0xFUL << (n * 4)); //clear AF bit sin AFRL
        gpio->AFR[0] |= ((uint32_t)afNum << (n *4));
    } else {
        gpio->AFR[1] &= ~(0xFUL << ((n-8) * 4));
        gpio->AFR[1] |= ((uint32_t)afNum << ((n-8) * 4));
    }
}

static inline void gpio_write(uint16_t pin, bool val) {
    // Access the GPIO structure corresponding to the pin's bank (group of pins)
    GPIO_TypeDef *gpio = GPIO(PINBANK(pin));
    // Write the value to the pin's bit set/reset register (BSRR).
    // The value is shifted left based on whether 'val' is true or false.
    // If 'val' is true (high), it sets the corresponding pin high.
    // If 'val' is false (low), it resets the pin (sets it low).
    gpio->BSRR = (1U << PINNO(pin)) << (val ? 0 : 16);
}

//https://www.st.com/resource/en/datasheet/stm32f072rb.pdf
//Only enables USART1
static inline void uartInit(USART_TypeDef *uart , unsigned long baud){
    uint8_t af=1;		//AF1 for USART1 from AF memory mapping
    uint16_t rx=0,tx=0;	

    //Enable Clock for the selected UART
    if(uart == UART1) RCC->APB2ENR |= BIT(14);

    //configure the tx and rx pins
    if(uart == UART1){
        tx = PIN('A',9);
        rx = PIN('A',10);
    }
    
   //Configure GPIO for UART pins in AF mode
    gpio_set_mode(tx, GPIO_MODE_AF);
    gpioSetAF(tx,af);
    gpio_set_mode(rx, GPIO_MODE_AF);
    gpioSetAF(rx,af);
    
    //Configure the UART
    uart->CR1 = 0; 			//Disable the UART before configuring
    uart->BRR = FREQ / baud; 		//Set Baud rate (FREQ is the UART clock frequency) 
    uart->CR1 |= BIT(0) | BIT(2) | BIT(3);//Enable UART, RX, TX
}
#if 0
static inline void uartInit(USART_TypeDef *uart , unsigned long baud) {
    // Enable GPIOA Clock
    RCC->AHBENR |= BIT(17);  // GPIOA Enable

    // Enable UART1 Clock
    RCC->APB2ENR |= BIT(14); // USART1 Enable

    // Set PA9 & PA10 to Alternate Function mode
    GPIOA->MODER &= ~(3U << (9 * 2));  // Clear PA9 Mode
    GPIOA->MODER |= (2U << (9 * 2));   // Set PA9 to AF Mode

    GPIOA->MODER &= ~(3U << (10 * 2)); // Clear PA10 Mode
    GPIOA->MODER |= (2U << (10 * 2));  // Set PA10 to AF Mode

    // Set Alternate Function AF1 for PA9 and PA10
    GPIOA->AFR[1] &= ~((uint32_t)0xF << ((9 - 8) * 4));  // Clear AF9
    GPIOA->AFR[1] |=  ((uint32_t)1  << ((9 - 8) * 4));  // Set AF9 = AF1

    GPIOA->AFR[1] &= ~((uint32_t)0xF << ((10 - 8) * 4)); // Clear AF10
    GPIOA->AFR[1] |=  ((uint32_t)1  << ((10 - 8) * 4));  // Set AF10 = AF1

    // Disable USART before configuring
    uart->CR1 = 0;

    // Set Baud Rate
    uart->BRR = (FREQ / baud);

    // Enable USART, TX, RX
    uart->CR1 |= BIT(0) | BIT(2) | BIT(3);
}
#endif

//Check if UART RX data is ready
static inline int uartReadReady(USART_TypeDef *uart){
    return uart->ISR & BIT(5); //Check RXNE(Receiver Not Empty) (bit5 in ISR register)

}

//Read a single Byte from UART
static inline uint8_t uartReadByte(USART_TypeDef *uart){
    return (uint8_t)(uart->RDR & 0xFF); //Read 8 bit data from RDR reg
}

//write a single byte to UART 
static inline void uartWriteByte(USART_TypeDef *uart, uint8_t byte){
    while((uart->ISR & BIT(7)) == 0); //Wait untill TXE (bit 7 in ISR register) is set
    uart->TDR = byte; //write byte to transmit data register
}

//write a buffer to UART
static inline void uartWriteBuf(USART_TypeDef *uart, char *buf, size_t len){
    while(len-- > 0) {
        uartWriteByte(uart, *(uint8_t *) buf++);
    }

}




