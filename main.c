#include "hal.h"

void SystemInit(void) {
}

int main(void){
    uint16_t led=PIN('C',7);
    gpio_set_mode(led, GPIO_MODE_OUTPUT);
    uartInit(UART1,115200);

    char buffer[10]={0};
    uint8_t index=0;
    char message[] = "Send 'on' to turn blue LED on or send 'off' to turn the blue LED off.\r\n";
    uartWriteBuf(UART1, message, sizeof(message) - 1);  // Send message
    for(;;){
        if(uartReadReady(UART1)){
            char c = uartReadByte(UART1);
            if (c == '\r' || c == '\n') {
                buffer[index] = '\0';  // Null-terminate the string
                if (buffer[0] == 'o' && buffer[1] == 'n') {
                    gpio_write(led, 1);  // Turn LED on
                    printf("LED is on\r\n"); 
                } else if (buffer[0] == 'o' && buffer[1] == 'f' && buffer[2] == 'f') {
                    gpio_write(led, 0);  // Turn LED off
                    printf("LED is off\r\n");
                }else{
                    printf("Invalid Key\r\n");
                }
                index = 0;
            } else {
                if (index < sizeof(buffer) - 1) {
                    buffer[index++] = c;  // Add character to buffer
                } else {
                    index = 0;
                }
            }
        }
    }

    return 0;
}

