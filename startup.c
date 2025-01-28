//Startup code
__attribute__((naked, noreturn)) 
    void _reset(void){
        //long is quivalent to uint32_t
        extern long _sbss, _ebss, _sdata, _edata, _srcdata;

        for(long *dst = &_sbss; dst < &_ebss;dst++) *dst=0;
        for(long *dst= &_sdata, *src= &_srcdata;dst< &_edata;) *dst++=*src++;
        extern void main(void);
        main();
        for(;;) (void) 0; //infinite loop incase main() returns

    }

extern void _estack(void); //defined in linker script

//7 standard and 32 STM32-specific handlers
__attribute__((section(".vectors")))
//An array of 38  constant function pointers,
// where each function returns void and takes no arguments.
void (*const vector_table[7+32])(void)={
    _estack,
    _reset
}; 



