/*
The <sys/stat.h> header is included because it defines the struct stat type, which is used in functions like _fstat() and _stat(). it is requried by newlib expectations
*/
#include <sys/stat.h>
#include "hal.h"

int _fstat(int fd, struct stat *st){
    if(fd<0) return -1;
    st->st_mode = S_IFCHR;
    //When you set the st_mode field to S_IFCHR, you're telling the system that the "file" being described by struct stat is a character device.
    return 0;
}

void *_sbrk(int incr){

    extern char _end;
    static unsigned char * heap = NULL;
    unsigned char *prev_heap;
    if(heap == NULL) heap=(unsigned char*) &_end;
    prev_heap =heap;
    heap+=incr;
    return prev_heap;

}
/*
 *By adding (void) fd;, you tell the compiler, "I'm intentionally not using this variable."
 * */
int _open(const char *path){
    (void) path;//Suppresses warnings for unused arguments
    return -1;
}

int _close(int fd){
    (void) fd;
    return -1;
}

int _isatty(int fd){

    (void) fd;
    return 1;

}

int _lseek(int fd,int ptr,int dir){
    (void) fd, (void) ptr, (void) dir;
    return 0;
}

void _exit(int status){
(void) status;
for(;;) asm volatile("BKPT #0");
}

void _kill(int pid, int sig){
(void) pid, (void) sig;
}

int _getpid(void){
return -1;
}
/*
 Since we've used a newlib stdio function, we need to supply newlib with the rest of syscalls. We add a simple stubs that do nothing, with exception of _sbrk(). It needs to be implemented, since printf() calls malloc() which calls _sbrk()
*/
int _write(int fd,char *ptr, int len){
//indicates to compiler we are not using these variables
(void) fd,(void)ptr, (void)len;
//IO retargeting
//by using (void) for fd you're explicitly stating that other values of fd are intentionally ignored.
if(fd==1) uartWriteBuf(UART1, ptr, (size_t) len);
return -1;

}

int _read(int fd, char *ptr, int len){
    (void) fd, (void) ptr, (void) len;
    return -1;
}

int _link(const char *a, const char *b){
    (void) a,(void) b;
    return -1;
}

int _unlink(const char *a){
    (void) a;
    return -1;
}


int _stat(const char *path, struct stat *st){
    (void) path,(void) st;
    return -1;
}

int mkdir(const char *path,mode_t mode){
    (void) path, (void) mode;
    return -1;
}


void _init(void) {
}
