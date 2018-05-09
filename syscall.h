
#include <stdlib.h>

__attribute__((always_inline)) static inline uint32_t __get_LR(void) { 
    register uint32_t result;
    __asm volatile ("mov %0, lp\n" : "=r" (result)); 
    return result;
}

__attribute__((always_inline)) static inline uint32_t __get_SP(void) { 
    register uint32_t result;
    __asm volatile ("mov %0, sp\n" : "=r" (result)); 
    return result;
}


#if 0
int _execve(char *name, char **argv, char **env);
int _fork(void);
int _getpid(void);
int _kill(int pid, int sig);
int _exit();
int _isatty(int file);
int _fstat(int file, struct stat *st);
int _link(char *old, char *new);
int _lseek(int file, int ptr, int dir);
int _open(const char *name, int flags, int mode);
int _read(int file, char *ptr, int len);
int _write(int file, char *ptr, int len);
int _stat(char *file, struct stat *st);
int _close(int file);
int _times(struct tms *buf);
int _unlink(char *name);
int _wait(int *status);
void * _sbrk(int incr);
#endif

/* EOF */
