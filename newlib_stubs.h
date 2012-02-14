#ifndef NEWLIB_STUBS_H_8QSRQHO3
#define NEWLIB_STUBS_H_8QSRQHO3

#include <errno.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <sys/unistd.h>
#include <stm32f10x.h>

#ifndef STDOUT_USART
#define STDOUT_USART 1
#endif

#ifndef STDERR_USART
#define STDERR_USART 1
#endif

#ifndef STDIN_USART
#define STDIN_USART 1
#endif

#if STDIN_USART == 1
	 #define MYSTDIN USART1
#elif STDIN_USART == 2
	 #define MYSTDIN USART2
#elif STDIN_USART == 3
	 #define MYSTDIN USART3
#else
	 #error "You defined STDIN_USART wrong!"
#endif

#if STDOUT_USART == 1
	 #define MYSTDOUT USART1
#elif STDOUT_USART == 2
	 #define STDOUT USART2
#elif STDOUT_USART == 3
	 #define MYSTDOUT USART3
#else
	 #error "You defined STDOUT_USART wrong!"
#endif

#if STDERR_USART == 1
	 #define MYSTDERR USART1
#elif STDERR_USART == 2
	 #define MYSTDERR USART2
#elif STDERR_USART == 3
	 #define MYSTDERR USART3
#else
	 #error "You defined STDERR_USART wrong!"
#endif



/*
 environ
 A pointer to a list of environment variables and their values. 
 For a minimal environment, this empty list is adequate:
 *
char *__env[1] = { 0 };
char **environ = __env;
*/

int _close(int file);
void _exit(int status);
int _fstat(int file, struct stat *st);
int _isatty(int file);
int _link(char *old, char *new);
int _lseek(int file, int ptr, int dir);
int _read(int file, char *ptr, int len);
caddr_t _sbrk(int incr);
int _stat(const char *filepath, struct stat *st);
clock_t _times(struct tms *buf);
int _unlink(char *name);
int _wait(int *status);
int _write(int file, char *ptr, int len);

#endif /* end of include guard: NEWLIB_STUBS_H_8QSRQHO3 */
