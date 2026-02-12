#if !defined(UTIL_H)
#define UTIL_H 1

#include <stdint.h>
#include <stddef.h>

#if !defined(__FAR)
#define __far
#endif

static inline void memsetb(__far void *ptr, uint8_t c, size_t len)
{
    __far uint8_t *p = (__far uint8_t *)ptr;
    while( len )
    {
        *p = c;
        p++;
        len--;
    }
}

static inline void memsetw(__far void *ptr, uint16_t c, size_t len)
{
    __far uint16_t *p = (__far uint16_t *)ptr;
    while( len )
    {
        *p = c;
        p++;
        len--;
    }
}

static inline void memset(__far void *ptr, int c, size_t len)
{
    memsetb(ptr, c, len);
}

static inline void memcpyb(__far void *a, const __far void *b, size_t len)
{
    __far uint8_t *p_a = (__far uint8_t *)a;
    __far uint8_t *p_b = (__far uint8_t *)b;

    while( len )
    {
        *p_a = *p_b;
        p_a++;
        p_b++;
        len--;
    }
}

static inline void memcpyw(__far void *a, const __far void *b, size_t len)
{
    __far uint16_t *p_a = (__far uint16_t *)a;
    __far uint16_t *p_b = (__far uint16_t *)b;

    while( len )
    {
        *p_a = *p_b;
        p_a++;
        p_b++;
        len--;
    }
}

static inline void memcpy(void *a, const void *b, size_t len)
{
    memcpyb(a, b, len);
}

static inline uint8_t __inb (uint16_t __port)
{
    uint8_t __val;
    __asm volatile ("{inb %1, %0|in %0, %1}"
		  : "=Ral" (__val)
		  : "Nd" (__port));
    return __val;
}

static inline uint16_t __inw (uint16_t __port)
{
    uint16_t __val;
    __asm volatile ("{inw %1, %0|in %0, %1}"
		  : "=a" (__val)
		  : "Nd" (__port));
    return __val;
}


static inline uint8_t __outb (uint16_t __port, uint8_t __val)
{
    __asm volatile ("{outb %1, %0|out %0, %1}"
		  : /* no outputs */
		  : "Nd" (__port),
		    "Ral" ((uint8_t) __val));
    return __val;
}

static inline uint16_t __outw (uint16_t __port, uint16_t __val)
{
    __asm volatile ("{outw %1, %0|out %0, %1}"
		  : /* no outputs */
		  : "Nd" (__port), "a" (__val));
    return __val;
}

#define IMPORT_BIN(sect, file, sym) asm (\
    ".section " #sect "\n"                  /* Change section */\
    ".balign 4\n"                           /* Word alignment */\
    ".global " #sym "\n"                    /* Export the object address */\
    #sym ":\n"                              /* Define the object label */\
    ".incbin \"" file "\"\n"                /* Import the file */\
    ".global _sizeof_" #sym "\n"            /* Export the object size */\
    ".set _sizeof_" #sym ", . - " #sym "\n" /* Define the object size */\
    ".balign 4\n"                           /* Word alignment */\
    ".section \".text\"\n")                 /* Restore section */

#endif