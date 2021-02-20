#include <stdlib.h>
#include <ctype.h>
#include <stdint.h>

// By ThomasH - https://stackoverflow.com/questions/2673207/c-c-url-decode-library/2766963

char *urldecode (char *dst, const char *src)
{
    uint8_t a, b;
    char *d = dst;

    while (*src) {
        if ((*src == '%') && ((a = src[1]) && (b = src[2])) && (isxdigit(a) && isxdigit(b))) {
            if (a >= 'a')
                a -= 'a'-'A';
            if (a >= 'A')
                a -= ('A' - 10);
            else
                a -= '0';
            if (b >= 'a')
                b -= 'a'-'A';
            if (b >= 'A')
                b -= ('A' - 10);
            else
                b -= '0';
            *dst++ = 16 * a + b;
            src+=3;
        } else if (*src == '+') {
            *dst++ = ' ';
            src++;
        } else
            *dst++ = *src++;
    }

    *dst = '\0';

    return d;
}
