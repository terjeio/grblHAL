//
// utils.c - Assorted utilities for networking plugin - sourced from internet and possibly modified (public domain)
//
// Part of GrblHAL
//

#include <string.h>
#include <ctype.h>
#include <stdarg.h>

#include "utils.h"
#include "GRBL/grbl.h"

char *stristr(const char *s1, const char *s2)
{
    const char *s = s1, *p = s2, *r = NULL;

    if (!s2 || strlen(s2) == 0)
        return (char *)s1;

    while(*s && *p) {

        if(CAPS(*p) == CAPS(*s)) {
            if(!r)
                r = s;
            p++;
        } else {
            p = s2;
            if(r)
                s = r + 1;
            if(CAPS(*p) == CAPS(*s)) {
                r = s;
                p++;
            } else
                r = NULL;
        }
        s++;
    }

    return *p ? NULL : (char *)r;
}

// NOTE: ensure buf is large enough to hold concatenated strings!
char *strappend (char *buf, int argc, ...)
{
    char c, *s = buf, *arg;

    va_list list;
    va_start(list, argc);

    while(argc--) {
        arg = va_arg(list, char *);
        do {
            c = *s++ = *arg++;
        } while(c);
        s--;
    }

    va_end(list);

    return buf;
}

bool is_valid_port (uint16_t port)
{
	return port > 0;
}

bool is_valid_hostname (const char *hostname)
{
    bool ok = true;
    char *s = (char *)hostname;
    int c;
    size_t len = 0;

    while(ok) {
    	if((c = *s++) == '\0')
    		break;
    	len++;
    	ok = c == '-' || isdigit(c) || (isalpha(c) && c != ' ');
    }

    return ok && len >= HOSTNAME_LENGTH_MIN && len <= HOSTNAME_LENGTH_MAX;
}

bool is_valid_ssid (const char *ssid)
{
    bool ok = true;
    char *s = (char *)ssid;
    int c;
    size_t len = 0;

    while(ok) {
    	if((c = *s++) == '\0')
    		break;
    	len++;
    	ok = isprint(c);
    }

    return ok && len >= SSID_LENGTH_MIN && len <= SSID_LENGTH_MAX;
}

bool is_valid_password (const char *password)
{
	size_t len = strlen(password);
    return len >= PASSWORD_LENGTH_MIN && len <= PASSWORD_LENGTH_MAX;
}
