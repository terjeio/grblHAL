/*
  strutils.c - a collection of useful string utilities

Copyright (c) 2019 Terje Io
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

· Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

· Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

· Neither the name of the copyright holder nor the names of its contributors may
be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <string.h>
#include <stdarg.h>
#include <stdbool.h>

#include "strutils.h"

#define CAPS(c) ((c >= 'a' && c <= 'z') ? c & 0x5F : c)

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

uint32_t strnumentries (const char *s, const char delimiter)
{
    char *p = (char *)s;
    uint32_t entries = *s ? 1 : 0;

    while(entries && (p = strchr(p, delimiter))) {
        p++;
        entries++;
    }

    return entries;
}

char *strgetentry (char *res, const char *s, uint32_t entry, const char delimiter)
{
    char *e, *p = (char *)s;

    *res = '\0';
    e = strchr(p, delimiter);

    if(entry == 0 && e == NULL)
        strcpy(res, s);
    else do {
        if(entry) {
            p = e;
            e = strchr(++p, delimiter);
            entry--;
        } else {
            if(e) {
                strncpy(res, p,  e - p);
                res[e - p] = '\0';
            } else
                strcpy(res, p);
            break;
        }
    } while(true);

    return res;
}

int32_t strlookup (const char *s1, const char *s2, const char delimiter)
{
    bool found = false;
    char *e, *p = (char *)s2;
    uint32_t idx = strnumentries(s2, delimiter), len = strlen(s1);
    int32_t entry = 0;

    while(idx--) {

        if((e = strchr(p, delimiter)))
            found = (e - p) == len && !strncmp(p, s1, e - p);
        else
            found = strlen(p) == len && !strcmp(p, s1);

        if(found || e == NULL)
            break;
        else {
            p = e + 1;
            entry++;
        }
    }

    return found ? entry : -1;
}
