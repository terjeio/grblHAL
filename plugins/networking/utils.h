//
// utils.h - Assorted utilities for networking plugin - some sourced from internet and possibly modified (public domain)
//
// Part of GrblHAL
//
#ifndef __NETWORKING_UTILS_H__
#define __NETWORKING_UTILS_H__

#include <stdint.h>
#include <stdbool.h>

// WARNING: keep max sized at or below relevant variable sizes defined in settings.h
//          when these are used for storage.
#define SSID_LENGTH_MIN 1
#define SSID_LENGTH_MAX 32
#define HOSTNAME_LENGTH_MIN 1
#define HOSTNAME_LENGTH_MAX 32
#define PASSWORD_LENGTH_MIN 8
#define PASSWORD_LENGTH_MAX 64

char *stristr(const char *s1, const char *s2);
char *strappend (char *buf, int argc, ...);
bool is_valid_port (uint16_t port);
bool is_valid_hostname (const char *hostname);
bool is_valid_ssid (const char *ssid);
bool is_valid_password (const char *password);

#endif
