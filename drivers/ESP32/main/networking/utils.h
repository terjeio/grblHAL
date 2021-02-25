//
// utils.h - Assorted utilities for networking plugin - some sourced from internet and possibly modified (public domain)
//
// Part of grblHAL
//

#ifndef __NETWORKING_UTILS_H__
#define __NETWORKING_UTILS_H__

#include <stdint.h>
#include <stdbool.h>

// WARNING: keep min sizes below relevant variable sizes defined in settings.h

#define SSID_LENGTH_MIN 1
#define SSID_LENGTH_MAX (sizeof(ssid_t) - 1)
#define HOSTNAME_LENGTH_MIN 1
#define HOSTNAME_LENGTH_MAX (sizeof(hostname_t) - 1)
#define PASSWORD_LENGTH_MIN 8
#define PASSWORD_LENGTH_MAX (sizeof(password_t) - 1)
#define HIDDEN_PASSWORD "********"

char *btoa (uint64_t bytes);
bool is_valid_port (uint16_t port);
bool is_valid_hostname (const char *hostname);
bool is_valid_ssid (const char *ssid);
bool is_valid_password (const char *password);

#endif
