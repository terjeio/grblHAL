/*
  platform_LINUX.c - linux specific functions with generic cross-platform interface

  Part of Grbl Simulator

  Copyright (c) 2014 Adam Shelly

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <time.h>
#include <sys/time.h>
#include "platform.h"

#define MS_PER_SEC 1000000


//any platform-specific setup that must be done before sim starts here
void platform_init()
{
}

//cleanup int here;
void platform_terminate()
{
}

//returns a free-running 32 bit nanosecond counter which rolls over
uint32_t platform_ns() 
{
    static uint32_t gTimeBase = 0;
    static uint32_t timestamp;
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    timestamp = ts.tv_sec * 1e9 + ts.tv_nsec;
    if (gTimeBase == 0)
        gTimeBase = timestamp;

    return timestamp - gTimeBase;
}

//sleep in microseconds
void platform_sleep(long  microsec)
{
    struct timespec ts={0};

    while (microsec >= MS_PER_SEC){
        ts.tv_sec++;
        microsec -= MS_PER_SEC;
    }
    ts.tv_nsec = microsec * 1000;
    nanosleep(&ts, NULL);
}

#define SIM_ECHO_TERMINAL 0 //use this to make grbl_sim act like a serial terminal with local echo on.

//set terminal to allow kbhit detection
void enable_kbhit(int dir)
{
    static struct termios oldt, newt;

    if ( dir == 1 ) {
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON); 
        if (!SIM_ECHO_TERMINAL)
            newt.c_lflag &= ~(ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    }
    else
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
}
 
//detect key pressed
int kbhit (void)
{
    struct timeval tv = {0};
    fd_set rdfs = {{0}};
    int retval;

    /* tv.tv_sec = 0; */
    /* tv.tv_usec = 0; */

    /* FD_ZERO(&rdfs); */
    FD_SET(STDIN_FILENO, &rdfs);

    select(STDIN_FILENO + 1, &rdfs, NULL, NULL, &tv);
    retval = FD_ISSET(STDIN_FILENO, &rdfs);

    return retval;
}

plat_thread_t* platform_start_thread(plat_threadfunc_t threadfunc)
{
    plat_thread_t* th = malloc(sizeof(plat_thread_t));
    if (pthread_create(&th->tid, NULL, threadfunc, &th->exit)){
        free(th);
        return NULL;
    }
    return th;
}

//ask thread to exit nicely, wait
void platform_stop_thread(plat_thread_t* th)
{
    th->exit = 1;
    pthread_join(th->tid, NULL);  
}

//force-kill thread
void platform_kill_thread(plat_thread_t* th)
{
    th->exit = 1;
    pthread_cancel(th->tid); 
}

//return char if one available.
uint8_t platform_poll_stdin()
{
    uint8_t char_in = 0;

    enable_kbhit(1);
    if (kbhit())
        char_in = getchar();

    enable_kbhit(0);

    return char_in;
}
