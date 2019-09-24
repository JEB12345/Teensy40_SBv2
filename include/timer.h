#ifndef TIMER_H
#define TIMER_H

#include "state.h"

/* Initializes timers used in main file */
void timer_init();

/* Function to update the main loop's timer state */
void mainTimer();

#endif // TIMER_H