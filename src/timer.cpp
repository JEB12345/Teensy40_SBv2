#include <Arduino.h>
#include "timer.h"

IntervalTimer mainLoop;      // Interupt driven library for timing in microseconds
timer_data timer_state;       // Must call this as extern in main file

void timer_init() {
    // Setup the main 10ms loop timer
    mainLoop.begin(mainTimer, 1000);
}

 // Timer update loop
void mainTimer() {
  timer_state.systime++;
}