#ifndef STATE_H
#define STATE_H

#define bool uint8_t

typedef struct {
        unsigned int volatile   systime;        //updated by a timer
        unsigned int            prev_systime;   //updated in main loop
} timer_data;

#endif // STATE_H