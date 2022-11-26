#ifndef FSM_H
#define FSM_H

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define MODE_START   0
#define MODE_P      1
#define MODE_I      2
#define MODE_D      3

#define STATE_DETECT        10
#define STATE_DEBOUNCE      11
#define STATE_HOLD          12
#define STATE_DEBOUNCE_OUT  13
#define STATE_CONTINUE     14

#define STATE_UP        20
#define STATE_UP_CONT   21
#define STATE_IDLE      22

#define LEFT    -1
#define RIGHT   1



void debounce(int input, int *counter, int *state, int *output){
    switch(*state){
        case STATE_HOLD:
            if(input == 1){
                *state = STATE_DEBOUNCE;
                *counter = 0;
                *output = 0;
            }
            else{
                *output = 1;
            }
            break;

        case STATE_DEBOUNCE:
            if(*counter > 2){
                *state = STATE_HOLD;
                *output = 1;
            }
            else{
                *counter += 1;
                *output = 0;
            }
            break;
        
        default:
            *output = 1;
            *counter = 0;
            *state = STATE_HOLD;
            break;

    }
}

void hold_counter(int button_edge, int button_type, int limit, int *counter, int *state, int *output, int *limit_alarm){
    switch(*state){
        case STATE_IDLE:
            if(button_edge != 0){
                *state = STATE_UP;
                *output = button_type;
                *counter = 0;
                *limit_alarm = 0;
            }
            else{
                *output = 0;
                *limit_alarm = 0;
            }
            break;
        
        case STATE_UP:
            if(button_edge == 0 && counter < limit){
                *output = 0;
                *counter += 1;
                *limit_alarm = 0;
            }
            else if(button_edge == 0 && counter >= limit){
                *output = button_type;
                *counter = 0;
                *limit_alarm = 1;
            }
            else{
                *state = STATE_IDLE;
                *output = 0;
                *counter = 0;
                *limit_alarm = 0;
            }
            break;
        
        default:
            *state = STATE_IDLE;
            *output = 0;
            *limit_alarm = 0;
            break;
    }
    
    
}

void edge_detect(int button_in, int debounce_out, int *state, int *edge){
    if(debounce_out == 1){
        switch(*state){
            case STATE_DETECT:
                if(button_in != 0){
                    *state = STATE_HOLD;
                    *edge = 1;
                }
                else{
                    *edge = 0;
                }
                break;

            case STATE_HOLD:
                if(button_in == 0){
                    *state = STATE_DETECT;
                    *edge = 1;
                }
                break;
            
            default:
                *edge = 0;
                *state = STATE_DETECT;
                break;
        }
    }
}


void fsm_mode(int input, int limit_alarm, int *state){
    switch(*state){
        case MODE_START:
            if(input == 1){
                *state = MODE_P;
            }
            break;
        
        case MODE_P:
            if(input == 1 ){
                *state = MODE_I;
            }
            if(limit_alarm == 1){
                *state = MODE_START;
            }
            break;
        
        case MODE_I:
            if(input == 1){
                *state = MODE_D;
            }
            if(limit_alarm == 1){
                *state = MODE_START;
            }
            break;
        
        case MODE_D:
            if(input == 1){
                *state = MODE_P;
            }
            if(limit_alarm == 1){
                *state = MODE_START;
            }
            break;

        default:
            *state = MODE_START;
            break;
    }
}


void param_config(int state_mode, int button_param, float *param_p, float *param_i, float *param_d){
    if(button_param != 0 && state_mode != MODE_START){
        if(state_mode == MODE_P){
            *param_p += button_param;
        }
        else if(state_mode == MODE_I){
            *param_i += button_param;
        }
        else if(state_mode == MODE_D){
            *param_d += button_param;
        }
    }
}


#endif
