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
#define STATE_DEBOUNCE_IN   11
#define STATE_HOLD          12
#define STATE_DEBOUNCE_OUT  13
#define STATE_CONTINUE      14

#define STATE_UP        20
#define STATE_UP_CONT   21
#define STATE_IDLE      22

#define READ_SHIFT      30
#define READ_ADD        31
#define READ_MODE       32

#define START_FLAG      9

#define BLOCK_1         0
#define BLOCK_2         1
#define BLOCK_3         2
#define BLOCK_4         3



void fsm_num_shift(int input, int *state){
    switch(*state){
        case BLOCK_1:
            if(input == READ_SHIFT){
                *state = BLOCK_2;
            }
            break;
        
        case BLOCK_2:
            if(input == READ_SHIFT){
                *state = BLOCK_3;
            }
            break;
        
        case BLOCK_3:
            if(input == READ_SHIFT){
                *state = BLOCK_4;
            }
            break;
        
        case BLOCK_4:
            if(input == READ_SHIFT){
                *state = BLOCK_1;
            }
            break;
        
        default:
            *state = BLOCK_1;
            break;
    }
}

void num_add(int button_add, int block_in, int num_out[4]){
    if(button_add == READ_ADD){
        if(num_out[block_in] < 9){
            num_out[block_in] += 1;
        }
        else{
            num_out[block_in] = 0;
        }
    }

}

void fsm_mode(int input, int *state){
    switch(*state){
        case MODE_START:
            if(input == READ_MODE){
                *state = MODE_P;
            }
            break;
        
        case MODE_P:
            if(input == READ_MODE){
                *state = MODE_I;
                
            }
            else if(input == START_FLAG){
                *state = MODE_START;
            }
            break;
        
        case MODE_I:
            if(input == READ_MODE){
                *state = MODE_D;
            }
            else if(input == START_FLAG){
                *state = MODE_START;
            }
            break;
        
        case MODE_D:
            if(input == READ_MODE){
                *state = MODE_P;
            }
            else if(input == START_FLAG){
                *state = MODE_START;
            }
            break;

        default:
            *state = MODE_START;
            break;
    }
}


void param_assign(int num_p[4], int num_i[4], int num_d[4], float *p, float *i, float *d){
    *p = 100 * num_p[3] + 10 * num_p[2] + num_p[1] + 0.1 * num_p[0];
    *i = 10 * num_i[3] + num_i[2] + 0.1 * num_i[1] + 0.01 * num_i[0];
    *d = 10 * num_d[3] + num_d[2] + 0.1 * num_d[1] + 0.01 * num_d[0];
}


void debouncer_fsm(int button_in, int tresh_count, int cont_count, int cont_out, int *counter, int *state_debounce, int *button_fsm){
    switch(*state_debounce){
        case STATE_DETECT:
            if(button_in != 0){
                *state_debounce = STATE_DEBOUNCE_IN;
                *button_fsm = button_in;
                *counter = 0;
            }
            break;

        case STATE_DEBOUNCE_IN:
            if(*counter > 1){
                *state_debounce = STATE_HOLD;
            }
            else{
                *button_fsm = 0;
                *counter += 1;
            }
            break;

        case STATE_HOLD:
            if(button_in == 0){
                *state_debounce = STATE_DEBOUNCE_OUT;
                *button_fsm = 0;
                *counter = 0;
            }
            else if(button_in != 0 && *counter < tresh_count){
                *button_fsm = 0;
                *counter += 1;
            }
            else if(button_in != 0 && *counter >= tresh_count){
                *state_debounce = STATE_CONTINUE;
                *button_fsm = cont_out;
                *counter = 0;
            }
            break;
        
        case STATE_CONTINUE:
            if(button_in == 0){
                *state_debounce = STATE_DEBOUNCE_OUT;
                *button_fsm = 0;
                *counter = 0;
            }
            else if(button_in != 0 && *counter < cont_count){
                *button_fsm = 0;
                *counter += 1;
            }
            else if(button_in != 0 && *counter >= cont_count){
                *button_fsm = cont_out;
                *counter = 0;
            }
            break;


        case STATE_DEBOUNCE_OUT:
            if(*counter > 1){
                *state_debounce = STATE_DETECT;
            }
            else{
                *button_fsm = 0;
                *counter += 1;
            }
            break;
        
        default:
            *button_fsm = 0;
            *state_debounce = STATE_DETECT;
            break;
    }
}

#endif
