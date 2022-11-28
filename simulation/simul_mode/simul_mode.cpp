#include <stdio.h>
#include "./../../supervisory_control/esp32_arduino/fsm.h"

int state_mode = MODE_START;
int case_button[15] = {0, 1, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1}; 
int counter_debounce = 0;
int debounce_out = 0;
int debounce_state = STATE_DETECT;

int main(){
    printf("SIMULASI TEST CASE MODE\n");
    for (int i = 0; i < 15; i++){
        printf("INPUT: %d\t", case_button[i]);
        debouncer_fsm(case_button[i], 1, 1, START_FLAG, &counter_debounce, &debounce_state, &debounce_out);
        if(debounce_out == 1){
            debounce_out = debounce_out * READ_MODE;
        }
        
        fsm_mode(debounce_out, &state_mode);
        
        printf("DEBOUNCE: %d\tMODE: %d\n", debounce_out, state_mode);
    }
 
    return 0;
}
