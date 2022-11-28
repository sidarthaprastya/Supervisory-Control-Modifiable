#include <stdio.h>
#include "./../../supervisory_control/esp32_arduino/fsm.h"


int case_button[15] = {0, 1, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1}; 
int counter_debounce = 0;
int debounce_out = 0;
int debounce_state = STATE_DETECT;

int main(){
    printf("SIMULASI TEST CASE DEBOUNCE\n");
    for (int i = 0; i < 15; i++){
        printf("INPUT: %d\tPREV STATE: %d\t", case_button[i], debounce_state);
        
        debouncer_fsm(case_button[i], 1, 1, case_button[i], &counter_debounce, &debounce_state, &debounce_out);
        
        printf("DEBOUNCE: %d\tCURRENT STATE: %d\n", debounce_out, debounce_state);
    }
 
    return 0;
}
