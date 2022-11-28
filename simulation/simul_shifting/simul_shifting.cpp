#include <stdio.h>
#include "./../supervisory_control/esp32_arduino/fsm.h"

int shift_state = BLOCK_1;
int case_button[15] = {1, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1}; 
int counter_debounce = 0;
int debounce_out = 0;
int debounce_state = STATE_DETECT;

int main(){
    printf("SIMULASI TEST CASE SHIFTING\n");
    for (int i = 0; i < 15; i++){
        printf("INPUT: %d\t", case_button[i]);
        debouncer_fsm(case_button[i], 1, 1, case_button[i], &counter_debounce, &debounce_state, &debounce_out);
        debounce_out = debounce_out * READ_SHIFT;
        fsm_num_shift(debounce_out, &shift_state);
        
        printf("DEBOUNCE: %d\tBLOCK: %d\n", debounce_out, shift_state);
    }
 
    return 0;
}
