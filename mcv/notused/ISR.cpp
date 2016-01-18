#include "ISR.h"
unsigned int volatile tim2_cnt;

ISR(TIMER2_OVF_vect) {
  tim2_cnt++;
  // set to invoke every 10ms
  // change 250 to 25, if this should be called every 1ms
  if(tim2_cnt >= ISR10ms) {
    tim2_cnt = 0;
    rts.dispatch();
  }
}