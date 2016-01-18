#include "RTSched.h"
RTSched rts;

RTSched::RTSched()
{
  cli();
  // clear timer2 registers
  TCCR2A = 0;
  TCCR2B = 0;

  // set compare match register as desired
  // can be used both for 1us & 10us precision
  // (249+1)(CTC) * 250(ISR called per sec) * 256(prescaler) = 16MHz
  OCR2A = 249; 

  // CTC on 
  //  WGM22 WGM21 WGM20
  //  0     1     0
  //TCCR2B |= (1 << WGM22);
  TCCR2A |= (1 << WGM21);
  //TCCR2A |= (1 << WGM20);

  //set 256 prescaler
  //  CS22  CS21  CS20
  //  1     1     0
  TCCR2B |= (1 << CS22);
  TCCR2B |= (1 << CS21);
  //TCCR2B |= (1 << CS20);

  // timer cmp interrupt on
  TIMSK2 |= (1 << OCIE2A);

  // init function list
  clear();
  sei();
  return; 
}

int RTSched::size() {
  return tim2_task_list.size();
}
int RTSched::clear() {
  tim2_task_list.clear();
  return 1;
}

int RTSched::add(TaskStruct task) {
  tim2_task_list.add(task);
  return 1;
}

int RTSched::remove(int idx) {
  tim2_task_list.remove(idx);
  return 1;
}

int RTSched::dispatch() {
  // dispatch functions
  for(int i = 0; i < tim2_task_list.size(); i++){
    tim2_task_list.get(i).fp(tim2_task_list.get(i).var1, tim2_task_list.get(i).var2, i);
  }
  return 1;
}



