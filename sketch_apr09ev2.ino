#include <Fsm.h>
#include <Controllino.h>  /* Usage of CONTROLLINO library allows you to use CONTROLLINO_xx aliases in your sketch. */
#include "ModbusRtu.h"
 
// data array for modbus network sharing
uint16_t au16data[16];
uint8_t u8state;
 
/**
 *  Modbus object declaration
 *  u8id : node id = 0 for master, = 1..247 for slave
 *  u8serno : serial port (use 0 for Serial)
 *  u8txenpin : 0 for RS-232 and USB-FTDI 
 *               or any pin number > 1 for RS-485
 */
Modbus master(0, 3, 2); // this is master and RS-232 or USB-FTDI
 
// This is an structe which contains a query to an slave device
modbus_t telegram;
 
unsigned long u32wait;

unsigned long time_now = 0;

//Events
#define FOGGER_ON_EVENT  0
#define FOGGER_OFF_EVENT  1

#define HEATER_ON_EVENT  0
#define HEATER_OFF_EVENT  1

State state_fogger_min_off(&switch_fogger_off, NULL, NULL);
State state_fogger_off(&log_fogger_off, NULL, NULL);
State state_fogger_min_on(&switch_fogger_on, NULL, NULL);
State state_fogger_on(&log_fogger_on, NULL, NULL);
Fsm fsm_fogger(&state_fogger_off);

State state_heater_min_off(&switch_heater_off, NULL, NULL);
State state_heater_off(&log_heater_off, NULL, NULL);
State state_heater_min_on(&switch_heater_on, NULL, NULL);
State state_heater_on(&log_heater_on, NULL, NULL);
Fsm fsm_heater(&state_heater_off);


// Transition functions
void switch_fogger_off()
{
  Serial.println("switching fogger off");
  digitalWrite(CONTROLLINO_R3, LOW);
}

void switch_fogger_on()
{
  Serial.println("switch fogger on");
  digitalWrite(CONTROLLINO_R3, HIGH);
}

void log_fogger_off()
{
  Serial.println("transitioned to state_fogger_off");
}

void log_fogger_on()
{
  Serial.println("transitioned to state_fogger_on");
}


void switch_heater_off()
{
  Serial.println("switching heater off");
  digitalWrite(CONTROLLINO_R4, LOW);
}

void switch_heater_on()
{
  Serial.println("switch heater on");
  digitalWrite(CONTROLLINO_R4, HIGH);
}

void log_heater_off()
{
  Serial.println("transitioned to state_heater_off");
}

void log_heater_on()
{
  Serial.println("transitioned to state_heater_on");
}


void check_sensor() {
  switch( u8state ) {
  case 0: 
    if (millis() > u32wait) u8state++; // wait state
    break;
  case 1: 
    telegram.u8id = 1; // slave address
    telegram.u8fct = 4; // function code (this one is registers read)
    telegram.u16RegAdd = 1; // start address in slave
    telegram.u16CoilsNo = 2; // number of elements (coils or registers) to read
    telegram.au16reg = au16data; // pointer to a memory array in the Arduino
    master.query(telegram); // send query (only once)
    u8state++;
    break;
  case 2:
    master.poll(); // check incoming messages
    if (master.getState() == COM_IDLE) {
      u8state = 0;
      u32wait = millis() + 3000; 
      Serial.print("temperature: ");
      Serial.print(au16data[0] / 10.0);   
      Serial.println("°C");
      Serial.print("humidity: ");
      Serial.print(au16data[1] / 10.0);   
      Serial.println("%");
      
      if(((au16data[1] / 10.0) > 80 || au16data[1] == 0)) {
        fsm_fogger.trigger(FOGGER_OFF_EVENT);
      }
      if((au16data[1] / 10.0) < 71 && au16data[1] > 0) {
        fsm_fogger.trigger(FOGGER_ON_EVENT);
      }
      
      //fsm_fogger.trigger(FOGGER_ON_EVENT);

      if((au16data[0] / 10.0) > 20.4) {
        fsm_heater.trigger(HEATER_OFF_EVENT);
      }
      if((au16data[0] / 10.0) < 20.0) {
        fsm_heater.trigger(HEATER_ON_EVENT);
      }
}
    break;
  }
}
 
void setup() {

  pinMode(CONTROLLINO_R3, OUTPUT);
  pinMode(CONTROLLINO_R4, OUTPUT);
  Serial.begin(9600);
  master.begin(9600); // baud-rate at 9600
  master.setTimeOut(5000); // if there is no answer in 5000 ms, roll over
  u32wait = millis() + 2000;
  u8state = 0;  

  Serial.println("setup");

  fsm_fogger.add_transition(&state_fogger_off, &state_fogger_min_on,
                     FOGGER_ON_EVENT, NULL);
  fsm_fogger.add_timed_transition(&state_fogger_min_on, &state_fogger_on, 90000, NULL);
  fsm_fogger.add_transition(&state_fogger_on, &state_fogger_min_off, FOGGER_OFF_EVENT, NULL);
  fsm_fogger.add_timed_transition(&state_fogger_min_off, &state_fogger_off, 120000, NULL);

  fsm_heater.add_transition(&state_heater_off, &state_heater_min_on,
                     HEATER_ON_EVENT, NULL);
  fsm_heater.add_timed_transition(&state_heater_min_on, &state_heater_on, 180000, NULL);
  fsm_heater.add_transition(&state_heater_on, &state_heater_min_off, HEATER_OFF_EVENT, NULL);
  fsm_heater.add_timed_transition(&state_heater_min_off, &state_heater_off, 180000, NULL);
}
 
void loop() {
  if(millis() > time_now + 1000){ // run every second
    time_now = millis();
    fsm_fogger.run_machine();
    fsm_heater.run_machine();
    check_sensor();
  }
}
