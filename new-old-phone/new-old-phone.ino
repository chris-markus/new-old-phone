#include <SoftwareSerial.h>

#define CMD_PLAY_NEXT 0x01
#define CMD_PLAY_PREV 0x02
#define CMD_PLAY_W_INDEX 0x03
#define CMD_SET_VOLUME 0x06
#define CMD_SEL_DEV 0x09
#define CMD_PLAY_W_VOL 0x22
#define CMD_PLAY 0x0D
#define CMD_PAUSE 0x0E
#define CMD_SINGLE_CYCLE 0x19

#define DEV_TF 0x02
#define SINGLE_CYCLE_ON 0x00
#define SINGLE_CYCLE_OFF 0x01

#define ARDUINO_RX 8  // Arduino Nano Pin connected to the TX of the Serial MP3 Player module
#define ARDUINO_TX 7  // Arduino Nano Pin connected to the RX of the Serial MP3 Player module

#define KNOB_SNS A7
#define RING_NOW_BTN 2

#define KNOB_MIN 0
#define KNOB_MAX 600

#define PIN_BELL_EN 4
#define PIN_BELL_IN1 5
#define PIN_BELL_IN2 6

#define PIN_HOOK 9

enum BellDriveState {
  EN1,
  EN1_2,
  EN2,
  BELL_DRIVE_MAX,
};

enum RingerState {
  WAITING,
  RING1,
  WAIT1,
  RING2,
  WAIT2,
  RING3,
  WAIT3,
  RING4,
  WAIT4,
  RING5,
  WAIT5,
  NO_ANSWER,
  ANSWERED,
  LISTENING,
  HUNG_UP,
};

const unsigned long MIN_DELAY = 60ul * 1000ul;
const unsigned long MAX_DELAY = 10ul * 60ul * 1000ul;

const unsigned long RING_DURATION = 3000;
const unsigned long WAIT_DURATION = 2000;

volatile bool bell_en = false;
BellDriveState bell_drive = EN1;

SoftwareSerial mp3(ARDUINO_RX, ARDUINO_TX);

// ISR to drive bell
ISR(TIMER1_COMPA_vect){
  if(bell_en) {
    digitalWrite(PIN_BELL_EN, HIGH);
    switch(bell_drive) {
      case EN1:
      case EN1_2:
        digitalWrite(PIN_BELL_IN2, LOW);
        digitalWrite(PIN_BELL_IN1, HIGH);
        break;
      case EN2:
        digitalWrite(PIN_BELL_IN1, LOW);
        digitalWrite(PIN_BELL_IN2, HIGH);
        break;
      default:
        digitalWrite(PIN_BELL_IN1, LOW);
        digitalWrite(PIN_BELL_IN2, LOW);
        break;
    }
  }
  else {
    digitalWrite(PIN_BELL_EN, LOW);
    digitalWrite(PIN_BELL_IN1, LOW);
    digitalWrite(PIN_BELL_IN2, LOW);
  }
  bell_drive = (BellDriveState)((bell_drive + 1) % BELL_DRIVE_MAX);
}

unsigned long last_state_change = 0;
unsigned long last_state_print = 0;

RingerState get_next_state(
  const RingerState current_state,
  const unsigned long ring_delay,
  const bool ring_now,
  const bool on_hook)
{
  RingerState next_state = current_state;
  switch (current_state) {
    case WAITING:
      if ((millis() - last_state_change > ring_delay && on_hook) || ring_now)
      {
        next_state = RING1;
      }
      if (millis() - last_state_print > 5000) {
        Serial.print("Waiting, ");
        Serial.print((signed long)(ring_delay) - (signed long)(millis() - last_state_change));
        Serial.println("ms until next ring.");
        last_state_print = millis();
      }
      break;
    case RING1:
    case RING2:
    case RING3:
    case RING4:
    case RING5:
      if (!on_hook) {
        next_state = ANSWERED;
      }
      else if (millis() - last_state_change > RING_DURATION) {
        next_state = (RingerState)(current_state + 1);
      }
      break;
    case WAIT1:
    case WAIT2:
    case WAIT3:
    case WAIT4:
    case WAIT5:
      if (!on_hook) {
        next_state = ANSWERED;
      }
      else if (millis() - last_state_change > WAIT_DURATION) {
        next_state = (RingerState)(current_state + 1);
      }
      break;
    case NO_ANSWER:
      next_state = WAITING;
      break;
    case ANSWERED:
      next_state = LISTENING;
      break;
    case LISTENING:
      // Give a generous 200ms to debounce
      if (millis() - last_state_change > 200 && on_hook) {
        next_state = HUNG_UP;
      }
      break;
    case HUNG_UP:
      next_state = WAITING;
      break;
  }
  if (current_state != next_state) {
    Serial.print("State change! Prev: ");
    Serial.print(current_state);
    Serial.print(", next: ");
    Serial.println(next_state);
    last_state_change = millis();
  }
  return next_state;
}


RingerState current_ringer_state = WAITING;
unsigned long ring_delay = MIN_DELAY;

void setup() {

  pinMode(KNOB_SNS, INPUT);
  pinMode(RING_NOW_BTN, INPUT_PULLUP);

  pinMode(PIN_BELL_EN, OUTPUT);
  pinMode(PIN_BELL_IN1, OUTPUT);
  pinMode(PIN_BELL_IN2, OUTPUT);

  pinMode(PIN_HOOK, INPUT_PULLUP);

  cli();//stop interrupts

  //set timer1 interrupt at 60Hz -> 3x iterations per period -> 20Hz bell
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 60hz increments
  OCR1A = 260;// = (16*10^6) / (60*1024) (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  sei();
  
  Serial.begin(9600);
  Serial.println("Initializing");
  
  mp3.begin(9600);
  delay(1000);  // wait chip initialization is complete

  // To get the MP3 player into a known state, set the source and pause playback
  mp3_command(CMD_SEL_DEV, DEV_TF);  // select the TF card
  delay(200);                        // wait for 200ms
  mp3_command(CMD_PAUSE, 0x0000);      // Pause mp3
}

void loop() {
  // RING_NOW is normally-open button that shorts to ground when pressed
  const bool ring_now_state = digitalRead(RING_NOW_BTN) == LOW;
  
  // HOOK is normally-closed button that shorts to ground when depressed
  const bool on_hook = digitalRead(PIN_HOOK) == HIGH;

  // First get the next state
  const RingerState next_state = get_next_state(current_ringer_state, ring_delay, ring_now_state, on_hook);

  // Now take actions based on state
  switch (next_state) {
    case WAITING:
      ring_delay = constrain(map(analogRead(KNOB_SNS), KNOB_MIN, KNOB_MAX, MIN_DELAY, MAX_DELAY), MIN_DELAY, MAX_DELAY);
      bell_en = false;
      break;
    case RING1:
    case RING2:
    case RING3:
    case RING4:
    case RING5:
      bell_en = true;
      break;
    case WAIT1:
    case WAIT2:
    case WAIT3:
    case WAIT4:
    case WAIT5:
      bell_en = false;
      break;
    case NO_ANSWER:
      bell_en = false;
      break;
    case ANSWERED:
      bell_en = false;
      mp3_command(CMD_PLAY_NEXT, 0x0000);
      break;
     case LISTENING:
      bell_en = false;
      break;
    case HUNG_UP:
      bell_en = false;
      mp3_command(CMD_PAUSE, 0x0000);
      break;
  }

  // Finally set current state to next state
  current_ringer_state = next_state;
}

void mp3_command(int8_t command, int16_t dat) {
  int8_t frame[8] = { 0 };
  frame[0] = 0x7e;                // starting byte
  frame[1] = 0xff;                // version
  frame[2] = 0x06;                // The number of bytes of the command without starting byte and ending byte
  frame[3] = command;             //
  frame[4] = 0x01;                // 0x00 = no feedback, 0x01 = feedback
  frame[5] = (int8_t)(dat >> 8);  // data high byte
  frame[6] = (int8_t)(dat);       // data low byte
  frame[7] = 0xef;                // ending byte
  for (uint8_t i = 0; i < 8; i++) {
    mp3.write(frame[i]);
  }
}
