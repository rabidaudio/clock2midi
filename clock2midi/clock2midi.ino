#include <Bounce.h>
#include "midi.h"

#define MIDI_CLOCKS_PER_BEAT 24
#define STARTUP_TIME_MS 0 // 6000
#define CLOCK_IN_PIN 5
#define BTN_PIN 6
#define DEBUG 1

Bounce btn = Bounce(BTN_PIN, 5);
Midi MIDI = Midi(&Serial1);

// is the output disabled by the toggle button
volatile bool enabled = false;
// is the clock input sending pulses
volatile bool triggering = false;
// how many clocks do we have left to send this beat
size_t ticksRemaining;

// the setup routine runs once when you press reset:
void setup() {
#ifdef DEBUG
  Serial.begin(115200);
#endif

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BTN_PIN, INPUT);
  pinMode(CLOCK_IN_PIN, INPUT);

  noInterrupts();

  // Run Timer1 in normal mode, with a presaler of 1024.
  // Use overflow interrupt, disable compare matching.
  // TCCR1A = COM1A1 COM3A0 COM1B1 COM1B0 COM1C1 COM1C0 WGM11  WGM10
  // TCCR1B =  ICNC1  ICES1   -     WGM13  WGM12   CS12  CS11   CS10
  // TIMSK1 =   –       –    ICIE1    –   OCIE1C OCIE1B OCIE1A TOIE1
  // Clear registers
  TCCR1A = 0;
  TCCR1B = 0;
  // disconnect compares
  // COM1A1/COM1B1/COM1C1 = 0
  // COM1A0/COM1B0/COM1C0 = 0
  // Mode 0 (normal)
  // WGM13=0,WGM12=0,WGM11=0,WGM10=0
  // Prescaler 1024: CS12=1, CS11=0, CS10=1
  TCCR1A |= (0 << COM1A1) | (0 << COM3A0) | (0 << WGM11) | (0 << WGM10);
  TCCR1B |= (0 << WGM13) | (0 << WGM12) | (1 << CS12) | (0 << CS11) | (1 << CS10);
  
  // Run Timer3 in phase-correct PWM mode
  // with a prescaler of 64.
  // Top is OCR3A. On top interrupt, send msg.
  // TCCR3A = COM3A1 COM3A0 COM3B1 COM3B0 COM3C1 COM3C0 WGM31  WGM30
  // TCCR3B =  ICNC3  ICES3   -     WGM33  WGM32   CS32  CS31   CS30
  // TIMSK3 =   –       –    ICIE3    –   OCIE3C OCIE3B OCIE3A TOIE3
  // Clear registers
  TCCR3A = 0;
  TCCR3B = 0;
  // Compare A
  OCR3A = F_CPU / 64 /*prescaler*/
    / 2 /*phase+freq correct mode*/
    / (MIDI_CLOCKS_PER_BEAT * 2 /*Hz, ie 120 BPM*/) - 1;

  // Phase and frequency correct PWM.
  // Compare A on, B and C disconnected.
  // COMnA1/COMnB1/COMnC1 = 0
  // COMnA0/COMnB0/COMnC0 = 1
  // WGM13:0 = 8, 9, 10, or 11
  // Mode9: WGMn3=1,WGMn2=0,WGMn1=0,WGMn0=1
  // PWM, Phase and Frequency Correct, Top=OCRnA
  // Pre-scaler 64: CSn2=0,CSn1=1,CSn0=1
  TCCR3A |= (0 << COM3A1) | (1 << COM3A0) | (0 << WGM31) | (1 << WGM30);
  TCCR3B |= (1 << WGM33) | (0 << WGM32) | (0 << CS32) | (1 << CS31) | (1 << CS30);


  TCNT1 = 0; // reset counter
  TIMSK1 |= (1 << TOIE1); // Turn overflow interrupt on

  TCNT3 = 0; // reset counter to trigger on the next tick
  TIMSK3 |= (1 << OCIE3A); // Turn compare A interrupt on
  
  // Turn Int0 interrupt on for rising edge
  attachInterrupt(0, onClock, RISING);

  interrupts();

  delay(STARTUP_TIME_MS); // give the synth plenty of time to startup
  enabled = true;
}

// the loop routine runs over and over again forever:
void loop() {
  // simulate clock
  triggering = true;
  if (enabled && TCNT1 >= 7810) onClock();
  
  btn.update();
  if (btn.risingEdge()) {
    if (!enabled) {
      // send the start message before enabling the clock
      MIDI.start();
      enabled = true;
    } else {
      // stop the clock before sending the stop message
      enabled = false;
      MIDI.stop();

      // turn the LED off
      digitalWrite(LED_BUILTIN, LOW);
    }
  }

  #ifdef DEBUG
  Serial.print(TCNT1); Serial.print("\t"); Serial.println(TCNT3);
  delay(10);
  #endif
}

// When a clock signal comes in, check the status of timer1,
// set timer3 to that time / MIDI_CLOCKS_PER_BEAT,
// trigger any outstanding timers (if early), reset timer1
void onClock()
{
  noInterrupts();

  if (!triggering) {
    // restart timers
    TCNT1 = 0;
    TCNT3 = 0;
    triggering = true;
    // go ahead and beat the first
    MIDI.clock();
    // restart the beat counter
    ticksRemaining = MIDI_CLOCKS_PER_BEAT - 1;
    interrupts();
    return;
  }

  // set T3 compare A to 1/24th of the time T1 reads
  OCR3A = convertTimer1toTimer3(TCNT1);
  // restart both timers
  TCNT3 = 0;
  TCNT1 = 0;

  if(enabled) {
    // flush any remaining clocks.
    // This is relatively safe to do in
    // an interrupt because it will take
    // at most 256us, and the minimum
    // timer interval is ~1.5ms.
    // still, since we have no control
    // over the external clock, we turn
    // interrupts off until we're done.
    while (ticksRemaining > 0) {
      ticksRemaining--;
      MIDI.clock();
    }

    // go ahead and beat the first
    MIDI.clock();
    // restart the beat counter
    ticksRemaining = MIDI_CLOCKS_PER_BEAT - 1;
    // blink the LED. It will blink off on the first T3 interval
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    // restart the beat counter.
    // we still want to keep track of the counter as if we
    // were outputing clocks because we can be turned on again at
    // any time.
    ticksRemaining = MIDI_CLOCKS_PER_BEAT - 1;
  }
  interrupts();
}

// Timer3 is 1024/64 = 16x faster than Timer1.
// But also Timer3's counter value is 1/2 period,
// because it counts both up and down per cycle.
// 1 cycle of Timer1 is 1 beat, and we need 24
// clock pulses per beat, so we then need to divide
// by 24. Ultimately then we just need to divide the
// counter by 24*2/(1024/64) = 3.
// The 24 factor for MIDI is very annoying. 16
// or 32 would have been very easy, but here
// division by a factor of 3 requires floating
// point math. Still, dividing a uint16_t by 3
// seems to take around 16us, so we'll just do
// it in an interrupt.
inline uint16_t convertTimer1toTimer3(uint16_t t1)
{
  return t1 / MIDI_CLOCKS_PER_BEAT * (1024 / 64) / 2;
}

// When Timer3 triggers, output MIDI message (if less than MIDI_CLOCKS_PER_BEAT)
ISR(TIMER3_COMPA_vect)
{
  if (!enabled) return;
  if (!triggering) return;

  if (ticksRemaining > 0) {
    ticksRemaining--;
    MIDI.clock();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off the LED if on from clock interrupt
}

// When Timer1 overflows, disable output
ISR(TIMER1_OVF_vect)
{
  triggering = false;
}
