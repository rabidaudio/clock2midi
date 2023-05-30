#include <Bounce.h>

/* MIDI CONSTANTS */
#define MIDI_BAUD 31250
#define MIDI_CLOCKS_PER_BEAT 24
#define MIDI_MSG_START 0xFA
#define MIDI_MSG_CONTINUE 0xFB
#define MIDI_MSG_STOP 0xFC
#define MIDI_MSG_CLOCK 0xF8

/* PIN CONFIGS */
#define MIDI Serial1
#define CLOCK_IN_PIN 5
#define BTN_PIN 6
#define DEBOUNCE_TIME_MS 5
Bounce btn = Bounce(BTN_PIN, DEBOUNCE_TIME_MS);

//#define DEBUG 1

/* STATE */
// is the output disabled by the toggle button
#define STATE_DISABLED 0
#define STATE_ENABLING 2
#define STATE_ENABLED 1
volatile size_t state = STATE_DISABLED;
// is the clock input sending pulses
volatile bool triggering = false;
// how many clocks do we have left to send this beat
size_t ticksRemaining;

void setup()
{
  MIDI.begin(MIDI_BAUD);
#ifdef DEBUG
  Serial.begin(115200);
#endif

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BTN_PIN, INPUT);
  pinMode(CLOCK_IN_PIN, INPUT);

  noInterrupts();
  configureTimer1();
  configureTimer3();
  // Turn on interrupt for clock signal
  attachInterrupt(digitalPinToInterrupt(CLOCK_IN_PIN), onClock, RISING);
  interrupts();
}

void loop()
{
  btn.update();
  if (btn.risingEdge())
  {
    switch (state)
    {
      case STATE_DISABLED:
        // request to turn on on the next clock
        state = STATE_ENABLING;
        // turn the LED on to indicate something changed
        ledOn();
        break;
      case STATE_ENABLING:
        // cancel enabling
        state = STATE_DISABLED;
        ledOff();
        break;
      case STATE_ENABLED:
        // stop immediately
        state = STATE_DISABLED;
        midiStop();
        ledOff();
        break;
    }
  }

#ifdef DEBUG
  Serial.print(TCNT1);
  Serial.print("\t");
  Serial.print(TCNT3);
  Serial.print("\t");
  Serial.print(triggering ? 5000 : 0);
  Serial.print("\t");
  Serial.print(state * 3000);
  Serial.print("\t");
  Serial.println(state == STATE_ENABLED ? ticksRemaining * 1000 : 0);
  delay(10);
#endif
}

// When a clock signal comes in, check the status of timer1,
// set timer3 to that time / MIDI_CLOCKS_PER_BEAT,
// trigger any outstanding timers (if early), reset timer1
void onClock()
{
  noInterrupts();

  // if requested to turn on, turn on
  // and send a start message first
  if (state == STATE_ENABLING)
  {
    state = STATE_ENABLED;
    midiStart();
  }

  if (triggering)
  {
    // set T3 compare A to 1/24th of the time T1 reads
    OCR3A = convertTimer1toTimer3(TCNT1);
  }

  // restart timers
  TCNT1 = 0;
  TCNT3 = 0;

  if (triggering)
  {
    // flush any remaining clocks.
    // This is relatively safe to do in
    // an interrupt because it will take
    // at most 256us, and the minimum
    // timer interval is ~1.5ms.
    // still, since we have no control
    // over the external clock, we turn
    // interrupts off until we're done.
    while (ticksRemaining > 0)
      tick();
  }

  restartTicks();

  // tick(); // go ahead and do the first tick immediately
  // blink the LED. It will blink off on the first T3 interval
  if (state == STATE_ENABLED)
    ledOn();

  triggering = true;
  interrupts();
}

// When Timer3 triggers, output MIDI message (if less than MIDI_CLOCKS_PER_BEAT)
ISR(TIMER3_COMPA_vect)
{
  tick();
  ledOff(); // turn off the LED if on from clock interrupt
}

// When Timer1 overflows, disable output
ISR(TIMER1_OVF_vect)
{
  triggering = false;
  restartTicks();
}

// Timer3 is 1024/64 = 16x faster than Timer1.
// 1 cycle of Timer1 is 1 beat, and we need 24
// clock pulses per beat, so we then need to divide
// by 24. Ultimately then we just need to divide the
// counter by 24/(1024/64) = 1.5.
// The 24 factor for MIDI is very annoying. 16
// or 32 would have been very easy, but here
// division by a factor of 3 requires floating
// point math. Still, dividing a uint16_t by 1.5
// seems to take around 16us, so we'll just do
// it in an interrupt.
inline uint16_t convertTimer1toTimer3(uint16_t t1)
{
  return t1 / MIDI_CLOCKS_PER_BEAT * (1024 / 64);
}

inline void ledOn()
{
  digitalWrite(LED_BUILTIN, HIGH);
}

inline void ledOff()
{
  digitalWrite(LED_BUILTIN, LOW);
}

inline void midiStart()
{
  MIDI.write(MIDI_MSG_START);
}

inline void midiStop()
{
  MIDI.write(MIDI_MSG_STOP);
}

inline void midiClock()
{
  MIDI.write(MIDI_MSG_CLOCK);
}

inline void restartTicks()
{
  ticksRemaining = MIDI_CLOCKS_PER_BEAT;
}

inline void tick()
{
  if (!triggering)
    return;

  if (ticksRemaining > 0)
  {
    ticksRemaining--;
    if (state == STATE_ENABLED)
      midiClock();
  }
}

inline void configureTimer1()
{
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

  TCNT1 = 0;              // reset counter
  TIMSK1 |= (1 << TOIE1); // Turn overflow interrupt on
}

inline void configureTimer3()
{
  // Run Timer3 in CTC mode
  // with a prescaler of 64.
  // Top is OCR3A. On top interrupt, send msg.
  // TCCR3A = COM3A1 COM3A0 COM3B1 COM3B0 COM3C1 COM3C0 WGM31  WGM30
  // TCCR3B =  ICNC3  ICES3   -     WGM33  WGM32   CS32  CS31   CS30
  // TIMSK3 =   –       –    ICIE3    –   OCIE3C OCIE3B OCIE3A TOIE3
  // Clear registers
  TCCR3A = 0;
  TCCR3B = 0;
  // Compare A
  OCR3A = F_CPU / 64 /*prescaler*/ / (MIDI_CLOCKS_PER_BEAT * 2 /*Hz, ie 120 BPM*/);

  // Compare A on, B and C disconnected.
  // COMnA1/COMnB1/COMnC1 = 0
  // COMnA0/COMnB0/COMnC0 = 1
  // WGM13:0 = 8, 9, 10, or 11
  // Mode4: WGM33=0,WGM32=1,WGM31=0,WGM30=0
  // CTC, Top=OCRnA
  // Pre-scaler 64: CS32=0,CS31=1,CS30=1
  TCCR3A |= (0 << COM3A1) | (1 << COM3A0) | (0 << WGM31) | (0 << WGM30);
  TCCR3B |= (0 << WGM33) | (1 << WGM32) | (0 << CS32) | (1 << CS31) | (1 << CS30);

  TCNT3 = 0;               // reset counter to trigger on the next tick
  TIMSK3 |= (1 << OCIE3A); // Turn compare A interrupt on
}
