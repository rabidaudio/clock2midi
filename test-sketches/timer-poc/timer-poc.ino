#define MIDI_CLOCKS_PER_BEAT 24

bool ledOn = false;

void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);

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
  
  TCNT1 = 0; // reset counter
//  TIMSK1 |= (1 << TOIE1); // Turn overflow interrupt on

  
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
    / 4 /*phase+freq correct mode*/
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

  TCNT3 = 0; // reset counter
  TIMSK3 |= (1 << OCIE3A); // Turn compare A interrupt on

  interrupts();
}

void loop() {
  Serial.println(TCNT1);
  delay(100);
}

inline void sendMessage() {
  ledOn = !ledOn;
  digitalWrite(LED_BUILTIN, ledOn ? HIGH : LOW);
}

// When Timer3 triggers, output MIDI message (if less than MIDI_CLOCKS_PER_BEAT)
ISR(TIMER3_COMPA_vect)
{
  sendMessage();
}

// When Timer1 overflows, disable output and reset timer
ISR(TIMER1_OVF_vect)
{
//  sendMessage();
}
