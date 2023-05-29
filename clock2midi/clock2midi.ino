#include <Bounce.h>
#include "midi.h"

#define BTN_PIN 6
#define CLOCK_IN_PIN 7

Bounce btn = Bounce(BTN_PIN, 5);
Midi MIDI = Midi(&Serial1);

bool enabled = true;
size_t loopIdx = 0;

// the setup routine runs once when you press reset:
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BTN_PIN, INPUT);
  delay(5);
}

// the loop routine runs over and over again forever:
void loop() {
  btn.update();
  if (btn.risingEdge()) {
    enabled = !enabled;
    if (enabled) {
      MIDI.start();
    } else {
      MIDI.stop();
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
  if (enabled) {
    MIDI.clock();
    delay(21);
    digitalWrite(LED_BUILTIN, loopIdx >= 12 ? LOW : HIGH);
    loopIdx = (loopIdx + 1) % 24; // 24 pulses per quarter note
  }
}
