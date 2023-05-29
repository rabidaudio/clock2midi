
#define MIDI_MSG_START 0xFA
#define MIDI_MSG_CONTINUE 0xFB
#define MIDI_MSG_STOP 0xFC
#define MIDI_MSG_CLOCK 0xF8

class Midi
{
  private:
    HardwareSerial* _serial;
  public:

    Midi(HardwareSerial* serial)
    {
      _serial = serial;
      _serial->begin(31250);
    }

    void start()
    {
      _serial->write(MIDI_MSG_START);
    }

    void stop()
    {
      _serial->write(MIDI_MSG_STOP);
    }

    void clock()
    {
      _serial->write(MIDI_MSG_CLOCK);
    }
};
