// Button state
struct Button {
  int pin;
  boolean repeat;
  int lastReading;
  boolean fired;
  unsigned long nextFireTime;
};

// Returns a new button attached to the given pin with the given debounce delay.
// The button will only fire once when it's pressed - it must be depressed before it will fire again.
Button newSingleButton(int pin);

// Returns a new button attached to the given pin with the given initial and repeat debounce delays.
// When the button is held, it will fire after the initial debounce delay, then fire repeatedly every repeat debounce delay.
Button newRepeatButton(int pin);

// Reads the button state, storing the current state in the button.
// Returns true if the button fired during this reading.
boolean readButton(Button* button, unsigned long now);

