// Piezo Element buzzer with Morse Code Sketch used

#ifndef _BUZZER_h
#define _BUZZER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "Led.h"
class BuzzerClass
{
 private:
	 const int buzzerPin = 10;
 	 int tonefreq = 1000;	// tone frequency C

	 // constants for tone and rest durations
	 const int dotlength = 100;
	 const int dashlength = dotlength * 3;
	 // inter-element gap - between each dot or dash of a letter
	 const int inter = dotlength;
	 // letter gap is 3 dots - the inter gap is always added - so this is one fewer
	 const int lgap = dotlength * 2; // inter-letter gap
									 // word gap is 7 dots - with letter and inter gap already counted, this is -1
	 const int wgap = dotlength * 4; //inter-word gap


 public:
	 bool setup();

	 void buzzSignal(String str);	// buzz a message with Morse Code
	 void dash();
	 void soundLetter(char letter);
	 void dot();
	 void off();
	 void on(int  toneFreq = 1000);
	 void toggle();
};

extern BuzzerClass Buzzer;

#endif

