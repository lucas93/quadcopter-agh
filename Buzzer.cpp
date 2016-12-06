// 
// 
// 

#include "Buzzer.h"

bool BuzzerClass::setup()
{
	pinMode(buzzerPin, OUTPUT);
	return true;
}

void BuzzerClass::buzzSignal(String str)
{
	for (unsigned int k = 0; k < str.length(); k++)
	{
		char thischar = str[k];
		if (thischar >= 'a' && thischar <= 'z')
		{
			// convert to upper case
			thischar = thischar - 32;
		}
		if (thischar < 65 || thischar>90)
		{
			// swap for a space if not in A-Z
			thischar = ' ';
		}
		soundLetter(thischar);
		delay(lgap);
	}
}

void BuzzerClass::dash()
{
	tone(buzzerPin, tonefreq);
	Led.on();
	delay(dashlength);
	noTone(buzzerPin);
	Led.off();
	delay(inter);
}

void BuzzerClass::soundLetter(char letter)
{
	// letters are in order of frequency
	switch (letter)
	{
	case 'E': dot(); return;
	case 'T': dash(); return;
	case 'A': dot(); dash(); return;
	case 'O': dash(); dash(); dash(); return;
	case 'I': dot(); dot(); return;
	case 'N': dash(); dot(); return;
	case 'S': dot(); dot(); dot(); return;
	case 'H': dot(); dot(); dot(); dot(); return;
	case 'R': dot(); dash(); dot(); return;
	case 'D': dash(); dot(); dot(); return;
	case 'L': dot(); dash(); dot(); dot(); return;
	case 'C': dash(); dot(); dash(); dot(); return;
	case 'U': dot(); dot(); dash(); return;
	case 'M': dash(); dash(); return;
	case 'W': dot(); dash(); dash(); return;
	case 'F': dot(); dot(); dash(); dot(); return;
	case 'G': dash(); dash(); dot(); return;
	case 'Y': dash(); dot(); dash(); dash(); return;
	case 'P': dot(); dash(); dash(); dot(); return;
	case 'B': dash(); dot(); dot(); dot(); return;
	case 'V': dot(); dot(); dot(); dash(); return;
	case 'K': dash(); dot(); dash(); return;
	case 'J': dot(); dash(); dash(); dash(); return;
	case 'X': dash(); dot(); dot(); dash(); return;
	case 'Q': dash(); dash(); dot(); dash(); return;
	case 'Z': dash(); dash(); dot(); dot(); return;
	case ' ': delay(wgap); return;
	}
}

void BuzzerClass::dot()
{
	tone(buzzerPin, tonefreq);
	Led.on();
	delay(dotlength);
	noTone(buzzerPin);
	Led.off();
	delay(inter);
}

void BuzzerClass::off()
{
	noTone(buzzerPin);
}

void BuzzerClass::on(int  toneFreq)
{
	tone(buzzerPin, tonefreq);
}

void BuzzerClass::toggle()
{
	static auto is_beeping = false;
	if (is_beeping = !is_beeping)
		on();
	else
		off();
}

BuzzerClass Buzzer;