
#include <Wire.h>
#include <PN532_I2C.h>
#include "PN532.h"

// If using the breakout with SPI, define the pins for SPI communication.
#define PN532_SCK  (2)
#define PN532_MOSI (3)
#define PN532_SS   (4)
#define PN532_MISO (5)

// If using the breakout or shield with I2C, define just the pins connected
// to the IRQ and reset lines.  Use the values below (2, 3) for the shield!
#define PN532_IRQ   (2)
#define PN532_RESET (3)  // Not connected by default on the NFC Shield

// Uncomment just _one_ line below depending on how your breakout or shield
// is connected to the Arduino:

// Use this line for a breakout with a software SPI connection (recommended):
//Adafruit_PN532 nfc(PN532_SCK, PN532_MISO, PN532_MOSI, PN532_SS);

// Use this line for a breakout with a hardware SPI connection.  Note that
// the PN532 SCK, MOSI, and MISO pins need to be connected to the Arduino's
// hardware SPI SCK, MOSI, and MISO pins.  On an Arduino Uno these are
// SCK = 13, MOSI = 11, MISO = 12.  The SS line can be any digital IO pin.
//Adafruit_PN532 nfc(PN532_SS);

// Or use this line for a breakout or shield with an I2C connection:
//Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);
PN532_I2C pn532i2c(Wire);
PN532 nfc(pn532i2c);
#if defined(ARDUINO_ARCH_SAMD)
// for Zero, output on USB Serial console, remove line below if using programming port to program the Zero!
// also change #define in Adafruit_PN532.cpp library file
#define Serial SerialUSB
#endif

int in_ADC0, in_ADC1;  //variables for 2 ADCs values (ADC0, ADC1)
int POT0, POT1, POT2, out_DAC0, out_DAC1; //variables for 3 pots (ADC8, ADC9, ADC10)
int LED = 3;
int FOOTSWITCH = 7;
int TOGGLE = 2;
int toggle_value = 0;
int effect=0;
int upper_threshold, lower_threshold;
uint16_t sDelayBuffer0[40000];
unsigned int Delay_Depth, DelayCounter = 0;
#define MAX_DELAY 500
#define MIN_DELAY 200
unsigned int count_up=1;
int p;
int louis=0;

//RFID Vars
uint8_t success;
uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
uint8_t uidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)

void setup()
{
  pinMode(13, OUTPUT);
  //RFID CODE
#ifndef ESP8266
  while (!Serial); // for Leonardo/Micro/Zero
#endif
  //Serial.begin(115200);
  Serial.println("Hello!");

  nfc.begin();
  delay(2000);

  uint32_t versiondata = nfc.getFirmwareVersion();
  if (! versiondata) {
    Serial.print("Didn't find PN53x board");

  }
  // Got ok data, print it out!
  Serial.print("Found chip PN5"); Serial.println((versiondata>>24) & 0xFF, HEX);
  Serial.print("Firmware ver. "); Serial.print((versiondata>>16) & 0xFF, DEC);
  Serial.print('.'); Serial.println((versiondata>>8) & 0xFF, DEC);

<<<<<<< Updated upstream
  // configure board to read RFID tags
  nfc.SAMConfig();
  nfc.setPassiveActivationRetries(0x01); 
=======
	// configure board to read RFID tags
  nfc.setPassiveActivationRetries(0x01);  
	nfc.SAMConfig();
>>>>>>> Stashed changes

  Serial.println("Waiting for an ISO14443A Card ...");


  //PEDAL SHIELD CODE
  /* turn on the timer clock in the power management controller */
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk(ID_TC4);

  /* we want wavesel 01 with RC */
  TC_Configure(/* clock */TC1,/* channel */1, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC
      | TC_CMR_TCCLKS_TIMER_CLOCK2);
  TC_SetRC(TC1, 1, 238); // sets <> 44.1 Khz interrupt rate
  //TC_SetRC(TC1, 1, 109); // sets <>   96 Khz interrupt rate

  TC_Start(TC1, 1);

  // enable timer interrupts on the timer
  TC1->TC_CHANNEL[1].TC_IER=TC_IER_CPCS;
  TC1->TC_CHANNEL[1].TC_IDR=~TC_IER_CPCS;

  /* Enable the interrupt in the nested vector interrupt controller */
  /* TC4_IRQn where 4 is the timer number * timer channels (3) + the channel number
     (=(1*3)+1) for timer1 channel1 */
  NVIC_EnableIRQ(TC4_IRQn);

  //ADC Configuration
  ADC->ADC_MR |= 0x80;   // DAC in free running mode.
  ADC->ADC_CR=2;         // Starts ADC conversion.
  ADC->ADC_CHER=0x1CC0;    // Enable ADC channels 0 and 1.

  //DAC Configuration
  analogWrite(DAC0,0);  // Enables DAC0
  analogWrite(DAC1,0);  // Enables DAC0



  //configure pin2 as an input and enable the internal pull-up resistor
  pinMode(LED, OUTPUT);
  pinMode(TOGGLE, INPUT_PULLUP);
  pinMode(FOOTSWITCH, INPUT);
}

void loop()
{

<<<<<<< Updated upstream
  // RFID CODE
  // Wait for an ISO14443A type cards (Mifare, etc.).  When one is found
  // 'uid' will be populated with the UID, and uidLength will indicate
  // if the uid is 4 bytes (Mifare Classic) or 7 bytes (Mifare Ultralight)
  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, 0);

  if (success) {
    // Display some basic information about the card
    Serial.println("Found an ISO14443A card");
    Serial.print("  UID Length: ");Serial.print(uidLength, DEC);Serial.println(" bytes");
    Serial.print("  UID Value: ");
    nfc.PrintHex(uid, uidLength);
    Serial.println("");

    //0xF0 0x1B 0xC6 0xDE
    if (uid[0] == 0xF0 &&
        uid[1] == 0x1B &&
        uid[2] == 0xC6 &&
        uid[3] == 0xDE) {
      Serial.println("keychain tag read 2");
      effect = 2;
      Serial.println(effect);
    }
    //0x82 0xA9 0x40 0xF2
    if (uid[0] == 0x82 &&
        uid[1] == 0xA9 &&
        uid[2] == 0x40 &&
        uid[3] == 0xF2) {
      Serial.println("card tag read 1");
      effect = 1;
      Serial.println(effect);
    }

  }

  //PEDAL SHIELD CODE
  //Read the ADCs
=======
	// RFID CODE
	// Wait for an ISO14443A type cards (Mifare, etc.).  When one is found
	// 'uid' will be populated with the UID, and uidLength will indicate
	// if the uid is 4 bytes (Mifare Classic) or 7 bytes (Mifare Ultralight)
	success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, 1000);

	if (success) {
		// Display some basic information about the card
		Serial.println("Found an ISO14443A card");
		Serial.print("  UID Length: ");Serial.print(uidLength, DEC);Serial.println(" bytes");
		Serial.print("  UID Value: ");
		nfc.PrintHex(uid, uidLength);
		Serial.println("");

		//0xF0 0x1B 0xC6 0xDE
		if (uid[0] == 0xF0 &&
				uid[1] == 0x1B &&
				uid[2] == 0xC6 &&
				uid[3] == 0xDE) {
			Serial.println("keychain tag read 2");
			effect = 2;
			Serial.println(effect);
		}
		//0x82 0xA9 0x40 0xF2
		if (uid[0] == 0x82 &&
				uid[1] == 0xA9 &&
				uid[2] == 0x40 &&
				uid[3] == 0xF2) {
			Serial.println("card tag read 1");
			effect = 1;
			Serial.println(effect);
		}

	}
>>>>>>> Stashed changes

}


void TC4_Handler()
{
  while ((ADC->ADC_ISR & 0x1CC0)!=0x1CC0);  // wait for ADC 0, 1, 8, 9, 10 conversion complete.
  in_ADC0=ADC->ADC_CDR[7];             // read data from ADC0
  in_ADC1=ADC->ADC_CDR[6];             // read data from ADC1
  POT0=ADC->ADC_CDR[10];                   // read data from ADC8
  POT1=ADC->ADC_CDR[11];                   // read data from ADC9
  POT2=ADC->ADC_CDR[12];                   // read data from ADC10
<<<<<<< Updated upstream
  //digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
  //delay(1000);              // wait for a second
  //digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
  //delay(1000);              // wait for a second

  // We need to get the status to clear it and allow the interrupt to fire again
  TC_GetStatus(TC1, 1);

  if (effect==0) // EFFECT 0: Volume-Booster
  {
    digitalWrite(LED, HIGH);
    //Adjust the volume with POT2
    out_DAC0=map(in_ADC0,0,4095,1,POT2);
    out_DAC1=map(in_ADC1,0,4095,1,POT2);

    //Write the DACs
    dacc_set_channel_selection(DACC_INTERFACE, 0);          //select DAC channel 0
    dacc_write_conversion_data(DACC_INTERFACE, out_DAC0);//write on DAC
    dacc_set_channel_selection(DACC_INTERFACE, 1);          //select DAC channel 1
    dacc_write_conversion_data(DACC_INTERFACE, out_DAC1);//write on DAC
  }


  else if (effect==1)  // EFFECT 1: Asymmetric Distortion
  {
    digitalWrite(LED, LOW);
    upper_threshold=map(POT0,0,4095,4095,2047);
    lower_threshold=map(POT1,0,4095,0000,2047);

    if(in_ADC0>=upper_threshold) in_ADC0=upper_threshold;
    else if(in_ADC0<lower_threshold)  in_ADC0=lower_threshold;

    if(in_ADC1>=upper_threshold) in_ADC1=upper_threshold;
    else if(in_ADC1<lower_threshold)  in_ADC1=lower_threshold;

    //adjust the volume with POT2
    out_DAC0=map(in_ADC0,0,4095,1,POT2);
    out_DAC1=map(in_ADC1,0,4095,1,POT2);

    //Write the DACs
    dacc_set_channel_selection(DACC_INTERFACE, 0);          //select DAC channel 0
    dacc_write_conversion_data(DACC_INTERFACE, out_DAC0);//write on DAC
    dacc_set_channel_selection(DACC_INTERFACE, 1);          //select DAC channel 1
    dacc_write_conversion_data(DACC_INTERFACE, out_DAC1);//write on DAC
  }
  else if (effect==2) // EFFECT 2: Echo.
  {
    digitalWrite(LED, LOW);
    //Store current readings
    sDelayBuffer0[DelayCounter]  = (in_ADC0 + (sDelayBuffer0[DelayCounter]))>>1;

    //Adjust Delay Depth based in pot0 position.
    Delay_Depth =map(POT0>>2,0,2047,1,40000);

    //Increse/reset delay counter.
    DelayCounter++;
    if(DelayCounter >= Delay_Depth) DelayCounter = 0;
    out_DAC0 = ((sDelayBuffer0[DelayCounter]));

    //Add volume feature based in POT2 position.
    out_DAC0=map(out_DAC0,0,4095,1,POT2);

    //Write the DACs
    dacc_set_channel_selection(DACC_INTERFACE, 0);          //select DAC channel 0
    dacc_write_conversion_data(DACC_INTERFACE, out_DAC0);//write on DAC
    dacc_set_channel_selection(DACC_INTERFACE, 1);          //select DAC channel 1
    dacc_write_conversion_data(DACC_INTERFACE, 0);          //write on DAC
  }

  else  // EFFECT 3: Chourus
  {
    digitalWrite(LED, LOW);
    //Store current readings
    sDelayBuffer0[DelayCounter] = in_ADC0;

    //Adjust Delay Depth based in pot0 position.
    POT0=map(POT0>>2,0,1024,1,25); //25 empirically chosen

    DelayCounter++;
    if(DelayCounter >= Delay_Depth)
    {
      DelayCounter = 0;
      if(count_up)
      {
        digitalWrite(LED, HIGH);
        for(p=0;p<POT0+1;p++)
          sDelayBuffer0[Delay_Depth+p]=sDelayBuffer0[Delay_Depth-1];
        Delay_Depth=Delay_Depth+POT0;
        if (Delay_Depth>=MAX_DELAY)count_up=0;
      }
      else
      {
        digitalWrite(LED, LOW);
        Delay_Depth=Delay_Depth-POT0;
        if (Delay_Depth<=MIN_DELAY)count_up=1;
      }
    }

    out_DAC0 = sDelayBuffer0[DelayCounter];

    //Add volume control based in POT2
    out_DAC0=map(out_DAC0,0,4095,1,POT2);

    //Write the DACs
    dacc_set_channel_selection(DACC_INTERFACE, 0);       //select DAC channel 0
    dacc_write_conversion_data(DACC_INTERFACE, out_DAC0);//write on DAC
    dacc_set_channel_selection(DACC_INTERFACE, 1);       //select DAC channel 1
    dacc_write_conversion_data(DACC_INTERFACE, 0);       //write on DAC

  }}
=======

	// We need to get the status to clear it and allow the interrupt to fire again
	TC_GetStatus(TC1, 1);

	if (effect==0) // EFFECT 0: Volume-Booster
	{
		digitalWrite(LED, HIGH);
		//Adjust the volume with POT2
		out_DAC0=map(in_ADC0,0,4095,1,POT2);
		out_DAC1=map(in_ADC1,0,4095,1,POT2);

		//Write the DACs
		dacc_set_channel_selection(DACC_INTERFACE, 0);          //select DAC channel 0
		dacc_write_conversion_data(DACC_INTERFACE, out_DAC0);//write on DAC
		dacc_set_channel_selection(DACC_INTERFACE, 1);          //select DAC channel 1
		dacc_write_conversion_data(DACC_INTERFACE, out_DAC1);//write on DAC
	}


	else if (effect==1)  // EFFECT 1: Asymmetric Distortion
	{
		digitalWrite(LED, LOW);
		upper_threshold=map(POT0,0,4095,4095,2047);
		lower_threshold=map(POT1,0,4095,0000,2047);

		if(in_ADC0>=upper_threshold) in_ADC0=upper_threshold;
		else if(in_ADC0<lower_threshold)  in_ADC0=lower_threshold;

		if(in_ADC1>=upper_threshold) in_ADC1=upper_threshold;
		else if(in_ADC1<lower_threshold)  in_ADC1=lower_threshold;

		//adjust the volume with POT2
		out_DAC0=map(in_ADC0,0,4095,1,POT2);
		out_DAC1=map(in_ADC1,0,4095,1,POT2);

		//Write the DACs
		dacc_set_channel_selection(DACC_INTERFACE, 0);          //select DAC channel 0
		dacc_write_conversion_data(DACC_INTERFACE, out_DAC0);//write on DAC
		dacc_set_channel_selection(DACC_INTERFACE, 1);          //select DAC channel 1
		dacc_write_conversion_data(DACC_INTERFACE, out_DAC1);//write on DAC
	}
	else if (effect==2) // EFFECT 2: Echo.
	{
		digitalWrite(LED, LOW);
		//Store current readings
		sDelayBuffer0[DelayCounter]  = (in_ADC0 + (sDelayBuffer0[DelayCounter]))>>1;

		//Adjust Delay Depth based in pot0 position.
		Delay_Depth =map(POT0>>2,0,2047,1,40000);

		//Increse/reset delay counter.
		DelayCounter++;
		if(DelayCounter >= Delay_Depth) DelayCounter = 0;
		out_DAC0 = ((sDelayBuffer0[DelayCounter]));

		//Add volume feature based in POT2 position.
		out_DAC0=map(out_DAC0,0,4095,1,POT2);

		//Write the DACs
		dacc_set_channel_selection(DACC_INTERFACE, 0);          //select DAC channel 0
		dacc_write_conversion_data(DACC_INTERFACE, out_DAC0);//write on DAC
		dacc_set_channel_selection(DACC_INTERFACE, 1);          //select DAC channel 1
		dacc_write_conversion_data(DACC_INTERFACE, 0);          //write on DAC
	}

	else  // EFFECT 3: Chourus
	{
		digitalWrite(LED, LOW);
		//Store current readings
		sDelayBuffer0[DelayCounter] = in_ADC0;

		//Adjust Delay Depth based in pot0 position.
		POT0=map(POT0>>2,0,1024,1,25); //25 empirically chosen

		DelayCounter++;
		if(DelayCounter >= Delay_Depth)
		{
			DelayCounter = 0;
			if(count_up)
			{
				digitalWrite(LED, HIGH);
				for(p=0;p<POT0+1;p++)
					sDelayBuffer0[Delay_Depth+p]=sDelayBuffer0[Delay_Depth-1];
				Delay_Depth=Delay_Depth+POT0;
				if (Delay_Depth>=MAX_DELAY)count_up=0;
			}
			else
			{
				digitalWrite(LED, LOW);
				Delay_Depth=Delay_Depth-POT0;
				if (Delay_Depth<=MIN_DELAY)count_up=1;
			}
		}

		out_DAC0 = sDelayBuffer0[DelayCounter];

		//Add volume control based in POT2
		out_DAC0=map(out_DAC0,0,4095,1,POT2);

		//Write the DACs
		dacc_set_channel_selection(DACC_INTERFACE, 0);       //select DAC channel 0
		dacc_write_conversion_data(DACC_INTERFACE, out_DAC0);//write on DAC
		dacc_set_channel_selection(DACC_INTERFACE, 1);       //select DAC channel 1
		dacc_write_conversion_data(DACC_INTERFACE, 0);       //write on DAC

	}}
>>>>>>> Stashed changes

void switch_handler()
{
  delayMicroseconds(100000); //debouncing protection
  if (toggle_value!=digitalRead(TOGGLE)) effect++;
  delayMicroseconds(100000); //debouncing protection
  toggle_value=digitalRead(TOGGLE);
  if (effect==4) effect=0;

  Delay_Depth=300; //reset the variable.
}

