
 
   int in_ADC0, in_ADC1;  //variables for 2 ADCs values (ADC0, ADC1)
   int POT0, POT1, POT2, out_DAC0, out_DAC1; //variables for 3 pots (ADC8, ADC9, ADC10)
   int LED = 3;
   int FOOTSWITCH = 7;
   int TOGGLE = 2;
   int toggle_value = 0;
   int effect=0;
   int upper_threshold, lower_threshold;
   #define MAX_DELAY 40000
   uint16_t sDelayBuffer0[MAX_DELAY];
   unsigned int Delay_Depth, DelayCounter = 0;
   #define MAX_DELAY 500
   #define MIN_DELAY 200
unsigned int count_up=1;
int p;
 
void setup()
{
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
  attachInterrupt(TOGGLE, switch_handler, CHANGE);
  pinMode(FOOTSWITCH, INPUT);    
}
 
void loop()
{
  //Read the ADCs
  while((ADC->ADC_ISR & 0x1CC0)!=0x1CC0);  // wait for ADC 0, 1, 8, 9, 10 conversion complete.
  in_ADC0=ADC->ADC_CDR[7];             // read data from ADC0
  in_ADC1=ADC->ADC_CDR[6];             // read data from ADC1 
  POT0=ADC->ADC_CDR[10];                   // read data from ADC8       
  POT1=ADC->ADC_CDR[11];                   // read data from ADC9  
  POT2=ADC->ADC_CDR[12];                   // read data from ADC10   
}
 
void TC4_Handler()
{
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
 
void switch_handler()
{
  delayMicroseconds(100000); //debouncing protection
  if (toggle_value!=digitalRead(TOGGLE)) effect++;
  delayMicroseconds(100000); //debouncing protection
  toggle_value=digitalRead(TOGGLE);
  if (effect==4) effect=0;
 
  Delay_Depth=300; //reset the variable.
}

