#include <BlockDriver.h>
#include <FreeStack.h>
#include <MinimumSerial.h>
#include <SdFat.h>
#include <SdFatConfig.h>
#include <SysCall.h>
#include <Wire.h>
#include <RTClib.h>
#include <SPI.h>
#include <AccelStepper.h>
#include <avr/wdt.h>            // library for default watchdog functions
#include <avr/interrupt.h>      // library for interrupts handling
#include <avr/sleep.h>          // library for sleep
#include <avr/power.h>          // library for power control
//Watchdog code modified from http://folk.uio.no/jeanra/Microelectronics/ArduinoWatchdog.html

//setup global variables
//pins used are D3/D4/D5/D6 = stepper, A0 (14) = servo, A4 and A5 (18,19) = RTC, D10,D11,D12,D13 = SDcard. update this.
//Motor code from http://42bots.com/tutorials/28byj-48-stepper-motor-with-uln2003-driver-and-arduino-uno/
#define HALFSTEP 8
#define FULLSTEP 4
#define motorPin1  A3     // IN1 on the ULN2003 driver 1
#define motorPin2  A2     // IN2 on the ULN2003 driver 1
#define motorPin3  A1     // IN3 on the ULN2003 driver 1
#define motorPin4  A0     // IN4 on the ULN2003 driver 1
#define switchmotorPin1  6     // IN1 on the ULN2003 driver 1
#define switchmotorPin2  7     // IN2 on the ULN2003 driver 1
#define switchmotorPin3  8     // IN3 on the ULN2003 driver 1
#define switchmotorPin4  9     // IN4 on the ULN2003 driver 1

//SD Card, motor and clock variables
SdFat SD;
File Logfile;
int pinCS = 10;
AccelStepper bottlestepper(FULLSTEP, motorPin1, motorPin3, motorPin2, motorPin4);
AccelStepper switchstepper(FULLSTEP, switchmotorPin1, switchmotorPin3, switchmotorPin2, switchmotorPin4);
RTC_DS3231  rtc;
//char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

//*****************
//General variables
//*****************
boolean sampleRunComplete =false;
byte sampleCounter = 1; // used to set the initial sampler ID. Could change if you want to start at some other ID.
byte activeDay;
int startMonth;
byte sampletime = 9; // Change this to the hour you want the new sample to begin (typically 9 am)(24hour time).
byte monthFuzzyStart = 25; //Change this depending on how much slop you want at the start of each month. eg: you may setup the sampler on the 27th due to logistics, but don't want that month to only collect from the 27th to 30th)
//Platter rotation variables
short outersteps = 3105; //adjust value to tweak outer ring sample steps. //first calc was 3088
short innersteps =  3795; //adjust value to tweak inner ring sample steps. //first calc was 3774
short changeoversteps = 350; //-4500; //adjust value to tweak changeover from outer to inner sample ring. (note. Goes the opposite way to the inner and outersteps).
//Water switch adjustment variables
short position1 = 50; //Should be 0. This is the starting position for the water switch
short position2 = 650; //Second position - adjust to align so waterswitch fills outer ring and month 2
short position3 = 1300; //Third position - adjust to align so waterswitch fills inner ring and month 2
short position4 = 1900; //Fourth position - adjust to align so waterswitch fills inner ring and month 3.
byte resetswitchpin = 3;
bool resettoggle = HIGH;//tipping bucket variables
// Tipping bucket sensor variables
const byte interruptsensor = 2;
volatile int tipcounter = 0; 
long debounceInterval = 50000; //Debouncing Time in microseconds
volatile unsigned long last_micros = 0;
volatile bool BucketTipped = false;
short tips = 0; //used in tippingSensorCheck
//int debounceInterval = 50;
short maxtipcount = 100;// 50; //Determine how many tips are allowed if using a tipping bucket sensor. eg: 10mm of rain per bottle, as 0.2mm per tip = 50.
//add servomax and servo min angles here for easy configuration.
short mintipcount = 2; // Used to provide a buffer in case of tiny rainfall amounts, morning dew, etc. 0 means it will start a new sample with even a single tip. 3 means it requires 3 tips to qualify as a new sample. 

//*****************
void setup() {
//*****************
//Setup serial output for monitoring.
 delay (3000); //wait 3 seconds for terminal to wake up.
//  Serial.begin(115200); //remove post debug
    Serial.println(F("MARS-02 activated."));

//Setup tipping bucket sensor
  pinMode(interruptsensor, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptsensor), tipinterrupt, FALLING);
  tipcounter = 0;

// configure the watchdog 
  configure_wdt();

//Setup RTC
    if (! rtc.begin()) {
      Serial.println(F("Couldn't find RTC"));
      while (1);
    }
   //rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); //use to force an RTC correction. Comment out after use.
if (rtc.lostPower()) {
    Serial.println(F("RTC lost power, lets set the time!"));
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(2000, 1, 1, 0, 0, 0));
    //This line sets the RTC with an explicit date & time, for example to set
    //January 21, 2014 at 3am you would call:
    //rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
} //end RTC
     DateTime Timestamp = rtc.now();
     Serial.print(F("Current time: ")); //remove serial output post debug
     Serial.print(Timestamp.day(), DEC);
     Serial.print("/");
     Serial.print(Timestamp.month(), DEC);
     Serial.print("/");
     Serial.print(Timestamp.year(), DEC);
     Serial.print("\t");     
     Serial.print(Timestamp.hour(), DEC);
     Serial.print(":");
     Serial.print(Timestamp.minute(), DEC);
     Serial.print(":");
     Serial.print(Timestamp.second(), DEC);
     Serial.println(F("\t(DD/MM/YY HH:MM:SS)"));

//set the starting month (with some fuzziness in case the sampler is setup early)
setStartMonth();

monthDelta();

//setup parameters for the stepper motors
  switchstepper.setMaxSpeed(500);
  switchstepper.setAcceleration(50.0);
  switchstepper.setSpeed (-300);
  bottlestepper.setMaxSpeed(1000);
  bottlestepper.setAcceleration(100.0);

//and the microswitch
  pinMode(resetswitchpin, INPUT_PULLUP);
  
  BottleOACW(); //test back and forth movement
  BottleOCW();
  Serial.println(F("MARS-01 bottlestepper activated."));

  //Here we need it to call the reset function for the water switch
  resetwaterswitch();
  waterSwitchPos(1); //reset to position 1.
  Serial.println(F("Water switch reset to starting position"));

//Setup the SD card. Make sure you disable the stepper motors before trying to setup the card, otherwise it will fail (guessing too much power usage).
  pinMode(pinCS, OUTPUT);
  digitalWrite(10, HIGH);
    if (SD.begin())
  {
    Serial.println(F("SD card is ready to use."));
  } else
  {
    Serial.println(F("SD card initialization failed"));
  waterSwitchPos(4);
  waterSwitchPos(1);
  waterSwitchPos(4);
    return;
  }
  
  // Create/Open file 
  Logfile = SD.open("MARS-01.txt", FILE_WRITE);
     if (Logfile){
     DateTime Timestamp = rtc.now();
     Serial.print(F("New sample run commenced on: "));
     Serial.print(Timestamp.day(), DEC);
     Serial.print("/");
     Serial.print(Timestamp.month(), DEC);
     Serial.print("/");
     Serial.print(Timestamp.year(), DEC);
     Serial.print(" ");     
     Serial.print(Timestamp.hour(), DEC);
     Serial.print(":");
     Serial.print(Timestamp.minute(), DEC);
     Serial.print(":");
     Serial.print(Timestamp.second(), DEC);
     Serial.println(F("\t(DD/MM/YY HH:MM:SS)"));
     Logfile.println();
     Logfile.print(F("New sample run commenced on: "));
     format (Timestamp.day());
     Logfile.print("/");
     format (Timestamp.month());
     Logfile.print("/");
     format (Timestamp.year());
     Logfile.print("\t");     
     format (Timestamp.hour());
     Logfile.print(":");
     format (Timestamp.minute());
     Logfile.print(":");
     format (Timestamp.second());
     Logfile.println(F("\t(Date Format: DD/MM/YY HH:MM:SS)"));
     Logfile.println("Sample ID\tWater Switch Position\tCollection start date&time\tCollection end date&time\tTipping count\tAverage Temperature\tAverage Humidity");
     Logfile.close();
     } else {
  // We need a way to alert to SD card errors. What we'll do is toggle the water switch 3 times if the SD card doesn't work
  waterSwitchPos(4);
  waterSwitchPos(1);
  waterSwitchPos(4);
  waterSwitchPos(1);
  waterSwitchPos(4);
     }

// Record the day for the first sample.
     SD.begin();
     Logfile = SD.open("MARS-01.txt", FILE_WRITE);
     Logfile.print("Sample: ");
     Logfile.print(sampleCounter);
     Logfile.print ("\t");
     Logfile.print (waterSwitchCheck());
     Logfile.print("\t");
     format(Timestamp.day());
     Logfile.print("/");
     format (Timestamp.month());
     Logfile.print("/");
     format (Timestamp.year());
     Logfile.print(" ");     
     format (Timestamp.hour());
     Logfile.print(":");
     format (Timestamp.minute());
     Logfile.print(":");
     format (Timestamp.second());
     Logfile.print(F("\t"));
     Logfile.close(); //leaves line open for completion with the next record.

// And note the active day
activeDay = currentDay();
monthDelta();
Serial.println("Setup complete");
}//end setup

//*****************
//Custom functions
//*****************
//****Watchdog interrupt
//Setup interrupt so the watchdog can wake up the Arduino.
ISR(WDT_vect)
{
       wdt_reset();
}

// function to configure the watchdog: let it sleep 8 seconds before firing
// when firing, configure it for resuming program execution by default.
void configure_wdt(void)
{
  cli();                           // disable interrupts for changing the registers
  MCUSR = 0;                       // reset status register flags
                                   // Put timer in interrupt-only mode:                                       
  WDTCSR |= 0b00011000;            // Set WDCE (5th from left) and WDE (4th from left) to enter config mode,
                                   // using bitwise OR assignment (leaves other bits unchanged).
  WDTCSR =  0b01000000 | 0b100001; // set WDIE: interrupt enabled (8 seconds)
  //  WDTCSR =  0b01000000 | 0b000000; // set WDIE: interrupt enabled (1 second)
                                   // clr WDE: reset disabled
                                   // and set delay interval (right side of bar) to 8 seconds
  sei();                           // re-enable interrupts 
  // reminder of the definitions for the time before firing
  // delay interval patterns:
  //  16 ms:     0b000000
  //  500 ms:    0b000101
  //  1 second:  0b000110
  //  2 seconds: 0b000111
  //  4 seconds: 0b100000
  //  8 seconds: 0b100001
}

// Put the Arduino to deep sleep. Only an interrupt can wake it up.
void sleep(int sleepcycles) {  
  // Set sleep to full power down.  Only external interrupts or
  // the watchdog timer can wake the CPU!
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  // Turn off the ADC while asleep.
  power_adc_disable();
  while (sleepcycles > 0){ // while some cycles left, sleep!
  // Enable sleep and enter sleep mode.
  sleep_mode();
  // CPU is now asleep and program execution completely halts!
  // Once awake, execution will resume at this point if the
  // watchdog is configured for resume rather than restart 
  // When awake, disable sleep mode
  sleep_disable();
    if (BucketTipped == true){
      tipcounter++;
      BucketTipped = false;
    }
  sleepcycles--;
  }
 // put everything on again
  power_all_enable();
}

//****Tipping bucket counter
//counter that the tipping bucket can increment using interrupts.
void tipinterrupt(){
//static unsigned long last_interrupt_time = 0;
    //if ((long)(micros() - last_micros) >= debounceInterval){
    BucketTipped = true;
    //last_micros = micros();
    //}
}

void tippingSensorCheck(){
  if (tips != tipcounter){
    Serial.print(F("Tipcounter tips: "));
    Serial.println (tipcounter);
    //Serial.println(last_micros);
    tips = tipcounter;
  }
}

//****Platter rotational controls
void BottleOCW() {
  bottlestepper.runToNewPosition(-outersteps);
  bottlestepper.setCurrentPosition(0);
  bottlestepper.disableOutputs(); //and power down when finished
}

void BottleOACW() {
  bottlestepper.runToNewPosition(outersteps);
  bottlestepper.setCurrentPosition(0);
  bottlestepper.disableOutputs(); //and power down when finished
}

void BottleICW() {
  bottlestepper.runToNewPosition(-innersteps);
  bottlestepper.setCurrentPosition(0);
  bottlestepper.disableOutputs(); //and power down when finished
}

void BottleIACW() {
  bottlestepper.runToNewPosition(innersteps);
  bottlestepper.setCurrentPosition(0);
  bottlestepper.disableOutputs(); //and power down when finished
}

void changePlatterRing (){
  bottlestepper.runToNewPosition(changeoversteps);
  bottlestepper.setCurrentPosition(0);
  bottlestepper.disableOutputs(); //and power down when finished
}

//****Sample logging code.
//This is a bit confusing. Because it's quite hard to go back and rewrite data to the SD Card, instead we write the data in two lines. 
//The first line closes off the previous record, (number of tips, finish time, etc), the second line starts a new record.
void Samplelog (int x) {
     SD.begin();
     Logfile = SD.open("MARS-01.txt", FILE_WRITE);
     DateTime Timestamp = rtc.now();
     format(Timestamp.day());
     Logfile.print("/");
     format (Timestamp.month());
     Logfile.print("/");
     format (Timestamp.year());
     Logfile.print(" ");     
     format (Timestamp.hour());
     Logfile.print(":");
     format (Timestamp.minute());
     Logfile.print(":");
     format (Timestamp.second());
     Logfile.print("\t");
     Logfile.print(tipcounter); 
     Logfile.print("\t");
     Logfile.print("na"); //spot for temperature
     Logfile.print("\t");
     Logfile.println("na"); //spot for humidity & closes of the sample record.
     //and start new record line below.
     Logfile.print("Sample: ");
     Logfile.print(x);
     Logfile.print ("\t");
     Logfile.print (waterSwitchCheck());
     Logfile.print("\t");
     format(Timestamp.day());
     Logfile.print("/");
     format (Timestamp.month());
     Logfile.print("/");
     format (Timestamp.year());
     Logfile.print(" ");     
     format (Timestamp.hour());
     Logfile.print(":");
     format (Timestamp.minute());
     Logfile.print(":");
     format (Timestamp.second());
     Logfile.print(F("\t"));
     Logfile.close(); //leaves line open for completion with the next record.
}

//****Formatting code to make things nice to read (just pads a zero to days and months with only 1 digit). 
void format(int x){
    if (x < 10){
    Logfile.print("0");
    Logfile.print(x);
    }else{
    Logfile.print(x);
    }
}

//****Used to update the current day so that sampler can take a rest. :D
int currentDay(){
DateTime timestamp = rtc.now();
int currentday = timestamp.day();
return currentday;
}

//Include fuzzy start time for the month.
void setStartMonth(){
DateTime timestamp = rtc.now(); 
if (currentDay() >= monthFuzzyStart){
  startMonth = timestamp.month();
  startMonth++;
} else {
  startMonth = timestamp.month();
}
}

int monthDelta (){ //*** used to get current month
 DateTime timestamp = rtc.now();
 int currentmonth = timestamp.month();
 int monthdelta = currentmonth - startMonth;
 return monthdelta;
}

//****Time checker
boolean checktime (){
  DateTime timestamp = rtc.now();
  if (timestamp.hour() >= sampletime && timestamp.day() != activeDay) { //Set time to 9 once active - non debug
  return true;
  } else {
  //Serial.print("Checktime failed ");
  //Serial.print (activeDay);
  //Serial.print(" vs " );
  //Serial.println(Timestamp.day(), DEC);
  return false;
  }
}

//****Resets the water switch. Uses the microswitch to determine zero point, and a resettoggle bool to determine if it should run or not.
void resetwaterswitch(){
  Serial.print(F("Checking waterswitch limit switch. Result = "));
  Serial.println(digitalRead(resetswitchpin));
if (resettoggle == HIGH){
while (digitalRead(resetswitchpin) == HIGH){
      switchstepper.runSpeed();
        if (digitalRead(resetswitchpin) == LOW){
          resettoggle = LOW;
        break;
       }
      }
      switchstepper.setCurrentPosition(0); //reset position counter.
      switchstepper.disableOutputs(); //and power down when finished. 
    }
    Serial.println(F("MARS-01 switchstepper reset."));
}

//****Function to determine the correct position of the waterswitch, based on monthdelta and sample ID.
int waterSwitchCheck(){
int position;
if (sampleCounter <= 33 && monthDelta() <= 0 ){
position = 1;
}
if (sampleCounter > 33 && monthDelta() <= 0 ){
position = 1;
//add a message in the log here that everything is broken. :D
}
if (sampleCounter <= 33 && monthDelta() == 1 ){
position = 2;
}
if (sampleCounter > 33 && monthDelta() == 1 ){
position = 3;
}
if (sampleCounter <= 33 && monthDelta() >= 2 ){ //must update sample ID as well. //Note: This is the likely cause of the bug. If waterSwitchCheck doesn't qualify for any of these, it probably returns 0
position = 4; // further update. It seems to return 1, but I'm not sure if that is a proper int 1, or just a 1 that serial.print generates from something like an NA.
}
if (sampleCounter > 33 && monthDelta() >= 2 ){
position = 4;
}
  Serial.print(F("WaterSwitchPos = "));
  Serial.println(position);
  return (position);
}

//****Function to move through each position of the water switch. Just call waterswitchpos(position number).
void waterSwitchPos(int x){
  switch (x){
    case 1:
       switchstepper.runToNewPosition(position1);
       Serial.println(F("Waterswitch moved to position 1 (month 1, ring 1)"));
    break;
    case 2:
       switchstepper.runToNewPosition(position2);
       Serial.println(F("Waterswitch moved to position 2 (month 2, ring 1)"));
    break;
    case 3:
       switchstepper.runToNewPosition(position3);
       Serial.println(F("Waterswitch moved to position 3 (month 2, ring 2)"));
    break;
    case 4:
       switchstepper.runToNewPosition(position4);
       Serial.println(F("Waterswitch moved to position 4 (month 3, ring 2)"));
    break;  
  }
     switchstepper.disableOutputs(); //and power down when finished
}

//*********************
//Main Loop
//*********************
void loop() {
  delay (50);
  tippingSensorCheck(); //used to check the tipping bucket sensor. Comment out after debug. Posts tipcount to serial. 
  delay (50);
  // put your main code here, to run repeatedly:
  // add in here a way to change date via console.
     /* removed 15/9/19
     int minutes = Serial.parseInt();
     if (minutes > 0 && minutes < 60){
          tipcounter = tipcounter + 50;
           Serial.print(F("minutes to add: "));
           Serial.println(minutes);
           DateTime newtime = rtc.now() + TimeSpan(0,0,minutes,0);
           rtc.adjust(newtime);
           Serial.print ("New time is: ");
     DateTime timestamp = rtc.now(); 
     Serial.print(timestamp.day(), DEC);
     Serial.print("/");
     Serial.print(timestamp.month(), DEC);
     Serial.print("/");
     Serial.print(timestamp.year(), DEC);
     Serial.print("\t");     
     Serial.print(timestamp.hour(), DEC);
     Serial.print(":");
     Serial.print(timestamp.minute(), DEC);
     Serial.print(":");
     Serial.print(timestamp.second(), DEC);
     Serial.println(F("\t(DD/MM/YY HH:MM:SS)"));
           }
      if (minutes >= 60) {
       int hours = round(minutes/60);
        minutes = minutes - (hours * 60);
        tipcounter = tipcounter + 0;
     Serial.print(F("time to add: "));
     Serial.print(hours);
     Serial.print(":");
     Serial.print(minutes);
     Serial.println(":00");
     DateTime newtime = rtc.now() + TimeSpan(0,hours,minutes,0);
     rtc.adjust(newtime);
     Serial.print (F("New time is: "));
     DateTime timestamp = rtc.now(); 
     Serial.print(timestamp.day(), DEC);
     Serial.print("/");
     Serial.print(timestamp.month(), DEC);
     Serial.print("/");
     Serial.print(timestamp.year(), DEC);
     Serial.print("\t");     
     Serial.print(timestamp.hour(), DEC);
     Serial.print(":");
     Serial.print(timestamp.minute(), DEC);
     Serial.print(":");
     Serial.print(timestamp.second(), DEC);
     Serial.println(F("\t(DD/MM/YY HH:MM:SS)"));
     }
    */
  //Program structure 
  //if startday > 25 (allow some slop in the beggining of the month - done in setup using monthfuzziness variable)

  if (checktime() && currentDay() == 1){ //lets sort out the water switch first.
     waterSwitchPos(waterSwitchCheck()); //change water switch to correct position
     Serial.println(F("Water switch checked"));
     //record log that month has changed.
  } //end if 
  if (sampleCounter <= 33 && monthDelta()== 2 ){
    int missedSamples = 33 - sampleCounter; //figure out how many samples we need to skip to get to 34.
    for (int x=0;x<missedSamples;x++){ //double check the maths here
    BottleOCW();
    }
    sampleCounter = 34; //update sample ID
    changePlatterRing (); // change to inner ring.
    Samplelog(sampleCounter); //closeout previous days log.
    tipcounter = 0;
    Serial.println(F("Water switch checked and rings changed to inner"));//sample has changed, so we can reset the tipcounter.
    //record log that month and sample ID has changed.
    }
  
  if (checktime() && tipcounter <= mintipcount){ //New day, but no tips counted. //changed 14/9/19 from == 0 to <= mintipcount
      Samplelog(sampleCounter);//record the sample details but do nothing else. No need for new sample.
      activeDay = currentDay();//Including this prevents the double sample that occurs after multiple days of no rainfall.
      Serial.println(F("Nothing happened today"));
  } else {
  if (checktime() || tipcounter >= maxtipcount){ //New day, or the number of tips is enough to warrant a new bottle.
  sampleCounter++; //add 1 to the counter.
  Serial.print (F("Tips counted: "));
  Serial.println(tipcounter); 
  activeDay = currentDay();
  Serial.print(F("Setting current day to "));
  Serial.println (currentDay());
    if (sampleCounter <= 33){
    Serial.print(F("Rotate outer bottles for sample: "));
    Serial.println(sampleCounter);
    BottleOCW();
  } else if (sampleCounter == 34){
    waterSwitchPos(waterSwitchCheck());//check the water switch.
    changePlatterRing ();
    Serial.print(F("Activate the waterswitch to change the ring over and rotate to align inner sample ring at sample: "));
    Serial.println(sampleCounter);
  } else if (sampleCounter > 34 && sampleCounter <= 60){
    Serial.print(F("Rotate inner bottles for sample: "));
    Serial.println(sampleCounter);
    BottleICW();
  }
  Samplelog(sampleCounter); //record the sample details
  tipcounter = 0; //reset tipcounter
  }
  }
  //sleep(75); // sleep for 10 minutes.
  //annoying quirk, but sleep needs to be enabled for the tipping sensor to work. Trying to count in the sensor ISR fails if using sleep.
  sleep(4); //sleep for 8 seconds
  //Serial.println("Waking up.");
}
