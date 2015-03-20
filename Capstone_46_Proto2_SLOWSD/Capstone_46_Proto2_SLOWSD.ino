/*
Capstone 46 Fall Datalogger
 
 Nick Yanchuk
 Ali
 Bahar
 Jing
 
 Note:
 MATLAB CODE to make Altimeter data low pass:
 plot(filtfilt(fir1(500,0.001),1,AltimeterDataVector));
 */

/*=============================================INCLUDES========================================*/
// SD card library
#include <SdFat.h>
// Used for I2C bus
#include <Wire.h>
// Used for SPI communication with RTC and Accelerometer
#include <SPI.h>
// Altimeter library
#include "MPL3115A2.h"
// File with a struct definition for the data array
// TO BE CHANGED/REMOVED 
// Only there as a leftover from the previous SD card write method
#include "UserDataType.h"
// Used for sleep mode / power down
#include <avr/sleep.h>
#include <avr/power.h>

/*===========================================DEFINITIONS========================================*/
// Log file base name for logging data.  Must be six characters or less.
#define FILE_BASE_NAME "DATA"
// SD card chip select pin
#define SD_CS_PIN 9
// Accelerometer chip select pin
#define chipSelectADXL 10
// RTC chip select pin
#define chipSelectRTC 8
// Green led pin
#define ledPin 7
// Button input pin (NOTE REDUNDANT AS BUTTON INTERUPT PIN USED AS WELL)
#define buttonPin 6
// Pin that recieves the ADXL interrupt
#define interruptPin 3
// Pin that recieves the button press interrupt
#define buttoninterruptPin 2
// Digital pin to indicate an error, set to -1 if not used.
// The led blinks for fatal errors. The led goes on solid for SD write
// overrun errors and logging continues.
#define ERROR_LED_PIN 5
// Defines how often the RTC reads and updates - SHOULD PROBABLY GO DOWN
#define RTC_UPDATE_INTERVAL 20
//This is a list of some of the registers available on the ADXL345.
//To learn more about these and the rest of the registers on the ADXL345, read the datasheet!
#define POWER_CTL 0x2D	//Power Control Register
#define DATA_FORMAT 0x31
#define DATAX0 0x32	//X-Axis Data 0
#define DATAX1 0x33	//X-Axis Data 1
#define DATAY0 0x34	//Y-Axis Data 0
#define DATAY1 0x35	//Y-Axis Data 1
#define DATAZ0 0x36	//Z-Axis Data 0
#define DATAZ1 0x37	//Z-Axis Data 1
#define INT_MAP 0x2F    //map to tell to send to INT1 or INT2
#define INT_ENABLE 0x2E //D7-D0: DATA_READY, SINGLE_TAP, DOUBLE_TAP, Activity, Inactivity, FREE_FALL, Watermark, Overrun
#define INT_SOURCE 0x30 //read to clear interrupt
#define THRESH_ACT 0x24 //unsigned 8bit magnitude of threshold
#define ACT_INACT_CTL 0x027
#define THRESH_FF 0x28 //unsigned rootsumsquare of all axes!!!try 0x09 to start
#define TIME_FF 0x29 //unsigned time, try 0x14 (100ms) or 350ms (0x46)

/*==============================================ALLOCATIONS========================================*/
// Presure Sensor definition
MPL3115A2 myPressure;
// File system object.
SdFat sd;
// Log file object.
SdFile file;
// Object for reading the settings into 
SdFile settingsFile;

// This buffer will hold values read from the ADXL345 registers.
byte values[10];

// Threshold and Record length read from the text settings file
char threshSettingText[2];
int threshSettingInt = 10;
char recordLengthText[4];
int recordLengthInt = 100;

// Time and Date Array, holds the current time
int TimeDateGlobal[7]; 
//Counter to update RTC value every once in a while (less than a second, of course)
int timeUpdateCounter = 0;

// Holds the x,y and z axis accelerometer values.
int x,y,z,s,a,t = 0;

// saves the SPI register before sleeping
byte spi_save;

// Timer used to count how long to stay awake for before going to sleep
int awakeTimer = 0;

// Just a state variable for Recording or Converting to CSV. Will go away eventually maybe
// Used mainly just by the LED
int currentState = 1;

/*==========================================FUNCTION DEFINITIONS==========================================*/
/*--------------------------------------------ADXL FUNCTIONS----------------------------------------------*/

//This function will write a value to a register on the ADXL345.
//Parameters:
//  char registerAddress - The register to write a value to
//  char value - The value to be written to the specified register.
void writeRegister(char registerAddress, char value){
  //Set Chip Select pin low to signal the beginning of an SPI packet.
  digitalWrite(chipSelectADXL, LOW);
  //Transfer the register address over SPI.
  SPI.transfer(registerAddress);
  //Transfer the desired register value over SPI.
  SPI.transfer(value);
  //Set the Chip Select pin high to signal the end of an SPI packet.
  digitalWrite(chipSelectADXL, HIGH);
}

//This function will read a certain number of registers starting from a specified address and store their values in a buffer.
//Parameters:
//  char registerAddress - The register addresse to start the read sequence from.
//  int numBytes - The number of registers that should be read.
//  char * values - A pointer to a buffer where the results of the operation should be stored.
void readRegister(char registerAddress, int numBytes, byte * values){
  //Since we're performing a read operation, the most significant bit of the register address should be set.
  char address = 0x80 | registerAddress;
  //If we're doing a multi-byte read, bit 6 needs to be set as well.
  if(numBytes > 1)address = address | 0x40;

  //Set the Chip select pin low to start an SPI packet.
  digitalWrite(chipSelectADXL, LOW);
  //Transfer the starting register address that needs to be read.
  SPI.transfer(address);
  //Continue to read registers until we've read the number specified, storing the results to the input buffer.
  for(int i=0; i<numBytes; i++){
    values[i] = SPI.transfer(0x00);
  }
  //Set the Chips Select pin high to end the SPI packet.
  digitalWrite(chipSelectADXL, HIGH);
}

/*--------------------------------------------SD CARD FUNCTIONS----------------------------------------------*/
// Error messages stored in flash.
// Taken from SD card sample code
#define error(msg) error_P(PSTR(msg))
void error_P(const char* msg) {
  sd.errorHalt_P(msg);
}

// Write data header.
void writeHeader() {
  file.print(F("Year, "));
  file.print(F("Month, "));
  file.print(F("Day, "));
  file.print(F("Hour, "));
  file.print(F("Min, "));
  file.print(F("Sec, "));
  file.print(F("Altitude, "));
  file.print(F("Temperature, "));
  file.print(F("x-axis, "));
  file.print(F("y-axis, "));
  file.print(F("z-axis "));
  file.println();
}

// Main loop during data recording
// Probably should split this up better
void logData() {
  // flag to leave the recording loop
  byte closeFile = false;
  // instance of data to record, should maybe be global?
  data_t data;

  // recording loop, while close file flag not set
  while(!closeFile){

    // read processor time
    // currently unused, but could be helpful in the future 
    data.time = micros();
    
    // read the altimeter
    a = myPressure.readAltitudeFt();

    // set the spi properly just to be safe,
    // clear the interrupt, just to be safe
    // read the ADXL data
    SPI.setDataMode(SPI_MODE3);
    readRegister(INT_SOURCE, 1, values);
    readRegister(DATAX0, 6, values);
    // The ADXL345 gives 10-bit acceleration values, but they are stored as bytes (8-bits). To get the full value, two bytes must be combined for each axis.
    // The X value is stored in values[0] and values[1].
    x = (int)((word)values[1]<<8)|(word)values[0];
    // The Y value is stored in values[2] and values[3].
    y = (int)((word)values[3]<<8)|(word)values[2];
    // The Z value is stored in values[4] and values[5].
    z = (int)((word)values[5]<<8)|(word)values[4];
    // save the sum. not currently used, but maybe we should just do it here anyways?
    s = abs(x)+abs(y)+abs(z);

    // if it is time to update the time and date from the RTC, do that
    if(timeUpdateCounter == 0) UpdateTimeDate(TimeDateGlobal);
    // incriment the counter until the next RTC update
    timeUpdateCounter++;
    // reset the counter to 0 (meaning its time to update) if it reaches the setting
    if(timeUpdateCounter >= RTC_UPDATE_INTERVAL) timeUpdateCounter = 0;   

    // move data to my data array
    // an uncessesary step, we should change this
    data.adc[0] = TimeDateGlobal[6];
    data.adc[1] = TimeDateGlobal[5];
    data.adc[2] = TimeDateGlobal[4]; 
    //3 is null
    data.adc[3] = TimeDateGlobal[2];
    data.adc[4] = TimeDateGlobal[1];
    data.adc[5] = TimeDateGlobal[0];
    data.adc[6] = a;
    data.adc[7] = t;
    data.adc[8] = x;
    data.adc[9] = y;
    data.adc[10] = z;
     
    // print the data to the SD card
    for (uint8_t i = 0; i < ADC_DIM; i++) {
      file.print(data.adc[i]);
      file.write(',');
    }
    file.println();

    // check to see if it is time to go to sleep yet
    if(awakeTimer >= recordLengthInt) {
      // write to serial
      Serial.println("gone to sleep ");
      // "save" data
      file.sync();
      // go to sleep
      enterSleep();
      // write to serial
      Serial.println("back awake ");
      // reset sleep counter
      awakeTimer = 0;
      // set to 0 to make sure to update RTC first loop awake
      timeUpdateCounter = 0;
    }
    // incriment sleep timer
    awakeTimer++;

    // looks to see if the button is pressed to STOP recording
    // works in conjunction with the buttonpress interrupt basically
    // it will probably reach here before you can take your finger off the button
    //if(digitalRead(buttonPin) == LOW) closeFile = true; 
    if(digitalRead(buttoninterruptPin) == LOW) closeFile = true; 
    
  }
}

/*----------------------------------------------SLEEP MODE----------------------------*/
// Disable lots of stuff and put the processor to sleep
void enterSleep(void)
{
  // Clear the ADXL interrupt (overkill..apparently both clear the interrupt)
  readRegister(INT_SOURCE, 1, values);
  readRegister(DATAX0, 6, values);

  /* Setup INT1 (ADXL) and INT0 (button) pins as an interrupt and attach handler. */
  attachInterrupt(1, interruptRoutine, LOW);
  attachInterrupt(0, interruptRoutine, LOW);
  // delay to let interrupts attach...not sure if needed
  delay(100);  

  // set the sleep mode and enable    
  // not 100% this is the best place for sleep enable
  // maybe it should be closer to the sleep_mode(), 
  // in case the interrupt fires before it gets past this stuff  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();

  // save the SPI register and turn off the SPI bus power
  spi_save = SPCR;
  SPCR = 0;
  SPI.end();

  // turn off everything in the god damm universe   
  power_adc_disable();
  power_spi_disable();
  power_timer0_disable();
  power_timer1_disable();
  power_timer2_disable();
  power_twi_disable();         

  // go to sleep!!        
  sleep_mode();

  /* The program will call interruptRoutine and continue from here. */

  // turn everything back on     
  power_all_enable();

  // turn on the SPI bus again
  SPI.begin();
  SPCR = spi_save;    
}

// function called when interrupt happens
void interruptRoutine(void)
{
  /* This will bring us back from sleep. */

  /* We detach the interrupt to stop it from 
   * continuously firing while the interrupt pin
   * is low.
   */
  detachInterrupt(1);
  detachInterrupt(0);

  // turn off the sleep mode
  sleep_disable();
}

/*----------------------------------------------------RTC-------------------------------------------------*/

// initialize the RTC
// from the sample code
int RTC_init(){ 
  pinMode(chipSelectRTC,OUTPUT); // chip select
  // start the SPI library:
  SPI.begin();
  SPI.setBitOrder(MSBFIRST); 
  SPI.setDataMode(SPI_MODE3); // both mode 1 & 3 should work MODE 3
  //set control register 
  digitalWrite(chipSelectRTC, LOW);  
  SPI.transfer(0x8E);
  SPI.transfer(0x60); //60= disable Osciallator and Battery SQ wave @1hz, temp compensation, Alarms disabled
  digitalWrite(chipSelectRTC, HIGH);
  delay(10);
}

// reads the time and data to the timedate array
// adapted from the sample code
void UpdateTimeDate(int* TimeDateLocal){

  SPI.setDataMode(SPI_MODE3);
  //int TimeDate [7]; //second,minute,hour,null,day,month,year		
  for(int i=0; i<=6;i++){
    if(i==3)
      i++;
    digitalWrite(chipSelectRTC, LOW);
    SPI.transfer(i+0x00); 
    unsigned int n = SPI.transfer(0x00);        
    digitalWrite(chipSelectRTC, HIGH);
    int a=n & B00001111;    
    if(i==2){	
      int b=(n & B00110000)>>4; //24 hour mode
      if(b==B00000010)
        b=20;        
      else if(b==B00000001)
        b=10;
      TimeDateLocal[i]=a+b;
    }
    else if(i==4){
      int b=(n & B00110000)>>4;
      TimeDateLocal[i]=a+b*10;
    }
    else if(i==5){
      int b=(n & B00010000)>>4;
      TimeDateLocal[i]=a+b*10;
    }
    else if(i==6){
      int b=(n & B11110000)>>4;
      TimeDateLocal[i]=a+b*10;
    }
    else{	
      int b=(n & B01110000)>>4;
      TimeDateLocal[i]=a+b*10;	
    }
  }
}

/*---------------------------------------------------SIMPLE IO FUNCTIONS--------------------------*/

// wait for button press
void pauseForButton(){
  while(1){
  // if(digitalRead(buttonPin) ==  HIGH) break;
    if(digitalRead(buttoninterruptPin) ==  HIGH) break;
  }
  while(1){
  // if(digitalRead(buttonPin) == LOW) break;
    if(digitalRead(buttoninterruptPin) == LOW) break;
    if(currentState == 1 ) blinkMyLedGreen();
  }
  while(1){
    // if(digitalRead(buttonPin) ==  HIGH) break;
    if(digitalRead(buttoninterruptPin) ==  HIGH) break;
  }
}

// blink the LED
// need to fix this so it doesn't rely on delay, but uses a counter instead
void blinkMyLedGreen(){
  digitalWrite(ledPin,LOW);
  if(currentState == 1) delay(100);
  digitalWrite(ledPin,HIGH);
  if(currentState == 1) delay(100);
}

/*==========================================SETUP===================================================*/
void setup(void) {
  // set all pins to proper i/o mode
  //pinMode(buttonPin, INPUT_PULLUP);
  pinMode(ledPin,OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP);
  pinMode(buttoninterruptPin, INPUT_PULLUP);

  // I2C initialization
  pinMode(13, OUTPUT);
  Wire.begin();        // Join i2c bus
  myPressure.begin(); // Get sensor online
  //Configure the sensor
  myPressure.setModeAltimeter(); // Measure altitude above sea level in meters
  //myPressure.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa
  myPressure.setOversampleRate(7); // Set Oversample to the recommended 128
  myPressure.enableEventFlags(); // Enable all three pressure and temp event flags 

  // Initiate an SPI communication instance.
  SPI.begin();
  // Configure the SPI connection for the ADXL345.
  SPI.setDataMode(SPI_MODE3);
  // Set up the Chip Select pin to be an output from the Arduino.
  pinMode(chipSelectADXL, OUTPUT);
  // Before communication starts, the Chip Select pin needs to be set high.
  digitalWrite(chipSelectADXL, HIGH);
  // Put the ADXL345 into +/- 8G range by writing the value 0x01 to the DATA_FORMAT register.?
  writeRegister(DATA_FORMAT, 0x02);
  // Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
  writeRegister(POWER_CTL, 0x08);  //Measurement mode  
  // Set interrupts to all go to INT1
  writeRegister(INT_MAP,0x00);
  // D7-D0: DATA_READY, SINGLE_TAP, DOUBLE_TAP, Activity, Inactivity, FREE_FALL, Watermark, Overrun
  writeRegister(INT_ENABLE, 0x04); //enable freefall
  // unsigned rootsumsquare of all axes!!!try 0x09 to start
  // a default threshold
  writeRegister(THRESH_FF, 9); 
  // unsigned time, try 0x14 (100ms) or 350ms (0x46)
  writeRegister(TIME_FF, 0x14);
  // clear interrupts
  readRegister(INT_SOURCE, 1, values);

  // offsets to fix werid issues with the ADXL values
  // LOOK INTO THIS MORE
  writeRegister(0x1E, 0x01);
  writeRegister(0x1F, 0xFF);
  writeRegister(0x20, 0x03);

  // initialize the RTC
  RTC_init();

  // allow serial communication to the arduino IDE
  Serial.begin(9600);
}

/*========================================MAIN LOOP==================================*/
void loop(void) {
  
  // initialize the SD card  
  if (!sd.begin(SD_CS_PIN, SPI_FULL_SPEED)) sd.initErrorHalt();

  // read the settings file
  if(!settingsFile.open("FREEFALL.TXT", O_READ)) error("settings.open");
  // read 2 chars
  settingsFile.read(threshSettingText,2);
  settingsFile.close();  
  // conver the 2 chars to an int
  threshSettingInt = atoi(threshSettingText);
  // disallow invalid values (maybe make more robust)?
  if(threshSettingInt > 60) threshSettingInt = 60;
  if(threshSettingInt < 1) threshSettingInt = 1;
  // print to serial
  Serial.println("Threshold Setting (range 1-60):");
  Serial.println(threshSettingInt);
  // set the ADXL to the value
  writeRegister(THRESH_FF, threshSettingInt); 
  
  // read the settings file
  if(!settingsFile.open("EVLENGTH.TXT", O_READ)) error("settings.open");
  // read 4 chars
  settingsFile.read(recordLengthText,4);
  settingsFile.close();  
  // convert from 4 chars to int
  recordLengthInt = atoi(recordLengthText);
  // disallow invalid values (maybe change in future?)
  if(recordLengthInt > 9999) recordLengthInt = 9999;
  if(recordLengthInt < 1) recordLengthInt = 1;
  // print to serial
  Serial.println("\nEvent Record Length (samples/event 1-9999, current speed is ~140Hz):");
  Serial.println(recordLengthInt);    
  
  // create new file name for data
  const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
  char fileName[13] = FILE_BASE_NAME "00.CSV";

  // Find an unused file name.
  if (BASE_NAME_SIZE > 6) {
    error("FILE_BASE_NAME too long");
  }
  while (sd.exists(fileName)) {
    if (fileName[BASE_NAME_SIZE + 1] != '9') {
      fileName[BASE_NAME_SIZE + 1]++;
    } 
    else if (fileName[BASE_NAME_SIZE] != '9') {
      fileName[BASE_NAME_SIZE + 1] = '0';
      fileName[BASE_NAME_SIZE]++;
    } 
    else {
      error("Can't create file name");
    }
  }
  
  // create new data file
  if (!file.open(fileName, O_CREAT | O_WRITE | O_EXCL)) error("file.open");
  // print data file name to serial
  Serial.print(F("\nFile created: "));
  Serial.println(fileName);
  Serial.println("Press device control button to begin logging data\n");  
  // Write data header.
  writeHeader();
  
  // update RTC just to make sure for first iteration
  // hopefully we can get rid of this, but it was being buggy and this was a quickfix
  UpdateTimeDate(TimeDateGlobal);

  // Turn off error led
  digitalWrite(ERROR_LED_PIN, HIGH);
  //turn ON Green led
  digitalWrite(ledPin,LOW);
  // wait for input
  currentState = 0;
  pauseForButton();
  // turn OFF green led
  digitalWrite(ledPin,HIGH);
  // print to serial
  Serial.println("Logging Data...\n");
  
  // call data logging loop! yay!
  logData();
  
  // close and save file when done
  file.close();
  
  //print to serial
  Serial.print('\n');
  Serial.print(fileName);
  Serial.println(" Completed\n");
  Serial.println("Press device control button to create new data log\n");
  
  // make the green led blink to indicate it's done recording
  currentState = 1;
  // wait for input to re read settings, create new file, and record again
  pauseForButton();

}

