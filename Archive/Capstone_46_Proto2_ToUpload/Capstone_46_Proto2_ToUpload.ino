/**
 * Samples are logged at regular intervals.  The maximum logging rate
 * depends on the quality of your SD card and the time required to
 * read sensor data.  This example has been tested at 500 Hz with
 * good SD card on an Uno.  4000 HZ is possible on a Due.
 *
 * If your SD card has a long write latency, it may be necessary to use
 * slower sample rates.  Using a Mega Arduino helps overcome latency
 * problems since 13 512 byte buffers will be used.
 *
 * Data is written to the file using a SD multiple block write command.



MATLAB CODE to make Altimeter data low pass:
plot(filtfilt(fir1(500,0.001),1,AltimeterDataVector));
*/

/*===========================================DEFINITIONS===========================================*/
#include <SdFat.h>
#include <SdFatUtil.h>
#include <Wire.h>
#include <SPI.h>
#include "MPL3115A2.h"
#include "UserDataType.h"
//#include <MemoryFree.h>
#include <avr/sleep.h>
#include <avr/power.h>

#define SD_CS_PIN 9
#define chipSelectADXL 10
#define chipSelectRTC 8
#define ledPin 7
#define SDpowerPin 4
#define buttonPin 6
#define interruptPin 3
#define buttoninterruptPin 2
#define RTC_UPDATE_INTERVAL 50
#define SERIAL_DUMP 0
#define LOG_INTERVAL_USEC 10000

//Presure Sensor definition
MPL3115A2 myPressure;

// Digital pin to indicate an error, set to -1 if not used.
// The led blinks for fatal errors. The led goes on solid for SD write
// overrun errors and logging continues.
#define ERROR_LED_PIN 5

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

#define HIGHER_TH 0x10
#define HIGH_TH 0x09
#define MEDIUM_TH 0x08
#define LOW_TH 0x07


//This buffer will hold values read from the ADXL345 registers.
byte values[10];

//Loop counter for not to write to SD
byte reduceSDwriteCount;

//Time and Date Array
int TimeDateGlobal[7]; 

//These variables will be used to hold the x,y and z axis accelerometer values.
int x,y,z,s,a,t = 0;

//for sleep mode testing
int seconds = 0;
byte spi_save;
int awakeTimer = 0;


//Recording or Converting to CSV. Will go away eventually
int currentState = 1;
//Counter to update RTC value every once in a while (less than a second, of course)
int timeUpdateCounter = 0;

// Maximum file size in blocks.
// The program creates a contiguous file with FILE_BLOCK_COUNT 512 byte blocks.
// This file is flash erased using special SD commands.  The file will be
// truncated if logging is stopped early.
const uint32_t FILE_BLOCK_COUNT = 256000;

// log file base name.  Must be six characters or less.
#define FILE_BASE_NAME "DATA"

// Buffer definitions.
//
// The logger will use SdFat's buffer plus BUFFER_BLOCK_COUNT additional 
// buffers.
//
#ifndef RAMEND
// Assume ARM. Use total of nine 512 byte buffers.
const uint8_t BUFFER_BLOCK_COUNT = 8;
//
#elif RAMEND < 0X8FF
#error Too little SRAM
//
#elif RAMEND < 0X10FF
// Use total of two 512 byte buffers.
const uint8_t BUFFER_BLOCK_COUNT = 1;
//
#elif RAMEND < 0X20FF
// Use total of five 512 byte buffers.
const uint8_t BUFFER_BLOCK_COUNT = 4;
//
#else  // RAMEND
// Use total of 13 512 byte buffers.
const uint8_t BUFFER_BLOCK_COUNT = 12;
#endif  // RAMEND

// Temporary log file.  Will be deleted if a reset or power failure occurs.
#define TMP_FILE_NAME "TMP_LOG.BIN"

// Size of file base name.  Must not be larger than six.
const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;

SdFat sd;
SdBaseFile binFile;
Sd2Card card;
SdVolume vol;

char binName[13] = FILE_BASE_NAME "00.BIN";

// Number of data records in a block.
const uint16_t DATA_DIM = (512 - 4)/sizeof(data_t);

//Compute fill so block size is 512 bytes.  FILL_DIM may be zero.
const uint16_t FILL_DIM = 512 - 4 - DATA_DIM*sizeof(data_t);

struct block_t {
  uint16_t count;
  uint16_t overrun;
  data_t data[DATA_DIM];
  uint8_t fill[FILL_DIM];
};

const uint8_t QUEUE_DIM = BUFFER_BLOCK_COUNT + 2;

block_t* emptyQueue[QUEUE_DIM];
uint8_t emptyHead;
uint8_t emptyTail;

block_t* fullQueue[QUEUE_DIM];
uint8_t fullHead;
uint8_t fullTail;

// Advance queue index.
inline uint8_t queueNext(uint8_t ht) {
  return ht < (QUEUE_DIM - 1) ? ht + 1 : 0;
}

/*==========================================FUNCTION DEFINITIONS==========================================*/
/*======AQUIRE A DATA RECORD======*/
void acquireData(data_t* data) {
  
  data->time = micros();
  a = myPressure.readAltitudeFt();
  //t = myPressure.readTemp();
  //a = data->adc[0];
  
  SPI.setDataMode(SPI_MODE3);
  
  //if(digitalRead(interruptPin)){
     
  //  digitalWrite(ledPin,LOW);
  //  delay(500);
  //  digitalWrite(ledPin,HIGH);
  //  delay(500);
  //  digitalWrite(ledPin,LOW);
  //  delay(500);
  //   digitalWrite(ledPin,HIGH);
  //   delay(500);
  //}
    
  data->adc[11] = digitalRead(interruptPin);
  readRegister(INT_SOURCE, 1, values);
  //readRegister(DATAX0, 6, values);
  readRegister(DATAX0, 6, values);

  ////The ADXL345 gives 10-bit acceleration values, but they are stored as bytes (8-bits). To get the full value, two bytes must be combined for each axis.
  ////The X value is stored in values[0] and values[1].
  x = (int)((word)values[1]<<8)|(word)values[0];
  ////The Y value is stored in values[2] and values[3].
  y = (int)((word)values[3]<<8)|(word)values[2];
  ////The Z value is stored in values[4] and values[5].
  z = (int)((word)values[5]<<8)|(word)values[4];

  s = abs(x)+abs(y)+abs(z);

  if(timeUpdateCounter == 0) UpdateTimeDate(TimeDateGlobal);
  timeUpdateCounter++;
  if(timeUpdateCounter >= RTC_UPDATE_INTERVAL) timeUpdateCounter = 0;   
  
  data->adc[0] = TimeDateGlobal[6];
  data->adc[1] = TimeDateGlobal[5];
  data->adc[2] = TimeDateGlobal[4]; 
  //3 is null
  data->adc[3] = TimeDateGlobal[2];
  data->adc[4] = TimeDateGlobal[1];
  data->adc[5] = TimeDateGlobal[0];
  data->adc[6] = a;
  data->adc[7] = t;
  data->adc[8] = x;
  data->adc[9] = y;
  data->adc[10] = z;
  
if(awakeTimer >= 300) {
    Serial.println("gone to sleep ");
    enterSleep();
    Serial.println("back awake ");
    awakeTimer = 0;
    timeUpdateCounter = 0;
}
  awakeTimer++;
   
}

/*======AQUIRE A DATA RECORD======*/
void fakeacquireData(data_t* data) {
 
  delay(2000);
  
  //readRegister(INT_SOURCE, 1, &dummy);
  
}


/*======CHECK DATA ======*/
void checkData() {
  
  a = myPressure.readAltitudeFt();
  //t = myPressure.readTemp();
  //a = data->adc[0];
  SPI.setDataMode(SPI_MODE3);

  //readRegister(DATAX0, 6, values);
  readRegister(DATAX0, 6, values);

  ////The ADXL345 gives 10-bit acceleration values, but they are stored as bytes (8-bits). To get the full value, two bytes must be combined for each axis.
  ////The X value is stored in values[0] and values[1].
  x = (int)((word)values[1]<<8)|(word)values[0];
  ////The Y value is stored in values[2] and values[3].
  y = (int)((word)values[3]<<8)|(word)values[2];
  ////The Z value is stored in values[4] and values[5].
  z = (int)((word)values[5]<<8)|(word)values[4];

  s = abs(x)+abs(y)+abs(z);

  if(timeUpdateCounter == 0) UpdateTimeDate(TimeDateGlobal);
  timeUpdateCounter++;
  if(timeUpdateCounter >= RTC_UPDATE_INTERVAL) timeUpdateCounter = 0;   
  
}


/*========PRINT A DATA RECORD=========*/
void printData(Print* pr, data_t* data) {
  pr->print(data->adc[0]);
  for (int i = 1; i < ADC_DIM; i++) {
    pr->write(',');  
    pr->print(data->adc[i]);
  }
  pr->println();
}

/*=========PRINT THE HEADER================*/
void printHeader(Print* pr) {
  pr->print(F("Year, "));
  pr->print(F("Month, "));
  pr->print(F("Day, "));
  pr->print(F("Hour, "));
  pr->print(F("Min, "));
  pr->print(F("Sec, "));
  pr->print(F("Altitude, "));
  pr->print(F("Temperature, "));
  pr->print(F("x-axis, "));
  pr->print(F("y-axis, "));
  pr->print(F("z-axis, "));
  pr->print(F("INT1, "));
  pr->println();
}

/*===================SLEEP MODE SSTUFF===============*/
void enterSleep(void)
{
  
  readRegister(INT_SOURCE, 1, values);
  readRegister(DATAX0, 6, values);
        
  //if (card.init(SPI_FULL_SPEED,SD_CS_PIN)) {
  // Serial.println("cardinit ");
  //}

  //if (vol.init(&card)) {
  //  Serial.println("volinit ");
  //}      
        
        
  /* Setup pin2 as an interrupt and attach handler. */
  attachInterrupt(1, interruptRoutine, LOW);
  attachInterrupt(0, interruptRoutine, LOW);
  delay(100);  
        
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
      
  sleep_enable();
        
     
          
 spi_save = SPCR;
 SPCR = 0;
 
 SPI.end();
 //pinMode(13,INPUT_PULLUP);
 //pinMode(12,INPUT_PULLUP);
 //pinMode(11,INPUT_PULLUP);
 
 // digitalWrite(13,LOW);
  
  
  digitalWrite(SDpowerPin,LOW);     
  power_adc_disable();
  power_spi_disable();
  power_timer0_disable();
  power_timer1_disable();
  power_timer2_disable();
  power_twi_disable();         
          
  sleep_mode();
  
  /* The program will continue from here. */
  
  /* First thing to do is disable sleep. */
  //sleep_disable(); 
  
    
  power_all_enable();
    
  SPI.begin();
  
  SPCR = spi_save;  
  
  digitalWrite(SDpowerPin,HIGH);
  //if (!sd.begin(SD_CS_PIN, SPI_FULL_SPEED)) {
  //  sd.initErrorPrint();
  //  fatalBlink();
  //}
  
  
}


void interruptRoutine(void)
{
   /* This will bring us back from sleep. */
  
  /* We detach the interrupt to stop it from 
   * continuously firing while the interrupt pin
   * is low.
   */
  detachInterrupt(1);
  detachInterrupt(0);
  sleep_disable();
   
}



//==================ADXL FUNCTIONS==================*/
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


/*=======================SD CARD FUNCTIONS====================*/
// Error messages stored in flash.
#define error(msg) error_P(PSTR(msg))
//------------------------------------------------------------------------------
void error_P(const char* msg) {
  //sd.errorPrint_P(msg);
  fatalBlink();
}
//------------------------------------------------------------------------------
//
void fatalBlink() {
  while (true) {
    if (ERROR_LED_PIN >= 0) {
      digitalWrite(ERROR_LED_PIN, LOW);
      delay(200);
      digitalWrite(ERROR_LED_PIN, HIGH);
      delay(200);
    }
  }
}
/*=============CONVERT BINARY TO CSV============*/
// Convert binary file to CSV file.
void binaryToCsv() {
  uint8_t lastPct = 0;
  block_t block;
  uint32_t t0 = millis();
  uint32_t syncCluster = 0;
  SdFile csvFile;
  char csvName[13];

  if (!binFile.isOpen()) {
    Serial.println();
    Serial.println(F("No current binary file"));
    return;
  }
  binFile.rewind();
  // Create a new csvFile.
  strcpy(csvName, binName);
  strcpy_P(&csvName[BASE_NAME_SIZE + 3], PSTR("CSV"));

  if (!csvFile.open(csvName, O_WRITE | O_CREAT | O_TRUNC)) {
    error("open csvFile failed");  
  }
  Serial.println();
  Serial.print(F("Writing: "));
  Serial.print(csvName);
  Serial.println(F(" - type any character to stop"));
  printHeader(&csvFile);
  uint32_t tPct = millis();
  while (!Serial.available() && binFile.read(&block, 512) == 512) {
    uint16_t i;
    if (block.count == 0) break;
    if (block.overrun) {
      csvFile.print(F("OVERRUN,"));
      csvFile.println(block.overrun);
    }
    for (i = 0; i < block.count; i++) {
      printData(&csvFile, &block.data[i]);
    }
    if (csvFile.curCluster() != syncCluster) {
      csvFile.sync();
      syncCluster = csvFile.curCluster();
    }
    if ((millis() - tPct) > 1000) {
      uint8_t pct = binFile.curPosition()/(binFile.fileSize()/100);
      if (pct != lastPct) {
        tPct = millis();
        lastPct = pct;
        Serial.print(pct, DEC);
        Serial.println('%');
      }
    }
    if (Serial.available()) break;
  }
  csvFile.close();
  Serial.print(F("Done: "));
  Serial.print(0.001*(millis() - t0));
  Serial.println(F(" Seconds"));
}
/*========CHECK FOR OVERRUN========*/
// read data file and check for overruns
void checkOverrun() {
  bool headerPrinted = false;
  block_t block;
  uint32_t bgnBlock, endBlock;
  uint32_t bn = 0;

  if (!binFile.isOpen()) {
    Serial.println();
    Serial.println(F("No current binary file"));
    return;
  }
  if (!binFile.contiguousRange(&bgnBlock, &endBlock)) {
    error("contiguousRange failed");
  }
  binFile.rewind();
  Serial.println();
  Serial.println(F("Checking overrun errors - type any character to stop"));
  while (binFile.read(&block, 512) == 512) {
    if (block.count == 0) break;
    if (block.overrun) {
      if (!headerPrinted) {
        Serial.println();
        Serial.println(F("Overruns:"));
        Serial.println(F("fileBlockNumber,sdBlockNumber,overrunCount"));
        headerPrinted = true;
      }
      Serial.print(bn);
      Serial.print(',');
      Serial.print(bgnBlock + bn);
      Serial.print(',');
      Serial.println(block.overrun);
    }
    bn++;
  }
  if (!headerPrinted) {
    Serial.println(F("No errors found"));
  } 
  else {
    Serial.println(F("Done"));
  }
}
/*=============DUMP DATA TO SERIAL=============*/
// dump data file to Serial
void dumpData() {
  block_t block;
  if (!binFile.isOpen()) {
    Serial.println();
    Serial.println(F("No current binary file"));
    return;
  }
  binFile.rewind();
  Serial.println();
  Serial.println(F("Type any character to stop"));
  delay(1000);
  printHeader(&Serial);
  while (!Serial.available() && binFile.read(&block , 512) == 512) {
    if (block.count == 0) break;
    if (block.overrun) {
      Serial.print(F("OVERRUN,"));
      Serial.println(block.overrun);
    }
    for (uint16_t i = 0; i < block.count; i++) {
      printData(&Serial, &block.data[i]);
    }
  }
  Serial.println(F("Done"));
}
/*================================LOG DATA==========================*/
// log data
// max number of blocks to erase per erase call
uint32_t const ERASE_SIZE = 262144L;
void logData() {
  uint32_t bgnBlock, endBlock;

  // Allocate extra buffer space.
  block_t block[BUFFER_BLOCK_COUNT];
  block_t* curBlock = 0;
  Serial.println();

  // Find unused file name.
  if (BASE_NAME_SIZE > 6) {
    error("FILE_BASE_NAME too long");
  }
  while (sd.exists(binName)) {
    delay(10);
    if (binName[BASE_NAME_SIZE + 1] != '9') {
      binName[BASE_NAME_SIZE + 1]++;
    } 
    else {
      binName[BASE_NAME_SIZE + 1] = '0';
      if (binName[BASE_NAME_SIZE] == '9') {
        error("Can't create file name");
      }
      binName[BASE_NAME_SIZE]++;
    }
  }
  // Delete old tmp file.
  if (sd.exists(TMP_FILE_NAME)) {
    Serial.println(F("Deleting tmp file"));
    if (!sd.remove(TMP_FILE_NAME)) {
      error("Can't remove tmp file");
    }
  }
  // Create new file.
  Serial.println(F("Creating new file"));
  binFile.close();
  if (!binFile.createContiguous(sd.vwd(),
  TMP_FILE_NAME, 512 * FILE_BLOCK_COUNT)) {
    error("createContiguous failed");
  }
  // Get the address of the file on the SD.
  if (!binFile.contiguousRange(&bgnBlock, &endBlock)) {
    error("contiguousRange failed");
  }
  // Use SdFat's internal buffer.
  uint8_t* cache = (uint8_t*)sd.vol()->cacheClear();
  if (cache == 0) error("cacheClear failed"); 

  // Flash erase all data in the file.
  Serial.println(F("Erasing all data"));
  uint32_t bgnErase = bgnBlock;
  uint32_t endErase;
  while (bgnErase < endBlock) {
    delay(10);
    endErase = bgnErase + ERASE_SIZE;
    if (endErase > endBlock) endErase = endBlock;
    if (!sd.card()->erase(bgnErase, endErase)) {
      error("erase failed");
    }
    bgnErase = endErase + 1;
  }
  // Start a multiple block write.
  if (!sd.card()->writeStart(bgnBlock, FILE_BLOCK_COUNT)) {
    error("writeBegin failed");
  }
  // Initialize queues.
  emptyHead = emptyTail = 0;
  fullHead = fullTail = 0;

  // Use SdFat buffer for one block.
  emptyQueue[emptyHead] = (block_t*)cache;
  emptyHead = queueNext(emptyHead);

  // Put rest of buffers in the empty queue.
  for (uint8_t i = 0; i < BUFFER_BLOCK_COUNT; i++) {
    delay(10);
    emptyQueue[emptyHead] = &block[i];
    emptyHead = queueNext(emptyHead);
  }
  Serial.println(F("Logging - type any character to stop"));
  // Wait for Serial Idle.
  Serial.flush();
  delay(10);
  uint32_t bn = 0;
  uint32_t t0 = millis();
  uint32_t t1 = t0;
  uint32_t overrun = 0;
  uint32_t overrunTotal = 0;
  uint32_t count = 0;
  uint32_t maxLatency = 0;
  int32_t diff;
  // Start at a multiple of interval.
  uint32_t logTime = micros()/LOG_INTERVAL_USEC + 1;
  logTime *= LOG_INTERVAL_USEC;
  bool closeFile = false;
  while (1) {
    // Time for next data record.
    logTime += LOG_INTERVAL_USEC;
    //if (Serial.available()) closeFile = true;
    //if (s<30 && s >10) closeFile = true;
    if(digitalRead(buttonPin) == LOW) closeFile = true; //--------WAIT FOR PUSHBUTTON

    if (closeFile) {
      if (curBlock != 0 && curBlock->count >= 0) {
        // Put buffer in full queue.
        fullQueue[fullHead] = curBlock;
        fullHead = queueNext(fullHead);
        curBlock = 0;
      }   
    } 
    else {
      if (curBlock == 0 && emptyTail != emptyHead) {
        curBlock = emptyQueue[emptyTail];
        emptyTail = queueNext(emptyTail);
        curBlock->count = 0;
        curBlock->overrun = overrun;
        overrun = 0;
      }
      do {
        diff = logTime - micros();
      } 
      while(diff > 0);
      //if (diff < -10) error("LOG_INTERVAL_USEC too small");----COMMENTED OUT FOR IT TO WORK
      if (curBlock == 0) {
        overrun++;
      } 
      else {
        
        //for(reduceSDwriteCount = 0;reduceSDwriteCount < 100;reduceSDwriteCount++){
        //  checkData();
        //}
       
        
        acquireData(&curBlock->data[curBlock->count++]);
        
        //fakeacquireData(&curBlock->data[curBlock->count++]);
       
         //while(1){
           
           //digitalWrite(ledPin,LOW);
           //delay(500);
           //digitalWrite(ledPin,HIGH);
           //delay(1000); 
           
           //delay(3000);
           //checkData();
           
         //}
       
            //Serial.print("freeMemory()=");
            //Serial.println(freeMemory());

            //delay(1000);
       

        if (curBlock->count == DATA_DIM) {
          fullQueue[fullHead] = curBlock;
          fullHead = queueNext(fullHead);        
          curBlock = 0;
        }
      }
    }

    if (fullHead == fullTail) {
      // Exit loop if done.
      if (closeFile) break;
    } 
    else if (!sd.card()->isBusy()) {
      // Get address of block to write.
      block_t* pBlock = fullQueue[fullTail];
      fullTail = queueNext(fullTail);     
      // Write block to SD.
      uint32_t usec = micros();
      if (!sd.card()->writeData((uint8_t*)pBlock)) {
        error("write data failed");
      }
      usec = micros() - usec;
      t1 = millis();
      if (usec > maxLatency) maxLatency = usec;
      count += pBlock->count;

      // Add overruns and possibly light LED. 
      if (pBlock->overrun) {
        overrunTotal += pBlock->overrun;
        if (ERROR_LED_PIN >= 0) {
          digitalWrite(ERROR_LED_PIN, LOW);
        }
      }
      // Move block to empty queue.
      emptyQueue[emptyHead] = pBlock;
      emptyHead = queueNext(emptyHead);
      bn++;
      if (bn == FILE_BLOCK_COUNT) {
        // File full so stop
        break;
      }
    }
  }

  if (!sd.card()->writeStop()) {
    error("writeStop failed");
  }

  // Truncate file if recording stopped early.
  if (bn != FILE_BLOCK_COUNT) {    
    Serial.println(F("Truncating file"));
    if (!binFile.truncate(512L * bn)) {
      error("Can't truncate file");
    }
  }

  if (!binFile.rename(sd.vwd(), binName)) {
    error("Can't rename file");
  }

  Serial.print(F("File renamed: "));
  Serial.println(binName);
  Serial.print(F("Max block write usec: "));
  Serial.println(maxLatency);
  Serial.print(F("Record time sec: "));
  Serial.println(0.001*(t1 - t0), 3);
  Serial.print(F("Sample count: "));
  Serial.println(count);
  Serial.print(F("Samples/sec: "));
  Serial.println((1000.0)*count/(t1-t0));
  Serial.print(F("Overruns: "));
  Serial.println(overrunTotal);
  Serial.println(F("Done"));
}

/*======================RTC INITIALIZATION=================*/
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

/*=================READ TIME DATE TO ARRAY===========*/
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

/*============PAUSE AND WAIT FOR BUTTON=======*/
void pauseForButton(){
  while(1){
    if(digitalRead(buttonPin) ==  HIGH) break;
  }
  while(1){
    if(digitalRead(buttonPin) == LOW) break;
    if(currentState == 1 ) blinkMyLedGreen();
  }
  while(1){
    if(digitalRead(buttonPin) ==  HIGH) break;
  }
}


/*=======LED BLINK FUNCTIONS====*/
void blinkMyLedGreen(){
  digitalWrite(ledPin,LOW);
  if(currentState == 1) delay(100);
  digitalWrite(ledPin,HIGH);
  if(currentState == 1) delay(100);
}


/*==========SERIAL COMMUNIATION THING (usually commented out)========*/

void SerialCommunication(){  
  // discard any input
  while (Serial.read() >= 0) {
  }
  Serial.println();
  Serial.println(F("type:"));
  Serial.println(F("c - convert file to CSV")); 
  Serial.println(F("d - dump data to Serial"));  
  Serial.println(F("e - overrun error details"));
  Serial.println(F("r - record data"));

  while(!Serial.available()) {
  }
  char c = tolower(Serial.read());

  // Discard extra Serial data.
  do {
    delay(10);
  } 
  while (Serial.read() >= 0);

  if (ERROR_LED_PIN >= 0) {
    digitalWrite(ERROR_LED_PIN, HIGH);
  }
  if (c == 'c') {
    binaryToCsv();
  } 
  else if (c == 'd') {
    dumpData();
  } 
  else if (c == 'e') {    
    checkOverrun();
  } 
  else if (c == 'r') {
    UpdateTimeDate(TimeDateGlobal);
    logData();
    binaryToCsv();
  } 
  else {
    Serial.println(F("Invalid entry"));
  }
}

/*==========================================SETUP===================================================*/
void setup(void) {
  if (ERROR_LED_PIN >= 0) {
    pinMode(ERROR_LED_PIN, OUTPUT);
  }

  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(ledPin,OUTPUT);
  pinMode(SDpowerPin,OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP);
  pinMode(buttoninterruptPin, INPUT);
 
  digitalWrite(SDpowerPin,HIGH);
  
  //my setup stuff
  pinMode(13, OUTPUT);
  Wire.begin();        // Join i2c bus

  myPressure.begin(); // Get sensor online

  //Configure the sensor
  myPressure.setModeAltimeter(); // Measure altitude above sea level in meters
  //myPressure.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa

  myPressure.setOversampleRate(7); // Set Oversample to the recommended 128
  myPressure.enableEventFlags(); // Enable all three pressure and temp event flags 

    ////Initiate an SPI communication instance.
  SPI.begin();
  ////Configure the SPI connection for the ADXL345.
  SPI.setDataMode(SPI_MODE3);
  ////Set up the Chip Select pin to be an output from the Arduino.
  pinMode(chipSelectADXL, OUTPUT);
  ////Before communication starts, the Chip Select pin needs to be set high.
  digitalWrite(chipSelectADXL, HIGH);
  ////Put the ADXL345 into +/- 8G range by writing the value 0x01 to the DATA_FORMAT register.?
  writeRegister(DATA_FORMAT, 0x02);
  ////Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
  writeRegister(POWER_CTL, 0x08);  //Measurement mode  
  ////Set interrupts to all go to INT1
  writeRegister(INT_MAP,0x00);
  //D7-D0: DATA_READY, SINGLE_TAP, DOUBLE_TAP, Activity, Inactivity, FREE_FALL, Watermark, Overrun
  writeRegister(INT_ENABLE, 0x04); //enable freefall
  //unsigned rootsumsquare of all axes!!!try 0x09 to start
  writeRegister(THRESH_FF, HIGH_TH); //HIGH_TH, MEDIUM_TH, LOW_TH, HIGHER_TH
  //unsigned time, try 0x14 (100ms) or 350ms (0x46)
  writeRegister(TIME_FF, 0x14);
  //clear interrupts
  readRegister(INT_SOURCE, 1, values);



  //offsets to fix werid issuess
  writeRegister(0x1E, 0x01);
  writeRegister(0x1F, 0xFF);
  writeRegister(0x20, 0x03);

  RTC_init();

  Serial.begin(9600);

  Serial.print(F("FreeRam: "));
  Serial.println(FreeRam());
  Serial.print(F("Records/block: "));
  Serial.println(DATA_DIM);

  //if (!sd.init(SPI_FULL_SPEED, SD_CS_PIN)) {
  //  sd.initErrorHalt();
  //}  


  if (sizeof(block_t) != 512) error("Invalid block size");
  // initialize file system.
  if (!sd.begin(SD_CS_PIN, SPI_FULL_SPEED)) {
    sd.initErrorPrint();
    fatalBlink();
  }
  



}

/*========================================MAIN LOOP==================================*/
void loop(void) {

  
  
  
  
  digitalWrite(ERROR_LED_PIN, HIGH);
  UpdateTimeDate(TimeDateGlobal);

  //SerialCommunination();

  digitalWrite(ledPin,LOW);
  currentState = 0;
  pauseForButton();
  digitalWrite(ledPin,HIGH);
  logData();
  binaryToCsv();
  currentState = 1;
  pauseForButton();
 

  /*
  delay(1000);
  seconds++;
  
  Serial.print("Awake for ");
  Serial.print(seconds, DEC);
  Serial.println(" second");
  
  if(seconds == 3)
  {
    Serial.println("Entering sleep");
    delay(200);
    seconds = 0;
    enterSleep();
  }
  */
}




