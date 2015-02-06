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
 */
 
 /*===========================================DEFINITIONS===========================================*/
#include <SPI.h>
#define chipSelectRTC 8
#define SD_CS_PIN 9
#define chipSelectADXL 10

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
/*=================SET RTC FUNCTION===============*/
int SetTimeDate(int d, int mo, int y, int h, int mi, int s){ 
  int TimeDate [7]={
    s,mi,h,0,d,mo,y  };
  for(int i=0; i<=6;i++){
    if(i==3)
      i++;
    int b= TimeDate[i]/10;
    int a= TimeDate[i]-b*10;
    if(i==2){
      if (b==2)
        b=B00000010;
      else if (b==1)
        b=B00000001;
    }	
    TimeDate[i]= a+(b<<4);

    digitalWrite(chipSelectRTC, LOW);
    SPI.transfer(i+0x80); 
    SPI.transfer(TimeDate[i]);        
    digitalWrite(chipSelectRTC, HIGH);
  }
}
/*==========READ RTC AS STRING============*/
String ReadTimeDate(){

  SPI.setDataMode(SPI_MODE3);
  String temp;
  int TimeDate [7]; //second,minute,hour,null,day,month,year		
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
      TimeDate[i]=a+b;
    }
    else if(i==4){
      int b=(n & B00110000)>>4;
      TimeDate[i]=a+b*10;
    }
    else if(i==5){
      int b=(n & B00010000)>>4;
      TimeDate[i]=a+b*10;
    }
    else if(i==6){
      int b=(n & B11110000)>>4;
      TimeDate[i]=a+b*10;
    }
    else{	
      int b=(n & B01110000)>>4;
      TimeDate[i]=a+b*10;	
    }
  }
  temp.concat(TimeDate[4]);
  temp.concat("/") ;
  temp.concat(TimeDate[5]);
  temp.concat("/") ;
  temp.concat(TimeDate[6]);
  temp.concat("     ") ;
  temp.concat(TimeDate[2]);
  temp.concat(":") ;
  temp.concat(TimeDate[1]);
  temp.concat(":") ;
  temp.concat(TimeDate[0]);
  return(temp);
}



/*==========================================SETUP===================================================*/
void setup(void) {
  ////Set up the Chip Select pin to be an output from the Arduino.
  pinMode(chipSelectADXL, OUTPUT);
  ////Before communication starts, the Chip Select pin needs to be set high.
  digitalWrite(chipSelectADXL, HIGH);
   ////Set up the Chip Select pin to be an output from the Arduino.
  pinMode(SD_CS_PIN, OUTPUT);
  ////Before communication starts, the Chip Select pin needs to be set high.
  digitalWrite(SD_CS_PIN, HIGH);
  RTC_init();
  Serial.begin(9600);
}

/*========================================MAIN LOOP==================================*/
void loop(void) {
   Serial.println("Current RTC time: ");
   Serial.print(ReadTimeDate());
  // discard any input
   while (Serial.read() >= 0) {}
   Serial.println();
   Serial.println();
   Serial.println(F("Set RTC:"));
   Serial.println(F("Enter Year (xx):")); 

   while(!Serial.available()) {}
   int year = Serial.parseInt();
   
   Serial.println();
   Serial.println(F("Enter Month (xx):")); 

   while(!Serial.available()) {}
   int month = Serial.parseInt();
  
   Serial.println();
   Serial.println(F("Enter Day (xx):")); 

   while(!Serial.available()) {}
   int day = Serial.parseInt();
   
   Serial.println();
   Serial.println(F("Enter Hour (xx):")); 

   while(!Serial.available()) {}
   int hour = Serial.parseInt();
   
   Serial.println();
   Serial.println(F("Enter Min (xx):")); 

   while(!Serial.available()) {}
   int minute = Serial.parseInt();
   
   Serial.println();
   Serial.println(F("Enter Second (xx):")); 

   while(!Serial.available()) {}
   int second = Serial.parseInt();
   
   // Discard extra Serial data.
   do {
   delay(10);
   } while (Serial.read() >= 0);
   
   SetTimeDate(day, month, year, hour, minute, second);
   Serial.println("New RTC time: ");
   Serial.println(ReadTimeDate());
   Serial.println();
   Serial.println();
   Serial.println();
   
   while(1){
     Serial.println(ReadTimeDate());
     delay(500);
   }
 

}



