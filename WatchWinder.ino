/*---------------------------------------------------------------------------- 
-Arduino IDE 1.0.5
-Hardware
  -Arduino Mega 2560 R3
  -NEMA 14 0.9deg (400 step/rev) 11Ncm stepper motor
  -SilentStepStick stepper motor driver
  -Adafruit Ultimate GPS Breakout [ID:746]
  -Adafruit RGB backlight negative LCD 20x4 + extras (RGB on black) [ID:498]
  -Adafruit i2c / SPI character LCD backpack[ID:292]

FUNCTIONS
-Adjustable number of turns per day (TPD)
  -increments of 5
-Adjustable direction
  -clockwise
  -counterclockwise
  -both
-Adjustable turn frequency
  -turn the watch ever n number of minutes?
  -calculate number of turns per cycle, depending on cycle frequency
-Full wind
  -run continuously, to wind a watch from a dead stop. 800 turns.
-GPS receiver
  -get time for clock to display on screen.
-Turn counter
  -displays number of turns since midnight.
----------------------------------------------------------------------------*/

#include <TimerOne.h>
#include <Wire.h>
//INSERT THE NEXT 4 INCLUDES AFTER INCLUDING WIRE LIBRARY
#include <LiquidTWI2.h>
#include <buttons.h>
#include <MENWIZ_LiquidTWI2.h>
#include <EEPROM.h>
//---------------------------------------------------
#include <AccelStepper.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Time.h>

//////////////////////////////////////////////////////////
//                      CONSTANTS                       //
//////////////////////////////////////////////////////////
const int  ESCAPE_BUTTON      =    9; //Escape Button
const int  CONFIRM_BUTTON     =   10; //Confirm Button
const int  DOWN_BUTTON        =   12; //Down Button
const int  UP_BUTTON          =   11; //Up Button
const int  w1ACT              =    8; //button to activate/deactivate watch slot
const int  w1SLEEP            =   24; //Driver Sleep Pin
const int  w1LED              =   43; //feedback light on w1ACT
const long REV                = 6400; //number of steps in 1 revolution of motor
const long RPM                =    8; //Speed at which the motor will spin
const long MINUTES_IN_DAY     = 1440; //minutes in 24 hours
const int  w1FULL_WIND        =  800; //Revolutions to fully wind from stop

char* w1ACTIVE_STAT[]={"STATUS: OFF", "STATUS: ON"}; //Used for Status screen.

//////////////////////////////////////////////////////////
//                  GLOBAL VARIABLES                    //
//////////////////////////////////////////////////////////
int     w1TPD                =              650; //Turns per day
int     w1DIR                =                0; //0=CW, 1=CCW, 2=BOTH used in menu
int     w1DIRECTION          =                1; //Used in functions to determine direction
int     w1CYCLE_TIME         =               15; //Minutes between cycles
int     w1CYCLE_TIME_S       =  w1CYCLE_TIME*60; //Seconds between cycles
int     w1CYCLE_TIME_LAST    =     w1CYCLE_TIME; //Used to determine change in CYCLE_TIME
long    w1STEPS                                ; //Use in WIND_NOW() function & full wind, and loop cleanup
int     w1FULL_WIND_STATUS   =                0; //0=OFF, 1=unidirectional, 2=bidirectional
int     w1FULL_WIND_COUNT    =                1; //Used to determine when to switch direction
boolean w1ACTIVE             =                1; //Is this slot turned on or off?
boolean w1ACT_STATE          =                0; //Used for determining w1ACTIVE
boolean w1ACT_LAST_STATE     =                0; //Used for determining w1ACTIVE
int     w1TURN_COUNTER       =                0; //Counts number of turns so far, since midnight
long    w1COUNTER_START      =                0; //Used for turn counter to count steps
int     LAST_HOUR            =                0; //Used to reset turn counter
boolean DST                  =                1; //Is DST in effect or not?
boolean DST_LAST             =              DST; 
int     ZONE_OFFSET          =               -5; //time zone offset
int     ZONE_OFFSET_LAST     =      ZONE_OFFSET; //time zone offset
boolean TIME_SET_FLAG        =            false; //Check if time has been synced to GPS
int     TIME_HOUR            =                0; //For clock on statusScreen screen
int     TIME_MIN             =                0;
int     TIME_SEC             =                0;
unsigned long SEC_COUNTER    =                0; //Used to see if a second has passed

//////////////////////////////////////////////////////////
//                    OBJECTS                           //
//////////////////////////////////////////////////////////

menwiz menu;
//Adafruit_LiquidCrystal lcd(0);
LiquidTWI2 lcd(0x20);

//CREATE ACCELSTEPPERS
AccelStepper w1STEPPER(1, 23, 22);  //1 pin=driver, 23=step pin, 22=direction pin

//GPS
HardwareSerial GPSSerial = Serial1;
Adafruit_GPS GPS(&GPSSerial);

//////////////////////////////////////////////////////////
//                       SETUP                          //
//////////////////////////////////////////////////////////
void setup()
{
  Serial.begin(19200);
  
  SEC_COUNTER = now();
  
  // SET UP LCD
  lcd.setMCPType(LTI_TYPE_MCP23008);
  lcd.begin(20, 4);
  
  //CHECK IF SETTINGS ARE STORED IN MEMORY & LOAD IF SO
  if (EEPROM.read(0) == 1)
  {
    w1TPD            = EEPROM.read(1) * 256 + EEPROM.read(2);
    w1DIR            = EEPROM.read(3);
    w1CYCLE_TIME     = EEPROM.read(4);
    ZONE_OFFSET      = EEPROM.read(5) - 12;
    ZONE_OFFSET_LAST = ZONE_OFFSET;
    DST              = EEPROM.read(6);
    DST_LAST         = DST;
  }
  
  //CONFIG PINS NOT PART OF MENU NAV
  pinMode(w1ACT, INPUT_PULLUP);
  pinMode(w1LED, OUTPUT);
  digitalWrite(w1LED, w1ACTIVE);
  pinMode(w1SLEEP, OUTPUT);
  digitalWrite(w1SLEEP, HIGH);

  //ACCELSTEPPERS
  w1STEPPER.setMaxSpeed((REV*RPM)/60.0);  //APPROX 8RPM
  w1STEPPER.setAcceleration((REV*RPM)/60.0); //takes 1 sec to reach full speed
  
  //GPS
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); 
  
  ////////////////////////////////////////////////
  //         MENU STRUCTURE STARTS HERE         //
  ////////////////////////////////////////////////
  _menu *r,*s1,*s2;
  _var *v; 
  
  //INIZIALIZE THE MENU OBJECT (20 COLUMS X 4 ROWS)
  menu.begin(&lcd,20,4);
  menu.setBehaviour(MW_MENU_INDEX,false);    

  //ROOT
  r=menu.addMenu(MW_ROOT,NULL,F("      SETTINGS"));    //create a root menu at first (required)
    //TPD
    s1=menu.addMenu(MW_VAR,r,F("TPD"));           //add a terminal node (that is "variable"); 
    s1->addVar(MW_AUTO_INT,&w1TPD,50,2000,10);    //create a variable of type auto int
    //DIR
    s1=menu.addMenu(MW_VAR,r,F("DIRECTION"));
    s1->addVar(MW_LIST,&w1DIR);
    s1->addItem(MW_LIST,F("CW"));                 //add option to the OPTION LIST
    s1->addItem(MW_LIST,F("CCW"));                //add option to the OPTION LIST
    s1->addItem(MW_LIST,F("BOTH"));               //add option to the OPTION LIST
    //CYCLE TIME
    s1=menu.addMenu(MW_VAR,r,F("FREQUENCY"));     //add a terminal node (that is "variable"); 
    s1->addVar(MW_AUTO_INT,&w1CYCLE_TIME,1,60,1); //create a variable of type auto int...
    //FULL WIND
    s1=menu.addMenu(MW_VAR,r,F("FULL WIND"));     //SPIN FOR 800 REVS NONSTOP
    s1->addVar(MW_ACTION,fullWind);
    s1->setBehaviour(MW_ACTION_CONFIRM,true);
    //CLOCK
    s1=menu.addMenu(MW_SUBMENU,r,F("CLOCK"));
      //DST
      s2=menu.addMenu(MW_VAR,s1,F("DST"));           //DST on/off?
      s2->addVar(MW_BOOLEAN,&DST);                     
      //TIME ZONE
      s2=menu.addMenu(MW_VAR,s1,F("TIME ZONE"));     //Change ZONE_OFFSET var for UTC offset
      s2->addVar(MW_AUTO_INT,&ZONE_OFFSET,-12,14,1);    
    //SAVE SETTINGS
    s1=menu.addMenu(MW_VAR,r,F("SAVE SETTINGS"));    //save TPD/Direction/Freq/DST/zone to EEPROM
    s1->addVar(MW_ACTION,saveSettings);
    s1->setBehaviour(MW_ACTION_CONFIRM,true);
        
  //DECLARE NAVIGATION BUTTONS (REQUIRED)
  menu.navButtons(UP_BUTTON,DOWN_BUTTON,ESCAPE_BUTTON,CONFIRM_BUTTON);

  //(optional)create a user define screen callback to activate after 5 secs (5000 millis) since last button push 
  menu.addUsrScreen(statusScreen,5000);

  //Timer1 INTERRUPTS - fires stepper.run() & reads GPS every 250 microseconds
  //using an interrupt to prevent menu.draw() from slowing down stepper motion.
  Timer1.initialize(250);
  Timer1.attachInterrupt(runWinder);
}

//////////////////////////////////////////////////////////
//                        LOOP                          //
//////////////////////////////////////////////////////////
void loop()
{
  //DRAW MENU AND MONITOR BUTTONS FOR MENU NAV
  menu.draw();

  //IF STEPPER NEEDS TO MOVE, WAKE IT UP
  if (w1STEPPER.distanceToGo() != 0)
  {
    digitalWrite(w1SLEEP, LOW);
  }
  
  //ACTIVATION BUTTON CHECK w1
  w1ACT_STATE = digitalRead(w1ACT);
  if (w1ACT_STATE == LOW && w1ACT_STATE != w1ACT_LAST_STATE)
  {
    w1ACTIVE = !w1ACTIVE;            //Toggle on/off w1
    digitalWrite(w1LED, w1ACTIVE);   //button feedback
  }
  w1ACT_LAST_STATE = w1ACT_STATE;

  //IF CYCLE TIMES CHANGE, UPDATE APPROPRIATE TIMER    
  if (w1CYCLE_TIME != w1CYCLE_TIME_LAST)
  {
    w1ChangeCycleTime();
  }
  
  //IF TIME ZONE OFFSET CHANGE, UPDATE CLOCK
  if (ZONE_OFFSET != ZONE_OFFSET_LAST)
  {
    changeTimeOffset();
  }
  
  if (DST != DST_LAST)
  {
    changeDSTOffset();
  }
  
  //STEPPER MOTION CLEANUP AND SLEEP
  if (w1STEPPER.distanceToGo() == 0)
  {
    //FINISHED FULL WIND EITHER CW OR CCW
    if (w1FULL_WIND_STATUS == 1)
    {
      w1FULL_WIND_STATUS = 0;
    }
    
    //FINISHED WITH FULL WIND IN BOTH DIRECTIONS
    if (w1FULL_WIND_COUNT == w1FULL_WIND/10)
    {
      w1FULL_WIND_STATUS = 0;
      w1FULL_WIND_COUNT = 1;
    }
    
    //NOT FINISHED WITH FULL WIND IN BOTH DIRECTIONS
    if (w1FULL_WIND_STATUS == 2)
    {
      w1FULL_WIND_COUNT = w1FULL_WIND_COUNT + 1;
      w1DIRECTION = -1*w1DIRECTION;
      w1STEPS = 10*REV;
      w1STEPPER.move(w1DIRECTION*w1STEPS);
    }
    
    if (w1STEPPER.distanceToGo() == 0)
    {
      digitalWrite(w1SLEEP, HIGH);
    }
  }

  //TURN COUNTER
  if (w1COUNTER_START - abs(w1STEPPER.distanceToGo()) >= REV)
  {
    w1COUNTER_START = abs(w1STEPPER.distanceToGo());
    w1TURN_COUNTER = w1TURN_COUNTER + 1;
  }
  
  //SECOND COUNTER
  if (now() > SEC_COUNTER)
  {
    SEC_COUNTER = now();
    everySecTimer();
  }
  
  //WAIT FOR GPS TO SYNC CLOCK ON STARTUP
  if (GPS.fix == 1 && TIME_SET_FLAG == false)
  {
    TIME_SET_FLAG = true;
    timeSync();
  }
  
  //IF NEW DATA WAITING FROM GPS, PARSE IT
  if (GPS.newNMEAreceived()) 
  {
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
    return;
  }
}

///////////////////////////////////
//             CLOCK             //
///////////////////////////////////
void clock()
{
  //INCREMENT 1 SEC
  TIME_SEC = TIME_SEC + 1;
  
  //END OF CURRENT MINUTE.
  if (TIME_SEC == 60)
  {
    TIME_SEC = 0;
    TIME_MIN = TIME_MIN + 1;
  }
  //END OF CURRENT HOUR.
  if (TIME_MIN == 60)
  {
    //IF TURN OF MIDNIGHT
    if (TIME_HOUR == 23)
    {
      w1TURN_COUNTER = 0;
    }
    timeSync();
  }
}

///////////////////////////////////
//           COUNTDOWN           //
///////////////////////////////////
void countdown()
{
  //IF SLOT IS ACTIVE AND NOT CURRENTLY IN FULL WIND MODE...
  if(w1ACTIVE == 1 && w1FULL_WIND_STATUS == 0)
  {
    w1CYCLE_TIME_S = w1CYCLE_TIME_S - 1;
    if (w1CYCLE_TIME_S < 0)
    {
      w1WindNow();
    }   
  }
}

///////////////////////////////////
//   CYCLE TIME WAS CHANGED.     //
// UPDATE TIMERS WITH NEW VALUE  //
///////////////////////////////////
void w1ChangeCycleTime()
{
  w1CYCLE_TIME_LAST = w1CYCLE_TIME;
  w1CYCLE_TIME_S = w1CYCLE_TIME*60;
}

///////////////////////////////////
// TIME ZONE OFFSET WAS CHANGED  //
///////////////////////////////////
void changeTimeOffset()
{
  TIME_HOUR = TIME_HOUR+(ZONE_OFFSET-ZONE_OFFSET_LAST);
  ZONE_OFFSET_LAST = ZONE_OFFSET;
}

//CAN/SHOULD I COMBINE THIS INTO ABOVE?
void changeDSTOffset()
{
  if (DST)
  { 
    if (TIME_HOUR != 23)
    {
      TIME_HOUR = TIME_HOUR+1;
    }
    else
    {
      TIME_HOUR = 0;
    }
  }
  if (!DST)
  {
    if (TIME_HOUR != 0)	 //add this if/else to deal with changing DST setting after clock is midnight. Currently changes hour to -1 which is no good.
    {
      TIME_HOUR = TIME_HOUR-1;
    }
    else
    {
      TIME_HOUR = 23;
    }
  }
  DST_LAST=DST;
}

///////////////////////////////////
//        EVERY SEC TIMER        //
///////////////////////////////////
void everySecTimer()
{
  countdown();
  clock();
}

///////////////////////////////////
//          FULLY WIND           //
///////////////////////////////////
void fullWind()
{
  if (w1FULL_WIND_STATUS != 0)
  {
    w1FULL_WIND_STATUS = 0;
    w1STEPPER.stop();
    w1FULL_WIND_COUNT = 1;
    return;
  }
  
  if (w1FULL_WIND_STATUS == 0)
  {
    switch (w1DIR)
    {
      //CLOCKWISE FULL WIND
      case 0:
        w1FULL_WIND_STATUS = 1;
        w1DIRECTION = -1;
        w1STEPS = w1FULL_WIND*REV;
        break;
        
      //COUNTER CLOCKWISE FULL WIND
      case 1:
        w1FULL_WIND_STATUS = 1;
        w1DIRECTION = 1;
        w1STEPS = w1FULL_WIND*REV;
        break;
        
      //BOTH DIRECTION FULL WIND
      case 2:
        w1FULL_WIND_STATUS = 2;
        w1DIRECTION = -1*w1DIRECTION;
        w1STEPS = 10*REV;
        break;
    }
    w1STEPPER.move(w1DIRECTION*w1STEPS);
  }
}

///////////////////////////////////
//          RUN STEPPER          //
///////////////////////////////////
void runWinder()
{
  //COLLECT DATA FROM GPS
  GPS.read();
   
  //RUN WINDER IF STEPS ARE WAITING
  if (w1STEPPER.distanceToGo() != 0)
  {
    w1STEPPER.run();
  }
}

///////////////////////////////////
//         SAVE SETTINGS         //
///////////////////////////////////
void saveSettings()
{
  //WRITE INDICATION INFO IS SAVED
  EEPROM.write(0,1);
  
  //SPLIT w1TPD INTO BYTE SIZED VARS
  int a = w1TPD/256;
  int b = w1TPD%256;
  EEPROM.write(1,a);
  EEPROM.write(2,b);
  
  //STORE w1DIR
  EEPROM.write(3,w1DIR);
  
  //STORE FREQUENCY
  EEPROM.write(4,w1CYCLE_TIME);

  //STORE TIME ZONE
  int UZO = ZONE_OFFSET + 12;
  EEPROM.write(5,UZO);
  
  //STORE DST
  EEPROM.write(6,DST);
}

///////////////////////////////////
//         STATUS SCREEN         //
///////////////////////////////////
void statusScreen()
{
  static char STATS[80];
  STATS[0]=0;
  static char buf_turn[7];
  static char buf_min[3];
  static char buf_sec[3];
  static char buf_timeh[3];
  static char buf_timem[3];
  static char buf_times[3];
  
  //STATUS
  strcat(STATS,w1ACTIVE_STAT[w1ACTIVE]);
  strcat(STATS,"\n");
  
  //TURNS
  strcat(STATS," TURNS: ");
  strcat(STATS,itoa(w1TURN_COUNTER,buf_turn,10));
  strcat(STATS,"\n");
  
  //NEXT CYCLE
  int COUNTDOWN_MIN = w1CYCLE_TIME_S/60;
  int COUNTDOWN_SEC = w1CYCLE_TIME_S%60;
  int COUNTDOWN_MIN_TENS = COUNTDOWN_MIN/10;
  int COUNTDOWN_MIN_ONES = COUNTDOWN_MIN%10;
  int COUNTDOWN_SEC_TENS = COUNTDOWN_SEC/10;
  int COUNTDOWN_SEC_ONES = COUNTDOWN_SEC%10;
  strcat(STATS,"  NEXT: ");
  strcat(STATS,itoa(COUNTDOWN_MIN_TENS,buf_timem,10)); //minutes 10s place
  strcat(STATS,itoa(COUNTDOWN_MIN_ONES,buf_timem,10)); //minutes  1s place
  strcat(STATS,":");
  strcat(STATS,itoa(COUNTDOWN_SEC_TENS,buf_times,10)); //seconds 10s place
  strcat(STATS,itoa(COUNTDOWN_SEC_ONES,buf_times,10)); //seconds  1s place
  strcat(STATS,"\n");
  
  //TIME
  int TIME_MIN_TENS = TIME_MIN/10;
  int TIME_MIN_ONES = TIME_MIN%10;
  int TIME_SEC_TENS = TIME_SEC/10;
  int TIME_SEC_ONES = TIME_SEC%10;
  strcat(STATS,"  TIME: ");
  if (TIME_SET_FLAG == false)
  {
    strcat(STATS,"WAITING...");
  }
  if (TIME_SET_FLAG == true)
  {
    //HOUR AM
    if (TIME_HOUR <= 12)
    {
      if (TIME_HOUR != 0)
      {
        strcat(STATS,itoa(TIME_HOUR,buf_timeh,10));
      }
      if (TIME_HOUR == 0)
      {
        strcat(STATS,itoa(12,buf_timeh,10));
      }
    }
    //HOUR PM
    if (TIME_HOUR > 12)
    {
      int TIME_HOUR_PM = TIME_HOUR - 12;
      strcat(STATS,itoa(TIME_HOUR_PM,buf_timeh,10));
    }
    strcat(STATS,":");
    strcat(STATS,itoa(TIME_MIN_TENS,buf_timem,10)); //minutes 10s place
    strcat(STATS,itoa(TIME_MIN_ONES,buf_timem,10)); //minutes  1s place
    strcat(STATS,":");
    strcat(STATS,itoa(TIME_SEC_TENS,buf_times,10)); //seconds 10s place
    strcat(STATS,itoa(TIME_SEC_ONES,buf_times,10)); //seconds  1s place
    strcat(STATS," ");
    //AM
    if (TIME_HOUR < 12)
    {
      strcat(STATS,"AM");  
    }
    //PM
    if (TIME_HOUR >= 12)
    {
      strcat(STATS,"PM");  
    }
    
    strcat(STATS,"\n");  
  }
  menu.drawUsrScreen(STATS);
}

///////////////////////////////////
//           TIME SYNC           //
///////////////////////////////////
time_t timeSync()
{
  TIME_HOUR = (GPS.hour+ZONE_OFFSET+DST+24)%24;
  TIME_MIN  = GPS.minute;
  TIME_SEC  = GPS.seconds; 
}

/////////////////////////////////////////////
// w1WindNow CALC WIND STEPS AND DIRECTION //
/////////////////////////////////////////////
void w1WindNow()
{
  if (w1ACTIVE == true)
  {
    switch (w1DIR)
    {
      //CLOCKWISE
    case 0:
      w1DIRECTION = -1;
      break;
      //COUNTER CLOCKWISE
    case 1:
      w1DIRECTION =  1;
      break;
      //BOTH
    case 2:
      w1DIRECTION = -1*w1DIRECTION;
      break;
    }
    w1STEPS = (w1TPD*REV)/(MINUTES_IN_DAY/w1CYCLE_TIME);
    noInterrupts();  //prevents Timer1 interrupt from moving motor and changing remaining steps for step counting calcs
    w1STEPPER.move(w1DIRECTION*w1STEPS);
    
    //TURN STEP COUNTER
    w1COUNTER_START = w1COUNTER_START + abs(w1STEPPER.distanceToGo());
    interrupts();  //reinitalize interrupts now that w1COUNTER_START is set to accurate value for calcs
  }
  
  //RESET COUNT DOWN
  w1CYCLE_TIME_S = w1CYCLE_TIME*60-1;
}
