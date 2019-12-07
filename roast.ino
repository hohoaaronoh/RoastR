/*
 *  Updated 2019-09-22
 *  
 */
// initialize var for roast name
String name;
 
/* MAX THERMOCOUPLE  include library and set pins       */
# include <max6675.h>
int thermoDO = 4; int thermoCS = 5; int thermoCLK = 6;
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);
int vccPin = 3; int gndPin = 2;

/* other pins  for heating element push button and indicator LED               */
int HEAT = 9;                // heater pin
const int ledPin =  11;      // the number of the LED pin
const int buttonPin = 12;     // the number of the pushbutton pin

/*  Other Variables       */
float temperature_read = 0.0;
float previous_error = 0; 
float elapsedTime, Time, timePrev, cumulativeTime;
int PID_error = 0; 

/*  PID initial values  */
int PID_value = 0;
float StartTime = 0;

/*  PID constants   */
int kp = 7;  
int kd = 0;  
int Range = 12;  
int set_pt;
float ki = 0.1;

/* Set initial Error values   */
int PID_p = 0;    
int PID_d = 0; 
float PID_i = 115;   

// variables will change:
int buttonState = 0;         // variable for reading the pushbutton status
int start = 1;
int ledValue = 0;

void setup() {
   
  Serial.begin(9600);   // start serial
  // use Arduino pins 
  pinMode(vccPin, OUTPUT); digitalWrite(vccPin, HIGH);
  pinMode(gndPin, OUTPUT); digitalWrite(gndPin, LOW);
  pinMode(HEAT, OUTPUT);
  
  // wait for MAX chip to stabilize
  delay(700);
  
  // write header line to serial monitor for later CSV reading
  Serial.println("RoastName, Time, Duration, PV, SP, PID_error, PID_value, PID_p, PID_i, PID_d, kp, ki, kd, Range");
  
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);

}

void loop() {

buttonState = digitalRead(buttonPin);   // check if start button is pressed

/*               blink while waiting                         */
  digitalWrite(ledPin, HIGH);
  delay(200);
  digitalWrite(ledPin, LOW);
  delay(200);

/* if button is pressed state = 1 start quick blink */
if(buttonState == 1){
digitalWrite(ledPin, LOW);
StartTime = millis();   // restart the clock so roasts start at 0



/* ROAST PROFILE SECTION ------------------------------------------- */

// Uncomment the profile to use 

/* SHort light roast  7min to 375   
 *  
name = "Light_Coffee";  // name the coffee

 roast( 180 , 280 );
 roast( 228 , 299 );
 roast( 276 , 318 );
 roast( 324 , 337 );
 roast( 372 , 356 );
 roast( 420 , 375 );
 roast( 467 , 390 );
 roast( 513 , 405 );
 roast( 560 , 420 );
        cool( 740 );

 */

/*   Medium 10 min to 375 finish at 420 -------------------------- 
 *    375-10-420
 */    
// 
name = "Medium_Coffee";  // name the coffee

 roast( 180 , 290 );
 roast( 264 , 299 );
 roast( 348 , 318 );
 roast( 432 , 337 );
 roast( 516 , 356 );
 roast( 600 , 380 );
 roast( 667 , 390 );
 roast( 733 , 405 );
 roast( 800 , 420 );
        cool( 980 );

/*
 //LONG - dark roast  -----------------------------------------

name = "Dark_Coffee";  // name the coffee

roast( 180 , 290 );
 roast( 276 , 299 );
 roast( 372 , 318 );
 roast( 468 , 337 );
 roast( 564 , 356 );
 roast( 660 , 375 );
 roast( 733 , 390 );
 roast( 807 , 405 );
 roast( 880 , 425 );
       cool( 1060 );

*/

/* ------------ end of roast profiles ----------------------------*/
  
}
/* exit the roast loops */

/* blink sequence all done with roasting */

}


/* ----------------------------------------
/*        DEFINE CUSTOM FXNS  */

/* -----------------------------------------------------------------------------
     define ROAST
 ----------------*/ 
  
void roast( int D, int set_pt) {

  while ( cumulativeTime < D ){
    
/*        calculate cumulative time since start of while loop  */  
    cumulativeTime = (millis() - StartTime) / 1000;  // cumulative time since the start of the program
/*  run the PID algorithm  */
/*  First we read the real value of temperature at thermocouple  */
    temperature_read = thermocouple.readFahrenheit();
/*  Next we calculate the error between the setpoint and the real value */
    PID_error = set_pt - temperature_read;
/*  Calc the P, I and D components   */
/*  Proportional term */
    PID_p = kp * PID_error;
/*  Integral term   */
/*  Calculate the I value in a range on: 10  only kicks in when the error is within 10 deg  */
    int range = abs(PID_error) - Range;
      if(range < 1)
      {
        PID_i = PID_i + (ki * PID_error);
      }
/*     Cap contribution of PID_i to 250  */
      if(PID_i > 250)  
      {    
        PID_i = 250;  
      }
      
    /* Derivative term    */  
    /* elapsed time */
    timePrev = Time;                            // the previous time is stored before the actual time read
    Time = millis();                            // actual time read
    elapsedTime = (Time - timePrev) / 1000; 
    /* Now we can calculate the D calue  */
    PID_d = kd * (( PID_error - previous_error ) / elapsedTime);
    /* total PID is the sum of P + I + D  */
    PID_value = PID_p + PID_i + PID_d;
    /* write to PWM pin with max of 255 and min of 0  */
    if(PID_value < 0)
    {    
      PID_value = 0;   
    }
    if(PID_value > 255)  
    {    
      PID_value = 255;  
    }

/* PRINT TO SERIAL PORT TO SAVE FOR DATA ANALYSIS   //
                                                            */
    Serial.print(name);                 Serial.print(", ");   // Roast Name
    Serial.print(cumulativeTime);       Serial.print(", ");   // run time so far
    Serial.print(D );                   Serial.print(", ");   // Duration as Set Point for the current roast stage
    Serial.print(temperature_read  );   Serial.print(", ");   // PV  as bean temp at thermocouple in deg F
    Serial.print(set_pt );              Serial.print(", ");   // SP  for roast stage
    Serial.print(PID_error);            Serial.print(", ");   // PID error sum of P, I and D values
    Serial.print(PID_value);            Serial.print(", ");   // PID value new output to PWM pin
    /* Capture the computed components too   */
    Serial.print(PID_p);                Serial.print(", ");   // PIDp
    Serial.print(PID_i);                Serial.print(", ");   // PIDi
    Serial.print(PID_d);                Serial.print(", ");   // PIDd
    /*  and record settings  */
    Serial.print(kp);                   Serial.print(", ");   // kp
    Serial.print(ki);                   Serial.print(", ");   // ki
    Serial.print(kd);                   Serial.print(", ");   // kid
    Serial.println(Range); //range
    /* write the value to the pin / heater */
    analogWrite(HEAT, PID_value);
    /* led to indicate amt of heat */
    if(PID_error < 5)
    {
      ledValue = 2;
    }
    ledValue = PID_error;

    analogWrite(ledPin, ledValue);

    previous_error = PID_error;     //Remember to store the previous error for next loop.
    delay(1000);                    // wait a sec
  }  
}

/* ----------------------------------------------------------------
     define Cooling cycle
 ----------------*/  

void cool(int D) {
   // takes Duration (D) as an argument
cumulativeTime = (millis() - StartTime) / 1000;  // cumulative time since the start of the program
  while ( cumulativeTime < D  ){
    cumulativeTime = (millis() - StartTime) / 1000;  // cumulative time since the start of the program
    //RUN PID
    // First we read the real value of temperature
    temperature_read = thermocouple.readFahrenheit();
    //Next we calculate the error between the setpoint and the real value
    PID_error = set_pt - temperature_read;
    PID_value = 0;
// PRINT VALUES
    Serial.print(name);                             Serial.print(", ");    // Time
    Serial.print((millis() - StartTime) / 1000);    Serial.print(", ");
    Serial.print(D );                               Serial.print(", ");    // PV  
    Serial.print(temperature_read);                 Serial.print(", ");    // SP
    Serial.print('0');                              Serial.print(", ");    // PID error
    Serial.print(PID_error);                        Serial.print(", ");    // PID value
    Serial.print(PID_value);                        Serial.print(", ");    // Capture the computed components too
    Serial.print(PID_p);                            Serial.print(", ");    // PIDp
    Serial.print(PID_i);                            Serial.print(", ");    // PIDi
    Serial.print(PID_d);                            Serial.print(", ");    // PIDd
    Serial.print(kp);                               Serial.print(", ");    // ki
    Serial.print(ki);                               Serial.print(", ");    // kid
    Serial.print(kd);                               Serial.print(", ");    // range
    Serial.println(Range);
  
//Now we can write the PWM signal to the mosfet on digital pin D9
    analogWrite(HEAT, PID_value);
    digitalWrite(ledPin, LOW);
    delay(500);
    digitalWrite(ledPin, HIGH);
    delay(500);
  }  // ends WHILE  
}
