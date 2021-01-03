/*
    Let baby interact with mobile

    Functionality:

    Version log:

    2020-12-27:
        v1.0.0  - Initial release

    Links

        https://github.com/arkhipenko/pumpkin/blob/master/Pumpkin_Firmware.ino (Sample)
*/
// ==== DEFINES ===================================================================================

#define LED_PIN 13
#define MOTION_PIN 7
#define MIC_PIN A0

#define SONAR_ECHO_PIN 6
#define SONAR_TRIG_PIN 5

#define FULL_STEP               8
#define STEPPER_PIN_1           8
#define STEPPER_PIN_2           9
#define STEPPER_PIN_3           10
#define STEPPER_PIN_4           11

// ==== Debug and Test options ==================
#define _DEBUG_
#define _TEST_

//===== Debugging macros ========================
#ifdef _DEBUG_
#define SerialD Serial
#define _PM(a) SerialD.print(millis()); SerialD.print(": "); SerialD.println(a)
#define _PP(a) SerialD.print(a)
#define _PL(a) SerialD.println(a)
#define _PX(a) SerialD.println(a, HEX)
#else
#define _PM(a)
#define _PP(a)
#define _PL(a)
#define _PX(a)
#endif




// ==== INCLUDES ==================================================================================

// ==== Uncomment desired compile options =================================
#define _TASK_SLEEP_ON_IDLE_RUN  // Enable 1 ms SLEEP_IDLE powerdowns between tasks if no callback methods were invoked during the pass
// #define _TASK_TIMECRITICAL       // Enable monitoring scheduling overruns
// #define _TASK_STATUS_REQUEST     // Compile with support for StatusRequest functionality - triggering tasks on status change events in addition to time only
// #define _TASK_WDT_IDS            // Compile with support for wdt control points and task ids
// #define _TASK_LTS_POINTER        // Compile with support for local task storage pointer
// #define _TASK_PRIORITY           // Support for layered scheduling priority
// #define _TASK_MICRO_RES          // Support for microsecond resolution
// #define _TASK_STD_FUNCTION       // Support for std::function (ESP8266 and ESP32 ONLY)
// #define _TASK_DEBUG              // Make all methods and variables public for debug purposes
// #define _TASK_INLINE             // Make all methods "inline" - needed to support some multi-tab, multi-file implementations
// #define _TASK_TIMEOUT            // Support for overall task timeout
// #define _TASK_OO_CALLBACKS       // Support for dynamic callback method binding
// #define _TASK_DEFINE_MILLIS      // Force forward declaration of millis() and micros() "C" style
// #define _TASK_EXPOSE_CHAIN       // Methods to access tasks in the task chain
// #define _TASK_SCHEDULING_OPTIONS // Support for multiple scheduling options

#include <TaskScheduler.h>
#include <AccelStepper.h>
//#include <PinChangeInterrupt.h>
//#include <U8glib.h>
#include <avr/sleep.h>
#include <avr/power.h>



// ==== GLOBALS ===================================================================================


// display
//U8GLIB_SSD1306_128X32 u8g(U8G_I2C_OPT_NONE);

//String display = "Initial";

// have to chech if seq of pin is right. pin 3 and 2 !! https://www.youtube.com/watch?v=0qwrnUeSpYQ
AccelStepper motor = AccelStepper(FULL_STEP, STEPPER_PIN_1, STEPPER_PIN_3, STEPPER_PIN_2, STEPPER_PIN_4);
unsigned long motorStartTs = 0;
unsigned long motorStopTs = 0;
unsigned long motorDuration = 60000;
short motorDirection = 1;


//unsigned long motionDuration = motorDuration * 1.25;
unsigned long micPauseDuration = 5000;
unsigned long micPauseUntil = 0;

unsigned long motionPauseDuration = 15000;
unsigned long motionPauseUntil = 0;

// ==== Scheduler ==============================
Scheduler ts;

void cbTimeout();
void onMotion();

void onMic();
void onSonar();

void onMotor();
bool onMotorEnable();
void onMotorDisable();

// ==== Scheduling defines (cheat sheet) =====================
/*
  TASK_MILLISECOND
  TASK_SECOND
  TASK_MINUTE
  TASK_HOUR
  TASK_IMMEDIATE
  TASK_FOREVER
  TASK_ONCE
  TASK_NOTIMEOUT

  TASK_SCHEDULE     - schedule is a priority, with "catch up" (default)
  TASK_SCHEDULE_NC  - schedule is a priority, without "catch up"
  TASK_INTERVAL     - interval is a priority, without "catch up"
*/

// ==== Task definitions ========================
Task tMotion(TASK_IMMEDIATE, TASK_FOREVER, &onMotion, &ts, true);
Task tMic(TASK_IMMEDIATE, TASK_FOREVER, &onMic, &ts, true);

//Task tSonar(TASK_IMMEDIATE, TASK_FOREVER, &onSonar, &ts, true);

//Task tTimeout (30 * TASK_SECOND, TASK_FOREVER, &cbTimeout, &ts, true);
//
Task tMotor(TASK_IMMEDIATE, TASK_FOREVER, &onMotor, &ts, false, &onMotorEnable, &onMotorDisable);



// ==== CODE ======================================================================================

/**************************************************************************/
/*!
    @brief    Standard Arduino SETUP method - initialize sketch
    @param    none
    @returns  none
*/
/**************************************************************************/
void setup() {
    //pinMode(MOTION_PIN, INPUT);
    //pinMode(LED_PIN, OUTPUT);

    //pinMode(SONAR_TRIG_PIN, OUTPUT); // Sets the trigPin as an Output
    //pinMode(SONAR_ECHO_PIN, INPUT);


    //motor.setAcceleration(500);

    // flip screen, if required
    // u8g.setRot180();


    // assign default color value
    /*if ( u8g.getMode() == U8G_MODE_R3G3B2 ) {
      u8g.setColorIndex(255);     // white
    }
    else if ( u8g.getMode() == U8G_MODE_GRAY2BIT ) {
      u8g.setColorIndex(3);         // max intensity
    }
    else if ( u8g.getMode() == U8G_MODE_BW ) {
      u8g.setColorIndex(1);         // pixel on
    }
    else if ( u8g.getMode() == U8G_MODE_HICOLOR ) {
      u8g.setHiColorByRGB(255,255,255);
    }*/


    // put your setup code here, to run once:
#if defined(_DEBUG_) || defined(_TEST_)
    Serial.begin(115200);
    //Serial.begin(9600);
    //delay(2000);
    _PL("Scheduler Template: setup()");
#endif


}


//void draw() {
// u8g.setFont(u8g_font_unifont);
//  u8g.setPrintPos(15, 25);
//  u8g.print(display);
//}


/**************************************************************************/
/*!
    @brief    Standard Arduino LOOP method - using with TaskScheduler there
              should be nothing here but ts.execute()
    @param    none
    @returns  none
*/
/**************************************************************************/
void loop() {
    ts.execute();
}


void onMic(){
    if(micPauseUntil >= millis()) return;
    int value = analogRead(MIC_PIN);
    if (value > 250){
        motorDirection  = motorDirection == 1 ? -1 : 1;
        //motor.setSpeed(motorDirection*motor.speed());
        motor.move(motorDirection*4096);
        micPauseUntil = millis() + micPauseDuration;
    }
}


void onMotion() {
    //tTimeout.restartDelayed();

    // don't check if motor is already running
    if(tMotor.isEnabled()) return;
    if(motionPauseUntil >= millis()) return;
    int value = digitalRead(MOTION_PIN);
    if (value == HIGH) {
        tMotor.enable();
        motionPauseUntil = motorStopTs + motionPauseDuration;
    }
}


void onMotor() {
    unsigned long now = millis();
    if(motorStopTs >= now){
        //if(motor.distanceToGo() > motor.targetPosition()/2){
        //  motor.move(motorDirection*4096);
        //}
        motor.move(motorDirection*4096);
        motor.run();
    }else{
        tMotor.disable();
        motor.stop();
        {}while(motor.run());  // wait
    }
}

bool onMotorEnable() {
    motor.setMaxSpeed(500);
    //motor.setSpeed(motorDirection*100);
    motor.setAcceleration(100);
    //motor.setCurrentPosition(0);
    motor.move(motorDirection*4096);
    motorStartTs = millis();
    motorStopTs = motorStartTs + motorDuration;
    return true;
}

void onMotorDisable() {
    //motor.runToNewPosition(0);

    //motor.setSpeed(0);
    //motor.moveTo(0);
    //motor.setCurrentPosition(0);
}



void onSonar(){
    digitalWrite(SONAR_TRIG_PIN, LOW);
    delayMicroseconds(2);

    digitalWrite(SONAR_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(SONAR_TRIG_PIN, LOW);

    int duration= pulseIn(SONAR_ECHO_PIN, HIGH);
    int cm =  duration * 0.034/2;
    int mm =  cm *10;

    if(cm>=0 && cm<=5){
        motor.setSpeed(0);
    }else if(cm>5 && cm<=10){
        motor.setSpeed(250);
    }else if(cm>10 && cm<=20){
        motor.setSpeed(500);
    }else if(cm>20){
        motor.setSpeed(1000);
    }
    else{
        digitalWrite(LED_PIN, LOW);
        //motor.setSpeed(0);
    }

    //if (value > 100){
    //  motor.setSpeed(1000);
    //}else{
    //  motor.setSpeed(0);
    //}
}




void cbTimeout() {
    _PM("taskTimeout()");
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    cli();
    sleep_enable();
    sleep_bod_disable();
    sei();
    //attachPinChangeInterrupt(MOTION_PIN, &onMotion, FALLING);
    //  power_all_disable();
    sleep_cpu();

    // z..z..z..Z..Z..Z

    /* wake up here */
    sleep_disable();
}