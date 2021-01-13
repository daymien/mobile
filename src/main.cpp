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

#define AUDIO_RX_PIN 5
#define AUDIO_WX_PIN 6
#define AUDIO_BUSY_PIN 12

#define SONAR_ECHO_PIN 6
#define SONAR_TRIG_PIN 5

#define FULL_STEP               8
#define STEPPER_PIN_1           8
#define STEPPER_PIN_2           9
#define STEPPER_PIN_3           10
#define STEPPER_PIN_4           11

// ==== Debug and Test options ==================
#define _DEBUG_
//#define _TEST_

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
#include <SoftwareSerial.h>
#include "DFRobotDFPlayerMini.h"
//#include <PinChangeInterrupt.h>
#include <U8glib.h>
#include "stdlib.h"
#include <avr/sleep.h>
#include <avr/power.h>



// ==== GLOBALS ===================================================================================


// display
//U8GLIB_SSD1306_128X32 u8g(U8G_I2C_OPT_NONE);


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

SoftwareSerial audioSerial(AUDIO_RX_PIN,AUDIO_WX_PIN);
DFRobotDFPlayerMini audio;
//U8GLIB_SSD1306_128X32 u8g(U8G_I2C_OPT_NONE); // Just for 0.91”(128*32)
//char display [32] = "Started";
//bool redraw = true;

unsigned int currentAudio = 0;
unsigned int fileCount = 0;
int *pAudioFiles;

// ==== Scheduler ==============================
Scheduler ts;

void cbTimeout();
void onMotion();

void onMic();
void onSonar();

void onMotor();
bool onMotorEnable();
void onMotorDisable();

void setupAudio();
void onAudio();
bool onAudioEnable();
void onAudioDisable();
bool audioIsPlaying();

//    void setupU8g();
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
Task tAudio(TASK_IMMEDIATE, TASK_FOREVER, &onAudio, &ts, false, &onAudioEnable, &onAudioDisable);


//void onTest();
//Task tTest(3*TASK_SECOND, TASK_FOREVER, &onTest, &ts, true);
//
//void onTest(){
//    _PL(millis());
//}


// ==== CODE ======================================================================================

/**************************************************************************/
/*!
    @brief    Standard Arduino SETUP method - initialize sketch
    @param    none
    @returns  none
*/
/**************************************************************************/
void setup() {
    pinMode(MOTION_PIN, INPUT);
    pinMode(MIC_PIN, INPUT);
    setupAudio();
    //setupU8g();

    // put your setup code here, to run once:
#if defined(_DEBUG_) || defined(_TEST_)
    Serial.begin(115200);
    _PL("Scheduler Template: setup()");
#endif

}

void setupAudio(){
    pinMode(AUDIO_BUSY_PIN, INPUT);
    audioSerial.begin(9600);
    if (!audio.begin(audioSerial)) {  //Use softwareSerial to communicate with mp3.
        while(true);
    }

    //audio.setTimeOut(500);
    audio.volume(20);
    audio.EQ(DFPLAYER_EQ_NORMAL);
    audio.outputDevice(DFPLAYER_DEVICE_SD);
}


//void setupU8g(void) {
//    // flip screen, if required
//    // u8g.setRot180();
//    // assign default color value
//    if (u8g.getMode() == U8G_MODE_R3G3B2) {
//        u8g.setColorIndex(255);     // white
//    } else if (u8g.getMode() == U8G_MODE_GRAY2BIT) {
//        u8g.setColorIndex(3);         // max intensity
//    } else if (u8g.getMode() == U8G_MODE_BW) {
//        u8g.setColorIndex(1);         // pixel on
//    } else if (u8g.getMode() == U8G_MODE_HICOLOR) {
//        u8g.setHiColorByRGB(255, 255, 255);
//    }
//    u8g.setFont(u8g_font_unifont);
//}
//void draw(void) {
//    u8g.drawStr(0, 10, display);
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

void onMotion() {
    if(tMotor.isEnabled() || motionPauseUntil >= millis()) return;
    int value = digitalRead(MOTION_PIN);
    if (value == HIGH) {
        tMotor.enable();
        motionPauseUntil = motorStopTs + motionPauseDuration;
    }
}

void onMotor() {
    if(motorStopTs >= millis()){
        motor.move(motorDirection * 2048);
        motor.run();
    }else{
        tMotor.disable();
    }
}

bool onMotorEnable() {
    motor.setMaxSpeed(400);
    motor.setAcceleration(80);
    motorStartTs = millis();
    motorStopTs = motorStartTs + motorDuration;

    // pause mic for a second if not paused
    if(micPauseUntil <= (millis() + 1000)) {
        micPauseUntil = millis() + 1000;
    }
    return true;
}

void onMotorDisable() {
    motor.stop();
    {}while(motor.run());  // wait

    // pause mic for a second if not paused
    if(micPauseUntil <= (millis() + 1000)) {
        micPauseUntil = millis() + 1000;
    }
}

void onMic(){
    if(micPauseUntil >= millis() || audioIsPlaying()) return;
    int value = analogRead(MIC_PIN);
    if (motor.isRunning() ? value > 1000 : value > 120) {
        micPauseUntil = millis() + 5000;
        tAudio.enable();
    }
}


bool onAudioEnable(){
    if (pAudioFiles == NULL) {
        fileCount = audio.readFileCounts();
        if(fileCount <= 0){
            exit(1);
        }
        pAudioFiles =  (int *)malloc(sizeof(int)*fileCount);
        if(pAudioFiles == NULL) {
            printf("malloc of size %d failed!\n", fileCount);   // could also call perror here
            exit(1);   // or return an error to caller
        }

        // fill audio array
        for(int i=0; i < fileCount +1; i++) {
            pAudioFiles[i] = i+1;
            //_PM(pAudioFiles[i]);
        }

        // shuffle audio array
        srand(millis() + motor.distanceToGo());
        for (int i = 0; i < fileCount; i++)
        {
            size_t j = i + rand() / (RAND_MAX / (fileCount - i) + 1);
            int t = pAudioFiles[j];
            pAudioFiles[j] = pAudioFiles[i];
            pAudioFiles[i] = t;
            _PM(pAudioFiles[i]);
        }
    }
    _PM(currentAudio++ % fileCount);
    audio.playMp3Folder(pAudioFiles[currentAudio]);
    return true;
}

void onAudio(){
    if ((micPauseUntil - 1000) >= millis() || audioIsPlaying()) return;
    tAudio.disable();
}

void onAudioDisable(){
    audio.stop();
}

bool audioIsPlaying(){
    return digitalRead(AUDIO_BUSY_PIN) == LOW;
}

void onSonar(){
    digitalWrite(SONAR_TRIG_PIN, LOW);
    delayMicroseconds(2);

    digitalWrite(SONAR_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(SONAR_TRIG_PIN, LOW);

    int duration= pulseIn(SONAR_ECHO_PIN, HIGH);
    int cm =  duration * 0.034/2;
    //int mm =  cm *10;

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
    }
}




void cbTimeout() {
    //_PM("taskTimeout()");
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    cli();
    sleep_enable();
    //sleep_bod_disable();
    sei();
    //attachPinChangeInterrupt(MOTION_PIN, &onMotion, FALLING);
    power_all_disable();
    sleep_cpu();
//
    //// z..z..z..Z..Z..Z
//
    ///* wake up here */
    sleep_disable();
}