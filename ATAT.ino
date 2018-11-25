/* 
 *  AT-AT Gun'n'Walk V.18
 * 
 * Arduino Sketch for controlling lights and sounds in Hoth Diorama
 * by Ole Andre aka @oleshobbyblog www.oleandre.net
 * 
 * I will try to list all my sources, but I will probably miss one or two. 
 * 
 * Sources:
 * - Generel examples found within the Arduino IDE
 * - Fading: http://forum.arduino.cc/index.php?topic=12004.0
 * - No delay: https://www.baldengineer.com/fading-led-analogwrite-millis-example.html
 * - Odd/Even: http://forum.arduino.cc/index.php?topic=41397.0
 * - IR Sensor: https://github.com/adafruit/Adafruit-NEC-remote-control-library
 * 
 * Datasheets: 
 * - Mono amplifier: https://cdn-shop.adafruit.com/datasheets/PAM8302A.pdf
 * - Stereo amplifier: https://cdn-shop.adafruit.com/datasheets/TPA2016D2.pdf
 * - WAV Trigger: https://cdn.sparkfun.com/datasheets/Widgets/STM32F405RGT6.pdf
 * - Pro Trinket 5V: https://cdn-shop.adafruit.com/datasheets/ATMEGA328P.pdf
 * 
 * Guides:
 * - Pro Trinket 5V: https://cdn-learn.adafruit.com/downloads/pdf/introducing-pro-trinket.pdf
 * - Stereo Amplifier: https://learn.adafruit.com/adafruit-tpa2016-2-8w-agc-stereo-audio-amplifier
 * - WAV Trigger Guide#1: https://learn.sparkfun.com/tutorials/wav-trigger-hookup-guide
 * - WAV Trigger Guide#2: http://robertsonics.com/wav-trigger-online-user-guide/
 * - IR Sensor: https://learn.adafruit.com/ir-sensor/overview
 *              http://z3t0.github.io/Arduino-IRremote/  <-- THIS!
 * 
 * Acknowledgements:
 * - IR sensor code: Ken Shirriff (http://arcfn.com & http://z3t0.github.io/Arduino-IRremote/), TRULY a lifesaver!
 *                    (I had to swap to timer 1 in boarddefs.h due to a conflict with the SoftPWM library)
 * - User sterretje of the Arduino forums: https://forum.arduino.cc/index.php?topic=454873.0 
 *
 * Parts list:
 *  1x IR Sensor: http://www.adafruit.com/products/157
 *  1x Remote control: http://www.adafruit.com/products/389
 *  1x WAV Trigger: https://www.sparkfun.com/products/13660
 *  1x Pro Trinket 5V: https://www.adafruit.com/product/2000
 *  1x Stereo amplifier: https://www.adafruit.com/product/1712
 *  2x Speakers: Bought at a store in Japan, but seems similar to this https://www.sparkfun.com/products/9151
 *  1x 128x32 i2c OLED: https://www.adafruit.com/product/931
 *  LEDs: Various sizes, but all 5V from here: https://www.modeltrainsoftware.com/ and here (the wide, bright used in the explosion): https://www.ledsales.com.au/
 *  Resistors: Just about any make..not sure where I got mine
 *  Wiring: Regular size: https://www.adafruit.com/product/1311
 *          Magnet "size" found here: https://www.modeltrainsoftware.com/ 
 *  Several terminal blocks of various sizes. Found at Adafruit. Important note: Use 0.1" pitch (aka. 2.54mm) so they will fit on breadboard friendly cards. 
 */
// Include some libraries we need :) 
#include <SoftPWM.h>          // Software PWM due to too few analog write pins
#include <wavTrigger.h>       // For controlling the WAV Trigger via serial
#include <Wire.h>             // Used for I2C communications
#include <Adafruit_SSD1306.h> // The OLED library
#include "Adafruit_TPA2016.h" // Used for the I2C controlled stereo amplifier
#include <IRremote.h>         // Used for the IR sensor function

// Pins
#define snowspeederCannon0Pin 8   // SoftPWM, speeder 1 cannon 1
#define snowspeederCannon1Pin 5   // SoftPWM, speeder 1 cannon 2
#define snowspeederCannon2Pin A2  // SoftPWM, speeder 2 cannon 1
#define snowspeederCannon3Pin A3  // SoftPWM, speeder 2 cannon 2
#define atatCockpitPin 6          // PWM, light in AT AT cockpit
#define hlc0Pin 11                // Soft PWM if necessary, left HLC
#define hlc1Pin 3                 // Soft PWM if necessary, right HLC
#define explosionPin 4            // Soft PWM, explosion on ground
#define rxPin 12                  // IR Sensor pin
#define SCREEN_WIDTH 128          // OLED display width, in pixels
#define SCREEN_HEIGHT 32          // OLED display height, in pixels
#define oledResetPin 13           // OLED requires a reset pin

// Variables
String hexRes = "000000";         // A hex state that will trigger nothing

bool voiceNow = false;            // State machining, this signals the voice is ON
bool scroll = false;              // True = we are scrolling
bool blinker = false;             // Used to blink the pause screen
bool asYouWere = false;           // Back to business.. 
bool volumeAdjustedStep1 = false; // State just after volume has been adjusted, in two steps
bool volumeAdjustedStep2 = false; // State just after volume has been adjusted, in two steps
bool paused = false;              // Pause if true, unpause if false
bool started = false;             // True = started, False = "First run, do this once"   
bool explosionState = false;      // Explosion going on = true
bool voiceStarted = false;        // Voice started = true, else = false
bool soloVoice = false;           // Used to play just one voice, if playing = true
bool atatDoneState = false;       // Used to manage states = true if in ATAT sequence..
bool rndmAtatShot = false;        // Used to control when new random variables are made
bool rndmExplosion = false;       // Used to control when new random variables are made
bool previousPaused = false;      // Used to get us back to normal after a pause..by always storing paused state from last loop
bool stoppingVoice = false;       // Start timer for stopping a voice in 300 ms.. 
bool atatState = false;           // State machine, this is TRUE while the ATAT fires and there is or isn't an explosion
bool explosionDone = false;       // State machine, it is ok to move on. Explosion done. 
bool snowspeederState = false;    // State machine, this is TRUE while the snowspeeders flies past and fires. 
bool speeder1 = false;            // Incase there are two speeders, this makes sure they are separated in time
bool speeder2 = false;            // Incase there are two speeders, this makes sure they are separated in time
bool rndmSnowspeeder = false;     // State machine to control the generation of random values for the snowspeeders
bool snowspeederDoneState = false;// State machine for snowspeeder completion..
bool explosionStopping = false;   // Signals the explosion is about to end :) 
bool still = false;               // Used to change the paused state to a still state where the pause sign should not blink
bool backOffTimerSpeeder = false; // Attempt at making room for the voices to break through between the speeders and atats..
bool backOffTimerAtat = false;    // Attempt at making room for the voices to break through between the speeders and atats..

unsigned long startScrollMillis;          // Used for the scrolling that starts automatically after boot          
unsigned long wasPausedTimeMillis;        // We want scrolling to resume after 
unsigned long adjustedVolumeTimeMillis;   // Also after adjusting the volume "stuff" should happen
//unsigned long lastElementMillis;        // Time since last element..
unsigned long previousaAtatMillis;        // Timer for the ATAT function
unsigned long previousBlinkMillis;        // Used to blink the PAUSE screen with even intervals
unsigned long rnd;                        // To randomize the atat or snowspeeder to decide who starts first, long tho?
unsigned long previousVoiceMillis;        // Used to time the voice so there is a slight delay before and after..
unsigned long previousPauseMillis;        // Used to control the return from being paused
unsigned long atatDelayMillis;            // Time to wait for ATAT..
unsigned long lastVoiceMillis;            // So we know how long it has been since the last voice..we want some separation so they're not going off at the same time
unsigned long lastAtatElementMillis;      // Some internal timing, and also a way to add spacing between atat sequences
unsigned long previousSpeederMillis;      // Some internal timing, and also a way to add spacing between speeder sequences
unsigned long rndmSpeederMillis;          // Possibly redundant, adds some more time before the speeder starts..randomized.
unsigned long rndmSpeederSpacerMillis;    // Adds some random time between speeders in a flight
unsigned long previousSpeederShotMillis1; // Used to time the speeder shot so they happen with some spacing ("pew..pew..pew" as oppsed to "pewpewpew")
unsigned long previousSpeederShotMillis2; // Used to time the speeder shot so they happen with some spacing ("pew..pew..pew" as oppsed to "pewpewpew")

byte atatShot;            // Which ATAT shot are we at
byte numAtatShots;        // The shots we should fire this round
byte firedAtat;           // Makes us alternate turret
byte rndmExplosionMillis; // Used to control the time from ATAT shot to explosion (random)
byte explodeOrNot;        // Used to decide if there will be an explosion
byte newVolume = 0;       // For mapping to "nicer" numbers
byte speederSound1;       // Put the speeder sound track number into a variable so we can use the track number again later
byte speederSound2;       // Put the speeder sound track number into a variable so we can use the track number again later
byte oneOrTwo;            // Two or one snowspeeder will fly past?
byte shootOrNot1;         // Do the snowspeeder fire or not?
byte shootOrNot2;         // Do the snowspeeder fire or not?
byte numberOfShots1;      // How many speeder laser shots
byte numberOfShots2;      // How many speeder laser shots
byte speederShot1 = 0;    // Count what shot we are at  
byte speederShot2 = 0;    // Count what shot we are at       
byte firedSs1 = 0;        // Used to ensure we enter the Snowspeeder firing sequence correctly
byte firedSs2 = 0;        // Used to ensure we enter the Snowspeeder firing sequence correctly

int explosion;            // Holds the explosion track number
int voice = 14;           // Starting voice
int volumeGain = -10;     // This is the default setting of dB from which we always start
int volumeGain_old = 0;   // Used to store old volume setting so it can be returned to after un-pausing   

static char HLCBright = 200;          // Initially 120, too low
static char ATATCockpitLED = 50;   // Initially 15, way too low
static char maxVolume = -5;           // Above this volume and we get crackling in the speakers..
static char minVolume = -70;          // Below this and it gets silly, the amp goes to -70..why would we need to go lower
static char volumeTrack_1 = -3;       // Background track 1 set to a gain of -3. Used for fading in and out during voices
static char volumeTrack_999 = -6;     // Background track 999 set to a gain of -3. Used for fading in and out during voices
char volumeChangeTrack_1;             // Used for fading in and out during voices
char volumeChangeTrack_999;           // Used for fading in and out during voices

// Empire Emblem
#define First_Galactic_Empire_emblem_width 32
#define First_Galactic_Empire_emblem_height 31
static const unsigned char PROGMEM First_Galactic_Empire_emblem_bits[] = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0xFC, 0x00, 0x00, 0xF9, 0x9F, 0x00, 0x01, 0xC7, 0xE3, 0x80,
0x03, 0x07, 0xE0, 0xC0, 0x06, 0x1F, 0xF8, 0x60, 0x0C, 0x7C, 0x3E, 0x30, 0x1B, 0xFC, 0x3F, 0xD8,
0x3F, 0xFC, 0x3F, 0xFC, 0x3F, 0xBC, 0x3D, 0xFC, 0x2F, 0x1E, 0x78, 0xF4, 0x6E, 0x0C, 0x30, 0x76,
0x66, 0x00, 0x00, 0x66, 0x67, 0x00, 0x01, 0xE6, 0x47, 0xE0, 0x07, 0xE2, 0x47, 0xE0, 0x07, 0xE2,
0x47, 0xE0, 0x07, 0xE2, 0x67, 0x00, 0x00, 0xE2, 0x66, 0x00, 0x00, 0x66, 0x6E, 0x0C, 0x30, 0x76,
0x2F, 0x1E, 0x78, 0xF4, 0x3F, 0xBC, 0x3D, 0xFC, 0x3F, 0xFC, 0x3F, 0xFC, 0x1B, 0xFC, 0x3F, 0xD8,
0x0C, 0x7C, 0x3E, 0x30, 0x06, 0x1F, 0xF8, 0x60, 0x03, 0x07, 0xE0, 0xC0, 0x01, 0xC7, 0xE3, 0x80,
0x00, 0xF9, 0x9F, 0x00, 0x00, 0x1F, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00,
};

//Rebel Logo
#define Rebel_Alliance_logo_small_width 32
#define Rebel_Alliance_logo_small_height 32
static const unsigned char PROGMEM Rebel_Alliance_logo_small [] = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x03, 0xC0, 0x00, 0x01, 0x07, 0xE0, 0x80,
0x02, 0x03, 0xC0, 0x40, 0x04, 0x1B, 0xD8, 0x20, 0x0C, 0x3D, 0xFC, 0x30, 0x1C, 0x1F, 0xF8, 0x18,
0x18, 0x0F, 0xF0, 0x18, 0x38, 0x07, 0xE0, 0x1C, 0x38, 0x07, 0xE0, 0x1C, 0x38, 0x03, 0xC0, 0x1E,
0x78, 0x03, 0xC0, 0x1E, 0x78, 0x03, 0xC0, 0x1E, 0x78, 0x03, 0xC0, 0x1E, 0x7C, 0x03, 0xC0, 0x3E,
0x7E, 0x07, 0xE0, 0x7E, 0x7F, 0x07, 0xE0, 0xFE, 0x7F, 0x8F, 0xF9, 0xFE, 0x7F, 0xFF, 0xFF, 0xFE,
0x7F, 0xFF, 0xFF, 0xFE, 0x3F, 0xFF, 0xFF, 0xFC, 0x3F, 0xFF, 0xFF, 0xFC, 0x1F, 0xFF, 0xFF, 0xFC,
0x1F, 0xFF, 0xFF, 0xF8, 0x0F, 0xFF, 0xFF, 0xF0, 0x07, 0xFF, 0xFF, 0xF0, 0x03, 0xFF, 0xFF, 0xE0,
0x01, 0xFF, 0xFF, 0xC0, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x1F, 0xFC, 0x00, 0x00, 0x01, 0xC0, 0x00,
};

// Star Wars Logo (unused v.18)
#define Star_Wars_Logo_width 73
#define Star_Wars_Logo_height 32
static const unsigned char PROGMEM Star_Wars_Logo [] = {
0x00, 0x00, 0x00, 0x00, 0x22, 0x04, 0x08, 0x00, 0x00, 0x00, 0x1F, 0xFF, 0xFF, 0xE3, 0xFC, 0x0F,
0xFF, 0x80, 0x00, 0x3F, 0xFF, 0xFF, 0xE7, 0xFC, 0x0F, 0xFF, 0xC0, 0x00, 0x3F, 0xFF, 0xFF, 0xE7,
0xFE, 0x0F, 0xFF, 0xE0, 0x00, 0x3F, 0xFF, 0xFF, 0xE7, 0xFE, 0x0F, 0x83, 0xE0, 0x00, 0x3F, 0x00,
0xF0, 0x0F, 0x9E, 0x0F, 0x83, 0xE0, 0x00, 0x3F, 0x80, 0xF0, 0x0F, 0x9F, 0x0F, 0x83, 0xE0, 0x00,
0x1F, 0xC0, 0xF0, 0x0F, 0x9F, 0x0F, 0xFF, 0xC0, 0x00, 0x0F, 0xC0, 0xF0, 0x1F, 0x0F, 0x0F, 0xFF,
0x80, 0x00, 0x07, 0xE0, 0xF0, 0x1F, 0xFF, 0x8F, 0xFF, 0x00, 0xFF, 0xFF, 0xE0, 0xF0, 0x1F, 0xFF,
0x8F, 0xFF, 0xFF, 0x7F, 0xFF, 0xE0, 0xF0, 0x1F, 0xFF, 0x8F, 0x9F, 0xFF, 0x7F, 0xFF, 0xE0, 0xF0,
0x3F, 0xFF, 0x8F, 0x8F, 0xFF, 0x7F, 0xFF, 0xC0, 0xF0, 0x3E, 0x07, 0xCF, 0x87, 0xFF, 0xFF, 0xFF,
0x80, 0xF0, 0x3E, 0x07, 0xEF, 0x83, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7C, 0x7E, 0x7C, 0x3F, 0xC0, 0xFF, 0xF8,
0x07, 0xFF, 0x7C, 0xFE, 0x7C, 0x7F, 0xC0, 0xFF, 0xFC, 0x0F, 0xFF, 0x7E, 0xFE, 0x7C, 0x7F, 0xE0,
0xFF, 0xFE, 0x1F, 0xFF, 0x3E, 0xFF, 0x7C, 0x7F, 0xE0, 0xFC, 0x7E, 0x1F, 0xFF, 0x3F, 0xFF, 0xF8,
0x79, 0xE0, 0xFC, 0x1E, 0x1F, 0x80, 0x3F, 0xFF, 0xF8, 0xF9, 0xF0, 0xFC, 0x1E, 0x1F, 0x80, 0x1F,
0xFF, 0xF8, 0xF9, 0xF0, 0xFF, 0xFE, 0x0F, 0xC0, 0x1F, 0xFF, 0xF0, 0xF9, 0xF0, 0xFF, 0xFC, 0x0F,
0xE0, 0x1F, 0xFF, 0xF1, 0xF0, 0xF0, 0xFF, 0xF8, 0x07, 0xF0, 0x1F, 0xEF, 0xF1, 0xFF, 0xF8, 0xFF,
0xF8, 0x03, 0xF0, 0x0F, 0xEF, 0xE1, 0xFF, 0xF8, 0xFD, 0xFF, 0xFF, 0xF0, 0x0F, 0xC7, 0xE3, 0xFF,
0xF8, 0xFD, 0xFF, 0xFF, 0xF0, 0x0F, 0xC7, 0xE3, 0xE0, 0x7C, 0xFC, 0xFF, 0xFF, 0xE0, 0x07, 0xC7,
0xE3, 0xE0, 0x7C, 0xFC, 0x7F, 0xFF, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x84, 0x00, 0x00, 0x00,
0x63, 0x65, 0x5F, 0x6C, 
};

// The WAV Trigger object
wavTrigger wTrig;

// The Stereo Amplifier object
Adafruit_TPA2016 audioamp = Adafruit_TPA2016();

// Initialize the OLED display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, oledResetPin);

// IR Sensor object
IRrecv irrecv(rxPin);  // Receiving IR sensor signals on this pin
decode_results results;   // Returns the decoded results (decode_type, addres, value, bits..etc.. Referenced using i.e. "results.value"

void setup() {

  // Make the numbers a bit more random between each power cycle. 
  randomSeed(A0);       // Seed random number generator

  // SOFTPWM SETUP & RUN ONCE >> This allows "regular" pins to act as PWM pins. Very useful!
  SoftPWMBegin();       
  // Define what pins are to be SoftPWM pins, and what their initial value will be
  SoftPWMSet(hlc0Pin, 0);  
  SoftPWMSet(hlc1Pin, 0);
  SoftPWMSet(snowspeederCannon0Pin, 0);
  SoftPWMSet(snowspeederCannon1Pin, 0);
  SoftPWMSet(snowspeederCannon2Pin, 0);
  SoftPWMSet(snowspeederCannon3Pin, 0);
  SoftPWMSet(explosionPin, 0);
  // Define the speed with which the LEDs will fade in, and fade out, respectively. 
  SoftPWMSetFadeTime(hlc0Pin, 10, 50);
  SoftPWMSetFadeTime(hlc1Pin, 10, 50);
  SoftPWMSetFadeTime(snowspeederCannon0Pin, 10, 50);
  SoftPWMSetFadeTime(snowspeederCannon1Pin, 10, 50);
  SoftPWMSetFadeTime(snowspeederCannon2Pin, 10, 10);
  SoftPWMSetFadeTime(snowspeederCannon3Pin, 10, 10);
  SoftPWMSetFadeTime(explosionPin, 10, 1000);  
  
  //OLED SETUP & RUN ONCE
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C for the 128x32
  description();
  
  // IR SENSOR SETUP & RUN ONCE
  irrecv.enableIRIn(); // Start the receiver

  // Light up the interior cockpit light of the AT-AT
  analogWrite(atatCockpitPin, ATATCockpitLED);  // Light the ATAT cockpit
  
  // WAV TRIGGER SETUP & RUN ONCE 
  wTrig.start();
  delay(10);

  // Send a stop-all command and reset the sample-rate offset, in case we have
  //  reset while the WAV Trigger was already playing.
  wTrig.stopAllTracks();
  wTrig.samplerateOffset(0);
  wTrig.setReporting(true);       // Must be enabled to allow feedback about tracks playing from WAV Trigger  
  

  // Allow time for the WAV Trigger to respond with the version string and
  //  number of tracks.
  delay(100); 
     
  wTrig.masterGain(volumeGain);   // Sets the master gain t
  wTrig.trackPlayPoly(1);         // Start Track 1, AT AT walking. Poly allows more tracks at the same time
  wTrig.trackLoop(1, 1);          // Loop Track 1
  wTrig.trackPlayPoly(999);       // Start Track 999, background music. Also poly. We want more tracks.
  wTrig.trackGain(999, -6);       // Set the track gain for track 999. This is background music
  wTrig.trackLoop(999, 1);        // Loop track 999 as well.. 

  // AMPLIFIER SETUP & RUN ONCE
  audioamp.begin();                             // Initializing the amplifier
  audioamp.enableChannel(true, true);           // Turn on the amplifier, both channels (I2C)
  audioamp.setGain(0);                          // Set the initial gain to 0. 
  audioamp.setAGCCompression(TPA2016_AGC_OFF);  // For testing, currently the AGC is off and seems to be what I wan
}

void loop() {
  wTrig.update();

  // IR RECEVIER
  if (irrecv.decode(&results)) {          // Get the results of the decoding..
    hexRes = String(results.value, HEX);  // Make it a hex value
    irrecv.resume();                      // Resume receiving..
  }

  // Volume UP
  if (hexRes == "fd40bf"){                                // This is hex for volume up
    volumeGain++;                                         // Add to the existing volueGain variable
    if (volumeGain >= maxVolume) volumeGain = maxVolume;  // Do not exceed the max volume
    wTrig.masterGain(volumeGain);                         // Set the master gain to the new value  
    hexRes = "000000";                            // In preparation for the next IR code
    display.stopscroll();                         // Stop any scrolling of the screen
    display.clearDisplay();                       // Clear the buffer 
    display.setTextSize(4);                       // Set the text size
    display.setTextColor(WHITE);                  // ..and color
    display.setCursor(40,5);                      // ..and location
    newVolume = map(volumeGain, -70, -5, 0, 65);  // Looks better to drop the negative number etc..
    display.println(newVolume);                   // ..enter text
    display.display();                            // Display!
    delay(10);                                    // Such a short delay that it is "ok". Just a "debounce"  
    adjustedVolumeTimeMillis = millis();          // Take the time when the last volume adjustment was made
    volumeAdjustedStep1 = true;                   // We are in this state now..
    volumeAdjustedStep2 = false;                  // We are waiting for this state..  
  }
  
  // Volume DOWN
  if (hexRes == "fd00ff"){                                // Hex for volume down.. 
    volumeGain--;                                         // Same a for volume up
    if (volumeGain <= minVolume) volumeGain = minVolume;  // Do not exceed the min volume
    wTrig.masterGain(volumeGain);                         // Adjust volume..      
    hexRes = "000000";                            // In preparation for the next IR code
    display.stopscroll();                         // Stop any scrolling of the screen
    display.clearDisplay();                       // Clear the buffer 
    display.setTextSize(4);                       // Set the text size
    display.setTextColor(WHITE);                  // ..and color
    display.setCursor(40,5);                      // ..and location
    newVolume = map(volumeGain, -70, -5, 0, 65);  // Looks better to drop the negative number etc..
    display.println(newVolume);                   // ..enter text
    display.display();                            // Display!
    delay(10);                                    // Such a short delay that it is "ok". Just a "debounce"  
    adjustedVolumeTimeMillis = millis();          // Take the time when the last volume adjustment was made
    volumeAdjustedStep1 = true;                   // We are in this state now..
    volumeAdjustedStep2 = false;                  // We are waiting for this state..      
  }

  // PAUSE / UNPAUSE
  if (hexRes == "fd807f"){                        // Hex for Play / Pause
    if (paused == false){                         // We were unpaused, but the button was pressed..start pausing!
      display.stopscroll();                       // Stop any scrolling of the screen
      delay(1);
      display.clearDisplay();                     // Clear the buffer
      display.display();                          // Display! 
      delay(1);                                   // Delay suggested in OLED library example code                               
      volumeGain_old = volumeGain;                // Save the old volume setting
      wTrig.trackPause(voice);                    // If voice is talking, put track on pause
      for (int i=volumeGain; i >= -70; i--){      // -70 is total silence..
        wTrig.masterGain(i);                      // Set the master gain
        delay(10);                                // Such a short delay that it is "ok". Just a "debounce"
      } 
      audioamp.enableChannel(false, false);       // Turn off both channels using I2C
      paused = true;                              // We are now paused..
      analogWrite(atatCockpitPin, 0);             // Turn off the ATAT Cockpit
      display.clearDisplay();                     // Clear the buffer
      display.setTextSize(3);                     // Set the text size
      display.setTextColor(WHITE);                // ..and color
      display.setCursor(12,5);                    // ..and location
      display.println("PAUSED");                  // ..enter text
      display.display();                          // Display!
      delay(1);
      previousBlinkMillis = millis();
    }else if (paused == true){                    // We were paused, but the button was pressed..start unapusing, get the show on the road!
      audioamp.enableChannel(true, true);         // Turn on both channels using I2C      
      for (int i=-70; i <= volumeGain_old; i++){  // Increase the volume back to the old setting  
        wTrig.masterGain(i);                      // Set the master gain
        delay(10);                                // Such a short delay that it is "ok". Just a "debounce"
      } 
      paused = false;                             // We are now unpaused.. 
      analogWrite(atatCockpitPin, ATATCockpitLED);// Turn the atat cockpit back on
      wTrig.trackResume(voice);                   // Unpause and resume the voice

    } 
    hexRes = "000000";                            // Reset the remote control hexres variable after button has been released.
  }

  // Static (still) Display
  if (hexRes == "fd58a7"){                        // Hex for 9 aka Static Display, Lights On!
    if (paused == false){                         // We were "unpaused", but the button was pressed..start pausing
      lightsOut(false);                           // Turn all lights on!       
      volumeGain_old = volumeGain;                // Save the old volume setting
      wTrig.trackPause(voice);                    // If voice is talking, put track on pause
      for (int i=volumeGain; i >= -70; i--){      // -70 is total silence..
        wTrig.masterGain(i);                      // Set the master gain
        delay(10);                                // Such a short delay that it is "ok". Just a "debounce"
      } 
      audioamp.enableChannel(false, false);       // Turn off both channels using I2C
      paused = true;                              // We are now "paused.."
      still = true;                               // Static display
    }else if (paused == true){                    // We were "paused", but the button was pressed..start "unapusing", get the show on the road!
      lightsOut(true);                            // Lights off..let the functions take back control
      audioamp.enableChannel(true, true);         // Turn on both channels using I2C      
      for (int i=-70; i <= volumeGain_old; i++){  // Increase the volume back to the old setting  
        wTrig.masterGain(i);                      // Set the master gain
        delay(10);                                // Such a short delay that it is "ok". Just a "debounce"
      } 
      paused = false;                             // We are now "unpaused.." 
      still = false;                              // Static display off
      wTrig.trackResume(voice);
    } 
    hexRes = "000000";                            // Reset the remote control hexres variable after button has been released.     
  }

  // Functions to run every loop, paused or not
  timedBootCmds();  // Stuff that should happen once and a given time after reset
  randoms();        // Generate new random variables every run if asked to
  volumeAdjusted(); // What to do if volume was adjsted
  pause();          // What to do if paused, or just comoing out of pause..

  // If we are not paused, do this
  if(paused == false){ // Check if we are paused..we are not
    if (voiceNow == false){ // Check if the voice function has been triggered
      if ((millis() - lastAtatElementMillis >= 500) && backOffTimerAtat == false){  // If it has been 500 or more ms since last atat sequence, and we have not triggered yet..
        lastAtatElementMillis = millis();   // Record time so we can do this at even intervals
        backOffTimerAtat = true;            // ATAT has triggered, backoff
      }
      if ((millis() - previousSpeederMillis >= 500) && backOffTimerSpeeder == false){
        lastAtatElementMillis = millis();
        backOffTimerSpeeder = true;
      }
    if (backOffTimerAtat == true) atat();           // If we have gone through the backoff timer, execute
    if (backOffTimerSpeeder == true) snowspeeder(); // If we have gone through the backoff timer, execute
    }
    // If atat and snowspeeders are not running, and it has been at least 8 seconds since last voice..
    if (atatState == false && snowspeederState == false && (millis() - lastVoiceMillis >= 8000)) voiceTrigger();  
  // If we are paused, do this
  } else if (paused == true){
    // Best do nothing..
  }
  previousPaused = paused; //Set the previous paused state
} 

///////////// END LOOP ///////////// 


static void description(){
  display.clearDisplay();                     // Clear the buffer 
  display.drawBitmap(2, 0, First_Galactic_Empire_emblem_bits, 32, 31, 1); // miniature bitmap display
  display.drawBitmap(94, 0, Rebel_Alliance_logo_small, 32, 32, 1);        // miniature bitmap display
  display.setTextSize(2);                     // Set the text size
  display.setTextColor(WHITE);                // ..and color
  display.setCursor(41,2);                    // ..and location
  display.println("HOTH");                    // ..enter text
  display.setTextSize(1);                     // Set the text size
  display.setCursor(42,23);                   // ..new line
  display.println("Ver: 1.0");                // ..enter text
  display.display();                          // Display!
  delay(1);
}

static void lightsOut(bool onOff){          // Make everything dark for the pause
  if (onOff == true){              
    SoftPWMSet(hlc0Pin, 0);  
    SoftPWMSet(hlc1Pin, 0);
    SoftPWMSet(snowspeederCannon0Pin, 0);
    SoftPWMSet(snowspeederCannon1Pin, 0);
    SoftPWMSet(snowspeederCannon2Pin, 0);
    SoftPWMSet(snowspeederCannon3Pin, 0);
    SoftPWMSet(explosionPin, 0);
  }else if (onOff == false){            
    SoftPWMSet(hlc0Pin, HLCBright);  
    SoftPWMSet(hlc1Pin, HLCBright);
    SoftPWMSet(snowspeederCannon0Pin, 120);
    SoftPWMSet(snowspeederCannon1Pin, 120);
    SoftPWMSet(snowspeederCannon2Pin, 120);
    SoftPWMSet(snowspeederCannon3Pin, 120);
    SoftPWMSet(explosionPin, 200);    
  }
}

void voiceTrigger(){
  if (voiceStarted == false && voiceNow == false){                            // No voice at the moment, and we're just starting one..need to do this just once for each voice triggered
    previousVoiceMillis = millis();                                           // Get the start time of the current voice trigger
    voiceNow = true;
    quiet(true);                                                              // Quiet the background noises
  }
 if ((millis() - previousVoiceMillis >= 300) && voiceStarted == false){       // Wait 300ms since the start, and by extension when the quite function was called.   
    voiceStarted = true;
    wTrig.trackPlayPoly(voice);                                               // Play selected voice                    
    wTrig.trackGain(voice, 3);                                                // Set the track gain for voice. 
    delay(20);                                                                // VERY IMPORTANT! Allow the wav trigger to actually start the file, before we check if it is running futher down
  }
  if (voiceStarted == true){                                                  // Voice started, let's do some checking to see if it is done.. 
    if(wTrig.isTrackPlaying(voice) == true) {                                 // This polls the wav trigger to see if voice track is playing, again, important to wait 20 ms to allow file to start..
    }
    if (wTrig.isTrackPlaying(voice) == false && stoppingVoice == false) {     // If the track has stopped, let's do this stuff.. 
      previousVoiceMillis = millis();                                         // Collec the current time, used to increase background sound 300ms after the voice ends
      stoppingVoice = true;                                                   // We'll start stopping the voice now ;) 
    }
    if ((millis() - previousVoiceMillis >= 300) && (stoppingVoice == true)){  // Wait 300 ms before stopping the voice, also do not do this until stopping has begun..
      quiet(false);                                                           // Increase background sound again
      voice++;                                                                // The next voice is..
      stoppingVoice = false;                                                  // Resetting the state booleans..
      voiceNow = false;                                                       // Resetting the state booleans..
      voiceStarted = false;                                                   // Resetting the state booleans..
      lastVoiceMillis = millis();                                             // Get the time the voice was all done (300ms + voice ms + 300 ms)
      if (voice == 32) voice = 14;                                            // Reset voice sequence              
    }
  }
}  


void atat(){
  // ATAT Firing Sequence
  atatState = true;
  if(atatDoneState == false){ 
    if (millis() - lastAtatElementMillis >= atatDelayMillis){                                                                     
      while(atatShot < numAtatShots){
        if ((millis() - previousaAtatMillis >= 150) && (firedAtat == 0)) {
          previousaAtatMillis = millis();
          wTrig.trackPlayPoly(4);
          delay(20);
          if ((atatShot % 2) == 0) SoftPWMSet(hlc0Pin, HLCBright); // Even
          if (atatShot % 2) SoftPWMSet(hlc1Pin, HLCBright); // Odd
          firedAtat = 1;    
        }
        if ((millis() - previousaAtatMillis >= 150) && (firedAtat == 1)) {
          previousaAtatMillis = millis();
          if ((atatShot % 2) == 0) SoftPWMSet(hlc0Pin, 0);   
          if (atatShot % 2) SoftPWMSet(hlc1Pin, 0); 
          atatShot++;
          firedAtat = 0;
        }
      }
      if (atatShot >= numAtatShots && atatDoneState == false){
        atatDoneState = true;
        lastAtatElementMillis = millis();
      }
    }
  }

  // Explosion Sequence
  if(atatDoneState == true && explosionState == false){
    if (millis() - lastAtatElementMillis >= rndmExplosionMillis) {  // Basically means we wait rndmATATExplosionMillis for this bit. 
      if (explodeOrNot > 4){                        // A way to weight the randomness..if the value of exlodeOrNot is greater than 4 we get an explosion! 
        explosionState = true;                                                
        wTrig.trackPlayPoly(explosion);             // Explosions = Files 9 - 13
        SoftPWMSet(explosionPin, 200);              // Light the explosion LED to a value of 200
        delay(20);
        explosionStopping = true;
        lastAtatElementMillis = millis();
      }else if(explodeOrNot <= 4){
        explosionState = true;
        explosionDone = true;
      }
    }
  }

 if ((explosionStopping == true) && (millis() - lastAtatElementMillis >= 20)){
    SoftPWMSet(explosionPin, 0);
    explosionDone = true;
  }

  wTrig.update();
  if (wTrig.isTrackPlaying(explosion) == false && explosionStopping == false) {
     explosionDone = true;
  }

  // Ending ATAT/Explosion sequence
  if (atatDoneState == true && explosionDone == true){
    if (wTrig.isTrackPlaying(explosion) == false) {
      rndmAtatShot = false;
      rndmExplosion = false;  
      explosionState = false;
      explosionDone = false; 
      explosionStopping = false; 
      atatDoneState = false;
      firedAtat = 0;
      atatShot = 0;
      atatState = false;
      backOffTimerAtat = true;
      lastAtatElementMillis = millis();     
    }
  }
}

void snowspeeder(){
  snowspeederState = true;
  if(speeder1 == false){
    if (millis() - previousSpeederMillis >= rndmSpeederMillis) {  
      previousSpeederMillis = millis(); 
      speederSound1 = random (5,8); 
      wTrig.trackPlayPoly(speederSound1);
      speeder1 = true;
    }
  }
  if (speeder2 == false && speeder1 == true){ 
    if (oneOrTwo > 12){  
      if (millis() - previousSpeederMillis >= rndmSpeederSpacerMillis) { 
        previousSpeederMillis = millis();   
        speederSound2 = random (5,8);          
        wTrig.trackPlayPoly(speederSound2);
        speeder2 = true;
      } 
    }else{
      speeder2 = true;
    }
  }
  
  if (speeder1 == true && speeder2 == true){
    if (millis() - previousSpeederMillis >= 1000) {
      if (shootOrNot1 <= 17){
        while(speederShot1<numberOfShots1){
          if ((millis() - previousSpeederShotMillis1 >= 80) && (firedSs1 == 0)) {
            previousSpeederShotMillis1 = millis();
            wTrig.trackPlayPoly(8);
            if ((speederShot1 % 2) == 0) SoftPWMSet(snowspeederCannon0Pin, 120); // Even
            if (speederShot1 % 2) SoftPWMSet(snowspeederCannon1Pin, 120); // Odd
            firedSs1 = 1;   
          }
          if ((millis() - previousSpeederShotMillis1 >= 80) && (firedSs1 == 1)) {
            previousSpeederShotMillis1 = millis();
            if ((speederShot1 % 2) == 0) SoftPWMSet(snowspeederCannon0Pin, 0);   
            if (speederShot1 % 2) SoftPWMSet(snowspeederCannon1Pin, 0); 
            speederShot1++;
            firedSs1 = 0;
          }
        }
      }
    }
    if (millis() - previousSpeederMillis >= 1000) {
      if (oneOrTwo > 12){ 
        if (shootOrNot2 <= 17){
          while(speederShot2 < numberOfShots2){
            if ((millis() - previousSpeederShotMillis2 >= 80) && (firedSs2 == 0)) {
              previousSpeederShotMillis2 = millis();
              wTrig.trackPlayPoly(8);
              if ((speederShot2 % 2) == 0) SoftPWMSet(snowspeederCannon2Pin, 120); // Even
              if (speederShot2 % 2) SoftPWMSet(snowspeederCannon3Pin, 120); // Odd
              firedSs2 = 1;    
            }
            if ((millis() - previousSpeederShotMillis2 >= 80) && (firedSs2 == 1)) {
              previousSpeederShotMillis2 = millis();
              if ((speederShot2 % 2) == 0) SoftPWMSet(snowspeederCannon2Pin, 0);   
              if (speederShot2 % 2) SoftPWMSet(snowspeederCannon3Pin, 0); 
              speederShot2++;
              firedSs2 = 0;
            }
          }
          if (speederShot1 >= numberOfShots1 && speederShot2 >= numberOfShots2 && snowspeederDoneState == false){
            snowspeederDoneState = true;
          }
        }
      }else if (oneOrTwo <= 12){
        snowspeederDoneState = true;
      }
      
      delay(20); // For wTrig.update()...
      if (snowspeederDoneState = true && (!wTrig.isTrackPlaying(speederSound1) and !wTrig.isTrackPlaying(speederSound2) and !wTrig.isTrackPlaying(8))){
        rndmSnowspeeder = false;
        snowspeederDoneState = false;
        speeder1 = false;      
        speeder2 = false;
        firedSs2 = 0;
        firedSs1 = 0;
        speederShot1 = 0;
        speederShot2 = 0;
        snowspeederState = false;
        backOffTimerSpeeder = false;
        previousSpeederMillis = millis();
      }
    }
  }
}

static void quiet(bool x){
  if (x==true){
    volumeChangeTrack_1 = volumeTrack_1 + -9; 
    volumeChangeTrack_999 = volumeTrack_999 + -9; 
  }
  if (x==false){
    volumeChangeTrack_1 = volumeTrack_1; 
    volumeChangeTrack_999 = volumeTrack_999; 
  }
  wTrig.trackFade(1, volumeChangeTrack_1, 50, 0);        // Fade Track 1 up/down volumeChange over 50 milli secs
  wTrig.trackFade(999, volumeChangeTrack_999, 50, 0);        // Fade Track 999 up/down volumeChange over 50 milli secs
}

void pause(){
    if (paused == true && still == false){
    if (millis() - previousPauseMillis >= 1500) {    // Basically means we wait 1.5 seconds before starting to blink.. 
      previousPauseMillis = millis();                // Storing the time we did this
      if (blinker == false){                        // Screen is blank, display something
        display.clearDisplay();                     // Clear the buffer 
        display.setTextSize(3);                     // Set the text size
        display.setTextColor(WHITE);                // ..and color
        display.setCursor(12,5);                    // ..and location
        display.println("PAUSED");                  // ..enter text
        display.display();                          // Display!
        delay(1);                                   // Guessing this is for "debouncing" the command sent to the display
        blinker = true;                             // Switch states, make the screen blank on the next interval
      }else if(blinker == true){                                        // Screen is not blank, blank it
        display.clearDisplay();                     // Clear the buffer
        display.display();                          // Display!
        delay(1);                                   // Guessing this is for "debouncing" the command sent to the display
        blinker = false;                            // Switch states, make the screen non-blank on the next interval
      }
    }  
  }

  // Return to normal..
  if (paused == false && previousPaused == true){    // If we are not paused, but we were paused last loop..
    asYouWere = true;                               // Set as you were state..
    wasPausedTimeMillis = millis();                       // Get the time now
    volumeAdjustedStep1 = false;                    // Some light house keeping in case we paused while adjusting volume  
    volumeAdjustedStep2 = false;                    // Some light house keeping in case we paused while adjusting volume 
    description();                                  // Display the text
  }

  if (asYouWere == true && paused != true){         // If as you were has been set, and we are not paused
    if (millis() - wasPausedTimeMillis >= 5000) {    // Basically means we wait 5 seconds before starting to scroll.. 
      display.startscrollright(0x00, 0x0F);         // Scroll screen contents to the right!
      asYouWere = false;                            // Reset the as you were state..we are as we were.. 
    }
  }
  // END - Flashing Pause
}

void randoms(){
  if (rndmAtatShot == false){
    numAtatShots = random(2,4);
    atatDelayMillis= random(1000,3000);
    rndmAtatShot = true;
  }
  if (rndmExplosion == false){
    rndmExplosionMillis = random(500, 1000);
    explosion = random(9,14);
    explodeOrNot = random(1,11);
    rndmExplosionMillis = true;
  }
  if (rndmSnowspeeder == false){
    rndmSpeederMillis = random(1000, 5000);
    rndmSpeederSpacerMillis = random(500, 1500);
    oneOrTwo = random(10,20);                       // Should the snowspeeder flight be a one of two ship formation?
    shootOrNot1 = random(10,20);                     // Should they fire or not?
    numberOfShots1 = random(1,4);                    // How many shots? From 1 to and including 3
    //rndmSpeederShotSpacerMillis1 = random(100,120);  // Time between salvos
    randomSeed(numberOfShots1);                       // Want to avoid pseudo randomly picking the same numbers again.. Looking into lottery-style drawing..
    shootOrNot2 = random(10,20);                     // Should they fire or not?
    numberOfShots2 = random(1,4);                    // How many shots? From 1 to and including 3
    //rndmSpeederShotSpacerMillis2 = random(100,120);  // Time between salvos
    rndmSnowspeeder = true;
  }
}

void volumeAdjusted(){
  if (volumeAdjustedStep1 == true && paused != true){ // If the volume was just adjusted, and we are non paused..
    if (millis()- adjustedVolumeTimeMillis >= 2000){  // Wait 2 seconds..
      volumeAdjustedStep2 = true;                     // Set the next state to enter..
      volumeAdjustedStep1 = false;                    // Done with the current state
      description();                                  // Remove volume indicator, and display text
    }
  }
  if (volumeAdjustedStep2 == true && paused != true){ // If we are in the second step after volume was adjusted, still not paused
    if (millis() - adjustedVolumeTimeMillis >= 7000){  // ..and we have waited another 5 (2+5=7) seconds
      display.startscrollright(0x00, 0x0F);           // Scroll screen contents to the right!
      volumeAdjustedStep2 = false;                    // Done with the current state..back to normal
    }
  }
}

void timedBootCmds(){
  if (scroll == false && paused == false){                     // We just booted up and are not scrolling yet, and we are not paused.. 
    if ((millis() >= 5000) && (millis() <= 5100)) { // If we are here between 5 seconds and 5.1 seconds, start scrolling
                                                              // The idea is that if we press pause immedately on boot, this time will come
                                                              // and go, but we will not be immedately scrolling the screen when we unpause..
      display.startscrollright(0x00, 0x0F);                   // Scroll screen contents to the right!
      scroll = true;                                          // We are scrolling, no need to enter this if statement again in that case.. 
    }  
  }
}