 /* 
 *  AT-AT Gun'n'Walk V.16
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
 */

// Include some libraries we need :) 
#include <SoftPWM.h>          // Software PWM due to too few analog write pins
#include <wavTrigger.h>       // For controlling the WAV Trigger via serial
#include <Wire.h>             // Used for I2C communications
#include <Adafruit_GFX.h>     // For graphics on the display  
#include <Adafruit_SSD1306.h> // The OLED library (defaults to the 128x32 size, must change if I go for bigger screen size)
#include "Adafruit_TPA2016.h" // Used for the I2C controlled stereo amplifier
#include <IRremote.h>         // Used for the IR sensor function

// Defining up the pins we will be using :)
// Pins used for I2C: A4/A5
#define snowspeederCannon0 8  // SoftPWM, speeder 1 cannon 1
#define snowspeederCannon1 5  // SoftPWM, speeder 1 cannon 2
#define snowspeederCannon2 A2 // SoftPWM, speeder 2 cannon 1
#define snowspeederCannon3 A3 // SoftPWM, speeder 2 cannon 2
#define atatCockpit 6         // PWM, light in AT AT cockpit
#define hlc0 11               // Soft PWM if necessary, left HLC
#define hlc1 3                // Soft PWM if necessary, right HLC
#define explosion 4           // Soft PWM, explosion on ground
#define RECV_PIN 12           // IR Sensor pin
#define OLED_RESET 13         // OLED requires a reset pin

// To make the diorama more dynamic, I will use pseudo randomization. These
// variables are meant to tell when to create new randoms. Basically on every run. 
bool rndmATATShot = false;          // false if no random timer set, true when waiting to execute
bool rndmATATExplosion = false;     // false if no random timer set, true when waiting to execute
bool rndmATATShotSpacer = false;    // false if no random timer set, true when waiting to execute
bool rndmSpeederFlight = false;     // false if no random timer set, true when waiting to execute
bool rndmSpeederSpacer = false;     // false if no random timer set, true when waiting to execute
bool rndmSpeederShot = false;       // false if no random timer set, true when waiting to execute
bool rndmSpeederShotSpacer = false; // false if no random timer set, true when waiting to execute

// Some bools to control various states
bool toggledAudio = true;         // True if audio ON
bool scroll = false;              // True = we are scrolling
bool blinker = false;             // Used to blink the pause screen
bool asYouWere = false;           // Back to business.. 
bool volumeAdjustedStep1 = false; // State just after volume has been adjusted, in two steps
bool volumeAdjustedStep2 = false; // State just after volume has been adjusted, in two steps
bool paused = false;              // Pause if true, unpause if false
bool previousPause = false;       // Saving the state we had on the last loop
bool started = false;             // True = started, False = "First run, do this once"

// Variables used to ensure the proper flow of the sequence. When set true, the sequence will 
// always move on to the next step. 
bool ATATShot1 = false; // False means we have not "been" there yet, true means we have and to "move on to the next"
bool ATATShot2 = false; // False means we have not "been" there yet, true means we have and to "move on to the next"
bool speeder1 = false;  // False means we have not "been" there yet, true means we have and to "move on to the next"
bool speeder2 = false;  // False means we have not "been" there yet, true means we have and to "move on to the next"

// Create some variables for randomization and timimg
byte ATATShotLength = 20;                   // The length of the ATAT laser cannon flash
unsigned long rndmATATShotMillis = 0;       // Time between each ATAT shooting sequence
unsigned long rndmATATExplosionMillis = 0;  // Time between last ATAT HCL firing and explosion
unsigned long rndmATATSpacerMillis = 0;     // How far between the AT AT HLC shots
unsigned long previousATATShotMillis = 0;   // This is the timer for the ATAT firing sequences, 
                                            // used to ensure that we trigger things at the right times
unsigned long rndmSpeederMillis = 0;            // The time between each Snowspeeder sequence
unsigned long rndmSpeederShotMillis = 0;        // The interval for the snowspeeder shots
unsigned long rndmSpeederSpacerMillis = 0;      // The time between first/second Snowspeeder
unsigned long rndmSpeederShotSpacerMillis = 0;  // The time between the Snowspeeder shots
unsigned long spacerSpeederShotMillis = 0;      // How far between the snowspeeders
unsigned long previousSpeederMillis = 0;        // This is the timer for the snowspeeder sequences
unsigned long startScroll = 0;                  // Used for the scrolling that starts automatically after boot          
unsigned long previousBlink = 0;                // Used for the blinking pause indicator
unsigned long wasPausedTime = 0;                // We want scrolling to resume after 
unsigned long adjustedVolumeTime = 0;           // Also after adjusting the volume "stuff" should happen

byte explodeOrNot;    // Used for a kind of dice roll to decide if the lasers hit, to generate an explosion
byte shootOrNot;      // Do the snowspeeder fire or not?
byte numberOfShots;   // How many speeder laser shots
byte speederShot = 0; // Count what shot we are at      
byte fired = 0;       // Used to ensure we enter the Snowspeeder firing sequence correctly
byte speeder = 0;     // Used to spread the firing on the two speeders 50/50..random was a bit biased
byte firing1,firing2; // Used for the same purpose, but is assigned the variables for the two cannon pairs

String hexRes = "000000"; // I decided to go for the value returned by the IR sensor, but in hex. Using this to store the hex value..

// The WAV Trigger object
wavTrigger wTrig;

// The Stereo Amplifier object
Adafruit_TPA2016 audioamp = Adafruit_TPA2016();

// Initialize the OLED display
Adafruit_SSD1306 display(OLED_RESET);

// IR Sensor object
IRrecv irrecv(RECV_PIN);  // Receiving IR sensor signals on this pin
decode_results results;   // Returns the decoded results (decode_type, addres, value, bits..etc.. Referenced using i.e. "results.value"

// Volume Settings
int maxVolume = -5;                 // Above this volume and we get crackling in the speakers..
int minVolume = -70;                // Below this and it gets silly, the amp goes to -70..why would we need to go lower
int volumeGain = -10;               // This is the default setting of dB from which we always start
int volumeGain_old = 0;             // Used to store the old volume setting when pausing so 
                                    // we know where to return to when unpausing
int newVolume = 0;                  // For mapping to "nicer" numbers

// LED brightnesses
int HLCBright = 200;            // Initially 120, too low
int ATATCockpitBright = 100;        // Initially 15, way too low

// Graphics Department.. 
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

void setup() {  
  SoftPWMBegin();       // Initiate SoftPWM, this allows "regular" pins to act as PWM pins. Very useful!

  // Define what pins are to be SoftPWM pins, and what their initial value will be
  SoftPWMSet(hlc0, 0);  
  SoftPWMSet(hlc1, 0);
  SoftPWMSet(snowspeederCannon0, 0);
  SoftPWMSet(snowspeederCannon1, 0);
  SoftPWMSet(snowspeederCannon2, 0);
  SoftPWMSet(snowspeederCannon3, 0);
  SoftPWMSet(explosion, 0);

  // Define the speed with which the LEDs will fade in, and fade out, respectively. 
  SoftPWMSetFadeTime(hlc0, 10, 50);
  SoftPWMSetFadeTime(hlc1, 10, 50);
  SoftPWMSetFadeTime(snowspeederCannon0, 10, 50);
  SoftPWMSetFadeTime(snowspeederCannon1, 10, 50);
  SoftPWMSetFadeTime(snowspeederCannon2, 10, 10);
  SoftPWMSetFadeTime(snowspeederCannon3, 10, 10);
  SoftPWMSetFadeTime(explosion, 10, 1000);        // The explosion looks better if it takes a while to die down. 
  
  // WAV Trigger startup
  wTrig.start();
  delay(10);
  wTrig.stopAllTracks();      // Just in case..we want to start with a clean slate
  wTrig.samplerateOffset(0);  // Same here, reset any offset (speed/pitch). 
                              // I guess this is good practice, though I will not use the function
  // Start the amplifier
  audioamp.begin();

  // Starting IR sensor receiver
  irrecv.enableIRIn(); // Start the receiver

  // Some display code
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C for the 128x32
  //display.display();                          // Display the splash screen..
  //delay(2000);                                // ..for two seconds.. 
  description();
}

void description(){
  display.clearDisplay();                     // Clear the buffer 
  display.drawBitmap(0, 0, First_Galactic_Empire_emblem_bits, 32, 31, 1); // miniature bitmap display
  display.setTextSize(3);                     // Set the text size
  display.setTextColor(WHITE);                // ..and color
  display.setCursor(45,0);                    // ..and location
  display.println("HOTH");            // ..enter text
  display.setTextSize(1);                     // Set the text size
  display.setCursor(47,25);                   // ..new line
  display.println("Version:1.0");             // ..enter text
  display.display();                          // Display!
  delay(1);
}

void lightsOut(){                     // Make everything dark for the pause
  analogWrite(atatCockpit, 0);               
  SoftPWMSet(hlc0, 0);  
  SoftPWMSet(hlc1, 0);
  SoftPWMSet(snowspeederCannon0, 0);
  SoftPWMSet(snowspeederCannon1, 0);
  SoftPWMSet(snowspeederCannon2, 0);
  SoftPWMSet(snowspeederCannon3, 0);
  SoftPWMSet(explosion, 0);
}
void lightsOn(){                      // Some LEDs should be lit again after the pause
  analogWrite(atatCockpit, 0);  
}

void loop() {
  // Time since boot..this is used all over the place
  unsigned long currentMillis = millis();
  
  // START - IR Receiver Code
  // Check if there is anything being received we should decode
  if (irrecv.decode(&results)) {          // Get the results of the decoding..
    hexRes = String(results.value, HEX);  // Make it a hex value
    irrecv.resume();                      // Resume receiving..
  }
  // What to do for this and that code
      if (hexRes == "fd40bf"){                              // This is hex for volume up
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
      adjustedVolumeTime = currentMillis;           // Take the time when the last volume adjustment was made
      volumeAdjustedStep1 = true;                   // We are in this state now..
      volumeAdjustedStep2 = false;                  // We are waiting for this state..  
    }
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
      adjustedVolumeTime = currentMillis;           // Take the time when the last volume adjustment was made
      volumeAdjustedStep1 = true;                   // We are in this state now..
      volumeAdjustedStep2 = false;                  // We are waiting for this state..      
    }
    if (hexRes == "fd807f"){                        // Hex for Play / Pause
      if (paused == false){                         // We were unpaused, but the button was pressed..start pausing!
        display.stopscroll();                       // Stop any scrolling of the screen
        delay(1);
        display.clearDisplay();                     // Clear the buffer
        display.display();                          // Display! 
        delay(1);
        lightsOut();                                // Will this work?
        volumeGain_old = volumeGain;                // Save the old volume setting
        for (int i=volumeGain; i >= -70; i--){      // -70 is total silence..
          wTrig.masterGain(i);                      // Set the master gain
          delay(10);                                // Such a short delay that it is "ok". Just a "debounce"
        } 
        audioamp.enableChannel(false, false);       // Turn off both channels using I2C
        paused = true;                              // We are now paused.. 
        display.clearDisplay();                     // Clear the buffer
        display.setTextSize(3);                     // Set the text size
        display.setTextColor(WHITE);                // ..and color
        display.setCursor(12,5);                    // ..and location
        display.println("PAUSED");                  // ..enter text
        display.display();                          // Display!
        delay(1);
        previousBlink = currentMillis;
      }else if (paused == true){                    // We were paused, but the button was pressed..start unapusing, get the show on the road!
        lightsOn();
        audioamp.enableChannel(true, true);         // Turn on both channels using I2C      
        for (int i=-70; i <= volumeGain_old; i++){  // Increase the volume back to the old setting  
          wTrig.masterGain(i);                      // Set the master gain
          delay(10);                                // Such a short delay that it is "ok". Just a "debounce"
        } 
        paused = false;                             // We are now unpaused.. 

      } 
      hexRes = "000000";     
    }
    if (hexRes == "fd08f7"){                        // Hex for 1
       if (toggledAudio == true){                   // We were unpaused, but the button was pressed..start pausing!
        volumeGain_old = volumeGain;                // Save the old volume setting
        for (int i=volumeGain; i >= -70; i--){      // -70 is total silence..
          wTrig.masterGain(i);                      // Set the master gain
          delay(10);                                // Such a short delay that it is "ok". Just a "debounce"
        } 
        audioamp.enableChannel(false, false);       // Turn off both channels using I2C
        toggledAudio = false;                       // We are now paused.. 
      }else if (toggledAudio == false){             // We were paused, but the button was pressed..start unapusing, get the show on the road!
        audioamp.enableChannel(true, true);         // Turn on both channels using I2C      
        for (int i=-70; i <= volumeGain_old; i++){  // Increase the volume back to the old setting  
          wTrig.masterGain(i);                      // Set the master gain
          delay(10);                                // Such a short delay that it is "ok". Just a "debounce"
        } 
        toggledAudio = true;                        // We are now unpaused.. 
      } 
      hexRes = "000000";     
    }


 /* A list of possible codes from the remote control
  * fd40bf Vol +        ## Used Vol +
  * fd00ff Vol -        ## Used Vol -
  * fd807f Play Pause   ## Used Play/Pause
  * fd08f7 1            ## USED - Toggle Audio mode ON/OFF
  * 
  * fd20df Setup        
  * fda05f Up           ## Will use for menu
  * fd609f Stop / Mode  
  * fd10ef Left         ## Will use for menu
  * fd906f Enter / Save ## Will use for menu
  * fd50af Right        ## Will use for menu
  * fd30cf 0 10+
  * fdb04f Down         ## Will use for menu
  * fd708f Back         ## Will use for menu
  * fd8877 2             
  * fd48b7 3
  * fd28d7 4
  * fda857 5
  * fd6897 6
  * fd18e7 7
  * fd9867 8 
  * fd58a7 9
*/
  // END - IR Receiver Code

  // START - OLED Animations
  // START - Flashing Pause
  if (paused == true){
    if (currentMillis - previousBlink >= 1500) {    // Basically means we wait 5 seconds before starting to scroll.. 
      previousBlink = currentMillis;                // Storing the time we did this
      if (blinker == false){                        // Screen is blank, display something
        display.clearDisplay();                     // Clear the buffer 
        display.setTextSize(3);                     // Set the text size
        display.setTextColor(WHITE);                // ..and color
        display.setCursor(12,5);                    // ..and location
        display.println("PAUSED");                  // ..enter text
        display.display();                          // Display!
        delay(1);                                   // Guessing this is for "debouncing" the command sent to the display
        blinker = true;                             // Switch states, make the screen blank on the next interval
      }else{                                        // Screen is not blank, blank it
        display.clearDisplay();                     // Clear the buffer
        display.display();                          // Display!
        delay(1);                                   // Guessing this is for "debouncing" the command sent to the display
        blinker = false;                            // Switch states, make the screen non-blank on the next interval
      }
    }  
  }

  // Return to normal..
  if (paused == false && previousPause == true){    // If we are not paused, but we were paused last loop..
    asYouWere = true;                               // Set as you were state..
    wasPausedTime = currentMillis;                  // Get the time now
    volumeAdjustedStep1 = false;                    // Some light house keeping in case we paused while adjusting volume  
    volumeAdjustedStep2 = false;                    // Some light house keeping in case we paused while adjusting volume 
    description();                                  // Display the text
  }

  if (asYouWere == true && paused != true){         // If as you were has been set, and we are not paused
    if (currentMillis - wasPausedTime >= 5000) {    // Basically means we wait 5 seconds before starting to scroll.. 
      display.startscrollright(0x00, 0x0F);         // Scroll screen contents to the right!
      asYouWere = false;                            // Reset the as you were state..we are as we were.. 
    }
  }
  // END - Flashing Pause

  // START - Volume Was Adjusted
  if (volumeAdjustedStep1 == true && paused != true){ // If the volume was just adjusted, and we are non paused..
    if (currentMillis - adjustedVolumeTime >= 2000){  // Wait 2 seconds..
      volumeAdjustedStep2 = true;                     // Set the next state to enter..
      volumeAdjustedStep1 = false;                    // Done with the current state
      description();                                  // Remove volume indicator, and display text
    }
  }
  if (volumeAdjustedStep2 == true && paused != true){ // If we are in the second step after volume was adjusted, still not paused
    if (currentMillis - adjustedVolumeTime >= 7000){  // ..and we have waited another 5 (2+5=7) seconds
      display.startscrollright(0x00, 0x0F);           // Scroll screen contents to the right!
      volumeAdjustedStep2 = false;                    // Done with the current state..back to normal
    }
  }
  // END - Volume Was Adjusted

  // START - Scrolling Screen After Boot
  if (scroll == false && paused != true){                     // We just booted up and are not scrolling yet, and we are not paused.. 
    if ((currentMillis >= 5000) && (currentMillis <= 5050)) { // If we are here between 5 seconds and 5.05 seconds, start scrolling
                                                              // The idea is that if we press pause immedately on boot, this time will come
                                                              // and go, but we will not be immedately scrolling the screen when we unpause..
      display.startscrollright(0x00, 0x0F);                   // Scroll screen contents to the right!
      scroll = true;                                          // We are scrolling, no need to enter this if statement again in that case.. 
    }  
  }
  // END - Scrolling Screen After Boot
  // END - OLED Animations

  if (paused == false){   // false = not paused

    // START - Code only executed the very first time through the loop
    if (started == false){                          // If this is the first time then started = false
      analogWrite(atatCockpit, ATATCockpitBright);  // Light the ATAT cockpit
      
      //wTrig.stopAllTracks();          // Was used for a period when I tried to halt the sequence.
                                        // Will keep this in case I find I want to add an on/off feature..
      wTrig.masterGain(volumeGain);     // Sets the master gain to whatever is set (set low during dev to 
                                        // maintain neighbors' sanity 
      wTrig.trackPlayPoly(1);           // Start Track 1, AT AT walking. Poly allows more tracks at the same time
      wTrig.trackLoop(1, 1);            // Loop Track 1
      wTrig.trackGain(999, -6);         // Set the track gain for track 999. This is background music
      wTrig.trackPlayPoly(999);         // Start Track 999, back ground music. Also poly. We want more tracks.
      wTrig.trackLoop(999, 1);          // Loop track 999 as well.. 
                                        // Must keep in mind that we can only play 8 tracks at any given time.. 
      started = true;                   // Set this to true so we never enter this part of the program again, 
                                        // unless there is a power cycle.

      // Stereo Amplifier commands (I2C)
      audioamp.enableChannel(true, false);           // Turn on the amplifier, both channels (I2C)
      audioamp.setGain(0);                          // Set the initial gain to 0. Must make a decision 
                                                    // to either use the amp or the wav trigger for volume adjustment
      //audioamp.setLimitLevelOn();                 // For testing
      //audioamp.setLimitLevel(13);                 // For testing
      //audioamp.setAGCCompression(TPA2016_AGC_4);  // For testing
      audioamp.setAGCCompression(TPA2016_AGC_OFF);  // For testing, currently the AGC is off and seems to be what I wanted
      //audioamp.setAttackControl(1);               // For testing
      //audioamp.setHoldControl(1);                 // For testing
      //audioamp.setReleaseControl(5);              // For testing
 
    }
    // END - Code only executed the very first time through the loop

    // START - Pseudo random generators
    if (rndmATATShot == false){                     // Basically check if new random timer should be calculated
      rndmATATShotMillis = random(1500, 4000);      // Pseudo random, will do the trick for this
                                                    // Actually it has turned out to be a good thing for troubleshooting
                                                    // ATAT Shots should occur every 1.5 - 4 seconds
      rndmATATShot = true;                          // New timer calculated, don't do it again until told to
    }
    if (rndmATATShotSpacer == false){
      rndmATATSpacerMillis = random(100, 200);      // The ATAT shots should be spaced by 100-200 ms
      rndmATATShotSpacer = true;
    }
    if (rndmATATExplosion == false){
      explodeOrNot = random(1,11);
      rndmATATExplosionMillis = random(500, 1000);  // The ATAT HLC should "hit" every 0.5 and 1 second after the second
                                                    // HLC has fired
      rndmATATExplosion = true;
    }
  
    if (rndmSpeederFlight == false){
      shootOrNot = random(15,20);                     // Should they fire or not?
      numberOfShots = random(1,4);                    // How many shots? From 1 to and including 3
      rndmSpeederShotSpacerMillis = random(100,200);  // Time between first two salvos
      rndmSpeederMillis = random(1000, 5000);         // The snowspeeder flight interval
      rndmSpeederFlight = true;
    }
    if (rndmSpeederSpacer == false){
      rndmSpeederSpacerMillis = random(500, 1000);    // The time between the snowspeeders in each flight. From 0.5 to 1 second apart.
      rndmSpeederSpacer = true;
    }
    if (rndmSpeederShot == false){
      rndmSpeederShotMillis = random(30, 60);         // The time between each snowspeeder shot, from 30 to 60 ms. 
      rndmSpeederShot = true;
    }
    
    // END - Random generators

    /* I have decided to do the ATAT and Snowspeeders as separate entities that exist in code independent of each other. Multitasking. 
    * This is where the "animation" takes place, timing all the events etc. 
    * This relies heavily on the use of millis() to know where in time the code is, and to trigger events at given (random) intervals. 
    * The theory is this: millis() counts milliseconds from the time the micro controller was powered up. This number is ever increasing and 
    * to point to any given millisecond we use "unsigned long"; a 32 bit number with no negative component: 0 to 4,294,967,295 (2^32 - 1). 
    * There are 3.6 million milliseconds pr hour..this number will allow the Trinket to run for approx 1193 hours, or almost 50 days, before rolling over.
    * I will never leave my diorama on for that long, so I am not even going to bother with dealing with a rollover in this code. 
    * 
    * Anyway, if time X has passed since power on, and we want Z to happen at time Y we must check what the time is, and if time Y has passed yet. 
    * I have created a variable called currentMillis that I use for this purpose, and at first "glance" the pseudo code could be this: 
    * If currentMillis is bigger or equal to Y, then do Z 
    * 
    * However this means we will do Z over and over and over and over again as the code loops through after we pass time Y the first time. 
    * In other words; The if statement will forever be true after time Y
    * 
    * To counter this and to make it happen at time Y intervals instead, we must add more math. 
    * In fact we must also add a new variable to store the time we last performed Z: i.e. previousMillis
    * 
    * The new pseudo code becomes:
    * if currentMillis minus previousMillis is greater than or equal to Y, then do Z
    * 
    * This means that as the code starts to run previousMillis will be 0. We have not stored a new time yet. This means
    * that after currentMillis is greater than or equal to time Y, Z happens. We store time Y as previousMillis. 
    * Then as the code loops we are faced with this math statement: currentMillis minus previousMillis, this will be less than time Y until the 
    * interval has once again passed. After which, we store a new time in previousMillis and so on and so on. 
    * 
    * I had a hard time getting my head around this simple concept. Once I did it was easy to link events using the previousMillis idea. 
    * So in my code I maintain only two of these variables. One for the ATATs and one for the Snowspeeders. If one knows about this, and listens 
    * carefully you can hear the sequence of things:
    * The ATAT fires twice, then an explosion occurs. The snowspeeders passes and fires. To mix things up I have added randomizers to maybe there is
    * an explosion or not after the ATAT is done. Maybe the snowspeeders dont't fire on their attack run, and they fire a random number of times when they do. 
    *
    * In addition, I don't want things to happen BEFORE I am ready, or rather. I want to make things a bit random. So when the ATAT has fired, it
    * should not fire again until a new time Y has been calculated. Thus I use a second IF statment to control the workflow. 
    */
    
    // START - AT AT Firing + Explosion 
    if(ATATShot1 == false){                                                 // Do not confuse this with the randomizer variables. 
                                                                            // This is used to control the flow. If the first HLC has fired
                                                                            // then we should not return here yet..the full "ATAT sequence" must finish first..
      if (currentMillis - previousATATShotMillis >= rndmATATShotMillis) {   // Basically means we wait rndmATATShotMillis before the HLC fires..
        previousATATShotMillis = currentMillis;                             // Storing the time this happened last, so that the next will happen
                                                                            // rndmATATShotMillis after this point in time. 
        wTrig.trackPlayPoly(4);                                             // Play track 4, the single ATAT laser sound
        SoftPWMSet(hlc0, HLCBright);                                              // Light up LED on pin hcl0 to a value of 120
        ATATShot1 = true;                                                   // We have fired the first HLC..now to fire the next.. 
      }
    }
    if (currentMillis - previousATATShotMillis >= ATATShotLength) SoftPWMSet(hlc0, 0);  // The HLC LED must be turned off, and this should happen fairly fast
                                                                                        // after it was lit. ATATShotLength is set to 20ms
                                                                                        // Here we see previousATATShotMilis still being used as a time ref for 
                                                                                        // the ATAT sequence. No point in storing this time. If the HLCs fire almost
                                                                                        // on top of each other that's "fine"
    if (ATATShot1 == true && ATATShot2 == false){                             // If we have fired the first, but not the second HLC, then do this
      if (currentMillis - previousATATShotMillis >= rndmATATSpacerMillis) {   // Basically means we still have to wait rndmATATSpacerMillis amount of time before doing this. 
        previousATATShotMillis = currentMillis;                               // Storing the time we did this
        wTrig.trackPlayPoly(4);                                               
        SoftPWMSet(hlc1, HLCBright);  
        ATATShot2 = true;                                                     // We're done firing the second HLC..moving on
      }   
    }
    if (currentMillis - previousATATShotMillis >= ATATShotLength) SoftPWMSet(hlc1, 0);  // Turn off the second HLCs LED after 20ms
    
    if ((ATATShot1 == true) && (ATATShot2 == true)){                            // Now that we are done firing, we can move to the explosion bit
      if (currentMillis - previousATATShotMillis >= rndmATATExplosionMillis) {  // Basically means we wait rndmATATExplosionMillis for this bit. 
        if (explodeOrNot > 4){                                                  // A way to weight the randomness..if the value of exlodeOrNot is 
                                                                                // greater than 4 we get an explosion! 
          previousATATShotMillis = currentMillis;                               // Storing time.. 
          // Explosions = Files 9 - 13
          int x = random(9,14);                                                 // Random audio file to play in place of the explosion
          wTrig.trackPlayPoly(x);
          SoftPWMSet(explosion, 200);                                           // Light the explosion LED to a value of 200
        }
        rndmATATShot = false;                                                   // Return a bunch of values set to true during the sequence back to false
        rndmATATExplosion = false;                                              // to start the ATAT sequence over. 
        rndmATATShotSpacer = false;
        ATATShot1 = false;      
        ATATShot2 = false;
      }
    }
    if (currentMillis - previousATATShotMillis >= ATATShotLength) SoftPWMSet(explosion, 0); // Kill the explosion LED. The fade is set elsewhere so turning it off after
                                                                                            // 20 ms is fine
    // END - AT AT Firing + Explosion

    // START - Speeder flying and firing
    if(speeder1 == false){
      if (currentMillis - previousSpeederMillis >= rndmSpeederMillis) {  
        previousSpeederMillis = currentMillis; 
        // Snowspeeder flyby = Files 4 - 7
        int y = random (5,8); 
        wTrig.trackPlayPoly(y); 
        speeder1 = true;
      }
    }
    if (speeder2 == false && speeder1 == true){ 
      if (currentMillis - previousSpeederMillis >= rndmSpeederSpacerMillis) {  
        previousSpeederMillis = currentMillis;
        // Snowspeeder flyby = Files 4 - 7
        int z = random (5,8);          
        wTrig.trackPlayPoly(z); 
        speeder2 = true;
      } 
    }
    if (speeder1 == true && speeder2 == true){
      if (currentMillis - previousSpeederMillis >= rndmSpeederShotMillis) {  
        if (shootOrNot <= 17){
          previousSpeederMillis = currentMillis;
          if (speeder == 0){
            firing1 = snowspeederCannon0;
            firing2 = snowspeederCannon1;
            speeder++;
          } else if (speeder == 1){
            firing1 = snowspeederCannon2;
            firing2 = snowspeederCannon3;
            speeder = 0;
          }
          while(speederShot<numberOfShots){
            currentMillis = millis();       // Get time while inside the while loop.. 
            if ((currentMillis - previousSpeederMillis >= rndmSpeederShotSpacerMillis) && (fired == 0)) {
              previousSpeederMillis = currentMillis;
              wTrig.trackPlayPoly(8);
              if ((speederShot % 2) == 0) SoftPWMSet(firing1, 120); // Even
              if (speederShot % 2) SoftPWMSet(firing2, 120); // Odd
              //rndmSpeederShotSpacerMillis = random(100,300);
              fired = 1;    
            }
            if ((currentMillis - previousSpeederMillis >= 20) && (fired == 1)) {
               if ((speederShot % 2) == 0) SoftPWMSet(firing1, 0);   
               if (speederShot % 2) SoftPWMSet(firing2, 0); 
               speederShot++;
               fired = 0;
            }
          }
          rndmSpeederSpacer = false;
          rndmSpeederFlight = false;
          rndmSpeederShot = false;
          speeder1 = false;      
          speeder2 = false;
          speederShot = 0;          // Reset for new run
        }else if (shootOrNot > 17){
          rndmSpeederSpacer = false;
          rndmSpeederFlight = false;
          rndmSpeederShot = false;
          speeder1 = false;      
          speeder2 = false;
        }
      }
    }
    // END - Speeder flying and firing
  }else if (paused == true) {           // If we are paused, go here.. 
    // do nothing..we are holding.. 
  }

  // Get the pause state
  previousPause = paused;
}