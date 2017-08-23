/* 
 *  AT-AT Gun'n'Walk V.1
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
 * - Rotary encoder: https://learn.adafruit.com/trinket-usb-volume-knob/code
 * - Odd/Even: http://forum.arduino.cc/index.php?topic=41397.0
 * 
 * Datasheets: 
 * - Rotary encoder: https://cdn-shop.adafruit.com/datasheets/pec11.pdf
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
 * 
 */

// Include some libraries we need :) 
#include <SoftPWM.h>          // Software PWM due to too few analog write pins
#include <wavTrigger.h>       // For controlling the WAV Trigger via serial

// Defining up the pins we will be using :)
// Pins used for I2C: A4/A5/8
#define snowspeederEng0 9         // PWM, left engine
#define snowspeederEng1 10        // PWM, right engine
#define snowspeederCannon0 8      // SoftPWM, left cannon
#define snowspeederCannon1 A0     // SoftPWM, right cannon
#define atatCockpit 6             // PWM, light in AT AT cockpit
#define hlc0 A2                   // Soft PWM if necessary, left HLC
#define hlc1 A3                   // Soft PWM if necessary, right HLC
#define explosion A1              // Soft PWM, explosion on ground
#define selectorSwitch 4          // State control button switch, was 8..but changed to 4 due to quicker read (PIND)
#define selectorUp 3              // SelectorUp
#define selectorDn 5              // SelectorDown
#define selectorPin PIND          // https://www.arduino.cc/en/Reference/PortManipulation 
                                  // Baiscally this means I must use the pins in the 0-7 range to use PIND.
                                  // This is all because reading the pins takes longer than direct port access. 

// To make the diorama more dynamic, I will use pseudo randomization. These
// variables are meant to tell when to create new randoms. Basically on every run. 
bool rndmATATShot = false;          // false if no random timer set, true when waiting to execute
bool rndmATATExplosion = false;     // false if no random timer set, true when waiting to execute
bool rndmATATShotSpacer = false;    // false if no random timer set, true when waiting to execute
bool rndmSpeederFlight = false;     // false if no random timer set, true when waiting to execute
bool rndmSpeederSpacer = false;     // false if no random timer set, true when waiting to execute
bool rndmSpeederShot = false;       // false if no random timer set, true when waiting to execute
bool rndmSpeederShotSpacer = false; // false if no random timer set, true when waiting to execute

// Variables used to ensure the proper flow of the sequence. When set true, the sequence will 
// always move on to the next step. 
bool ATATShot1 = false; // False means we have not "been" there yet, true means we have and to "move on to the next"
bool ATATShot2 = false; // False means we have not "been" there yet, true means we have and to "move on to the next"
bool speeder1 = false;  // False means we have not "been" there yet, true means we have and to "move on to the next"
bool speeder2 = false;  // False means we have not "been" there yet, true means we have and to "move on to the next"

// To allow for some start-up stuff to happen once. Set true when start up has happened. 
bool started = false; // True = started, False = "First run, do this once"

// Create some variables for randomization
byte ATATShotLength = 20;                   // The length of the ATAT laser cannon flash
unsigned long rndmATATShotMillis = 0;       // Time between each ATAT shooting sequence
unsigned long rndmATATExplosionMillis = 0;  // Time between last ATAT HCL firing and explosion
unsigned long rndmATATSpacerMillis = 0;     // How far between the AT AT HLC shots
unsigned long previousATATShotMillis = 0;   // This is the timer for the ATAT firing sequences, 
                                            // used to ensure that we trigger things at the right times

unsigned long rndmSpeederMillis = 0;            // The time between each Snowspeeder sequence
unsigned long rndmSpeederShotMillis = 0;        // The interval for the snowspeeder shots
unsigned long rndmSpeederSpacerMillis = 0;      // The time between first/second Snowspeeder
unsigned long rnmdSpeederShotSpacerMillis = 0;  // The time between the Snowspeeder shots
unsigned long spacerSpeederShotMillis = 0;      // How far between the snowspeeders
unsigned long previousSpeederMillis = 0;        // This is the timer for the snowspeeder sequences

byte explodeOrNot;    // Used for a kind of dice roll to decide if the lasers hit, to generate an explosion
byte oneOrTwo;        // Two or one snowspeeder will fly past?
byte shootOrNot;      // Do the snowspeeder fire or not?
byte numberOfShots;   // How many speeder laser shots
byte speederShot = 0; // Count what shot we are at      
byte fired = 0;       // Used to ensure we enter the Snowspeeder firing sequence correctly

// The WAV Trigger object
wavTrigger wTrig;

// Rotary Encoder, pressing the rotary encoder button = pause/unpause, rotating changes the volume
static uint8_t enc_prev_pos   = 0;  // These were copied directly from Adafruit's example
static uint8_t enc_flags      = 0;  // These were copied directly from Adafruit's example
static char    sw_was_pressed = 0;  // These were copied directly from Adafruit's example
int volumeGain = -8;                // This is the default setting of dB from which we always start
int volumeGain_old = 0;             // Used to store the old volume setting when pausing so 
                                    // we know where to return to when unpausing
bool paused = false;                // Pause if true, unpause if false

void setup() {
  SoftPWMBegin();     // Initiate SoftPWM, this allows "regular" pins to act as PWM pins. Very useful!

  // Define what pins are to be SoftPWM pins, and what their initial value will be
  SoftPWMSet(hlc0, 0);  
  SoftPWMSet(hlc1, 0);
  SoftPWMSet(snowspeederCannon0, 0);
  SoftPWMSet(snowspeederCannon1, 0);
  SoftPWMSet(explosion, 0);

  // Define the speed with which the LEDs will fade in, and fade out, respectively. 
  SoftPWMSetFadeTime(hlc0, 10, 50);
  SoftPWMSetFadeTime(hlc1, 10, 50);
  SoftPWMSetFadeTime(snowspeederCannon0, 10, 50);
  SoftPWMSetFadeTime(snowspeederCannon1, 10, 50);
  SoftPWMSetFadeTime(explosion, 10, 1000);        // The explosion looks better if it takes a while to die down. 

  // set pins as input with internal pull-up resistors enabled, copied from Adafruit
  pinMode(selectorUp, INPUT_PULLUP);
  pinMode(selectorDn, INPUT_PULLUP);
  pinMode(selectorSwitch, INPUT_PULLUP);

  // get an initial reading on the encoder pins, copied from Adafruit 
  if (digitalRead(selectorUp) == LOW) {
    enc_prev_pos |= (1 << 0);
  }
  if (digitalRead(selectorDn) == LOW) {
    enc_prev_pos |= (1 << 1);
  }

  // WAV Trigger startup at 57600
  wTrig.start();
  delay(10);
  wTrig.stopAllTracks();      // Just in case..we want to start with a clean slate
  wTrig.samplerateOffset(0);  // Same here, reset any offset (speed/pitch). 
                              // I guess this is good practice, though I will not use the function
}

void loop() {
  // START - Rotary Encoder Code, copied from Adafruit
  int8_t enc_action = 0; // 1 or -1 if moved, sign is direction
 
  // note: for better performance, the code will use
  // direct port access techniques
  // http://www.arduino.cc/en/Reference/PortManipulation
  // This bit was necessary to read in order to use the correct pin configuration.
  // I use a Pro Trinket, it varies slightly from the regular full size Arduino boards.
  
  uint8_t enc_cur_pos = 0;
  // read in the encoder state first
  if (bit_is_clear(selectorPin, selectorUp)) {
    enc_cur_pos |= (1 << 0);
  }
  if (bit_is_clear(selectorPin, selectorDn)) {
    enc_cur_pos |= (1 << 1);
  }
 
  // if any rotation at all
  if (enc_cur_pos != enc_prev_pos)
  {
    if (enc_prev_pos == 0x00)
    {
      // this is the first edge
      if (enc_cur_pos == 0x01) {
        enc_flags |= (1 << 0);
      }
      else if (enc_cur_pos == 0x02) {
        enc_flags |= (1 << 1);
      }
    }
 
    if (enc_cur_pos == 0x03)
    {
      // this is when the encoder is in the middle of a "step"
      enc_flags |= (1 << 4);
    }
    else if (enc_cur_pos == 0x00)
    {
      // this is the final edge
      if (enc_prev_pos == 0x02) {
        enc_flags |= (1 << 2);
      }
      else if (enc_prev_pos == 0x01) {
        enc_flags |= (1 << 3);
      }
 
      // check the first and last edge
      // or maybe one edge is missing, if missing then require the middle state
      // this will reject bounces and false movements
      if (bit_is_set(enc_flags, 0) && (bit_is_set(enc_flags, 2) || bit_is_set(enc_flags, 4))) {
        enc_action = 1;
      }
      else if (bit_is_set(enc_flags, 2) && (bit_is_set(enc_flags, 0) || bit_is_set(enc_flags, 4))) {
        enc_action = 1;
      }
      else if (bit_is_set(enc_flags, 1) && (bit_is_set(enc_flags, 3) || bit_is_set(enc_flags, 4))) {
        enc_action = -1;
      }
      else if (bit_is_set(enc_flags, 3) && (bit_is_set(enc_flags, 1) || bit_is_set(enc_flags, 4))) {
        enc_action = -1;
      }
 
      enc_flags = 0; // reset for next time
    }
  }
 
  enc_prev_pos = enc_cur_pos;
 
  if (enc_action > 0) {
    volumeGain--;                 // Clockwise, i.e. send multimedia volume up
    wTrig.masterGain(volumeGain); // Set the master gain
  }
  else if (enc_action < 0) {
    volumeGain++;                 // Counterclockwise, i.e. multimedia volume down
    wTrig.masterGain(volumeGain); // Set the master gain
  }
  
  // remember that the switch is active low 
  if (bit_is_clear(selectorPin, selectorSwitch)) 
  {
    if (sw_was_pressed == 0) // only on initial press, so the keystroke is not repeated while the button is held down
    {
      if (paused == false){                         // We were unpaused, but the button was pressed..start pausing!
        volumeGain_old = volumeGain;                // Save the old volume setting
        for (int i=volumeGain; i >= -70; i--){      // -70 is total silence..
          wTrig.masterGain(i);                      // Set the master gain
          delay(10);                                // Swap this with millis() function..
                                                    // now it causes some level of "halt" when maniuplating the volume
        } 
        //digitalWrite(ampShutdown, LOW);           // Used to send shutdown signal to a SD pin on the amplifier. Will use I2C instead.
                                                    // Waiting for the new I2C enable amp to arrive. 
        paused = true;                              // We are now paused.. 
      }else if (paused == true){                    // We were paused, but the button was pressed..start unapusing, get the show on the road!
        //digitalWrite(ampShutdown, HIGH);          // Used to send shutdown signal to a SD pin on the amplifier. Will use I2C instead.
                                                    // Waiting for the new I2C enable amp to arrive.      
        for (int i=-70; i <= volumeGain_old; i++){  // Increase the volume back to the old setting  
          wTrig.masterGain(i);                      // Set the master gain
          delay(10);                                // Swap this with millis() function, see above
        } 
        paused = false;                             // We are now unpaused.. 
      }      
      // Encoder pushed down, i.e. toggle mute or not or select/enter
      delay(5); // debounce delay
    }
    sw_was_pressed = 1;
  }
  else
  {
    if (sw_was_pressed != 0) {
      delay(5); // debounce delay
    }
    sw_was_pressed = 0;
  }
  // END - Rotary Encoder Code
  
  // Time since starting..this is used all over the place
  unsigned long currentMillis = millis();

  if (paused == false){ // false = not paused

    // START - Code only executed the very first time through the loop
    if (started == false){              // If this is the first time then started = false
      analogWrite(atatCockpit, 15);     // Light the ATAT cockpit
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
    }
    // END - Code only executed the very first time through the loop

    // START - Random generators
    if (rndmATATShot == false){
      rndmATATShotMillis = random(1500, 4000); // Pseudo random, will do the trick for this
      rndmATATShot = true;
    }
    if (rndmATATShotSpacer == false){
      rndmATATSpacerMillis = random(100, 200); // Pseudo random, will do the trick for this
      rndmATATShotSpacer = true;
    }
    if (rndmATATExplosion == false){
      explodeOrNot = random(1,11);
      rndmATATExplosionMillis = random(500, 1000); // Pseudo random, will do the trick for this
      rndmATATExplosion = true;
    }
  
    if (rndmSpeederFlight == false){
      oneOrTwo = random(10,15);
      shootOrNot = random(15,20);
      numberOfShots = random(1,4);                  // a number from 1 to and including 3
      rnmdSpeederShotSpacerMillis = random(100,200); // Time between first two salvos
      rndmSpeederMillis = random(1000, 5000);       // Pseudo random, will do the trick for this
      rndmSpeederFlight = true;
    }
    if (rndmSpeederSpacer == false){
      rndmSpeederSpacerMillis = random(500, 1000);    // Pseudo random, will do the trick for this
      rndmSpeederSpacer = true;
    }
    if (rndmSpeederShot == false){
      rndmSpeederShotMillis = random(30, 60);         // Pseudo random, will do the trick for this
      rndmSpeederShot = true;
    }
    // END - Random generators

    //unsigned long currentMillis = millis();   // Pretty sure this is redundant..

    // START - AT AT Firing + Explosion 
    if(ATATShot1 == false){
      if (currentMillis - previousATATShotMillis >= rndmATATShotMillis) {  // Basically means every rndmATATShotMillis this will happen. 
        previousATATShotMillis = currentMillis;
        wTrig.trackPlayPoly(4);
        SoftPWMSet(hlc0, 120);
        ATATShot1 = true;
      }
    }
    if (currentMillis - previousATATShotMillis >= ATATShotLength) SoftPWMSet(hlc0, 0);
    if (ATATShot1 == true && ATATShot2 == false){
      if (currentMillis - previousATATShotMillis >= rndmATATSpacerMillis) {  // Basically means every rndmATATShotMillis this will happen. 
        previousATATShotMillis = currentMillis;   
        wTrig.trackPlayPoly(4);
        SoftPWMSet(hlc1, 120);  
        ATATShot2 = true;
      } 
    }
    if (currentMillis - previousATATShotMillis >= ATATShotLength) SoftPWMSet(hlc1, 0);
    if ((ATATShot1 == true) && (ATATShot2 == true)){ 
      if (currentMillis - previousATATShotMillis >= rndmATATExplosionMillis) {  // Basically means every rndmATATShotMillis this will happen. 
        if (explodeOrNot > 4){ // Above 4 = explode
          previousATATShotMillis = currentMillis;  
          // Explosions = Files 9 - 13
          int x = random(9,14);
          wTrig.trackPlayPoly(x);
          SoftPWMSet(explosion, 200);  
        }
        rndmATATShot = false;
        rndmATATExplosion = false;
        rndmATATShotSpacer = false;
        ATATShot1 = false;      
        ATATShot2 = false;
      }
    }
    if (currentMillis - previousATATShotMillis >= ATATShotLength) SoftPWMSet(explosion, 0);
    // END - AT AT Firing + Explosion

    // START - Speeder flying and firing
    if(speeder1 == false){
      if (currentMillis - previousSpeederMillis >= rndmSpeederMillis) {  // Basically means every rndmSpeederMillis this will happen. 
        previousSpeederMillis = currentMillis; 
        // Snowspeeder flyby = Files 4 - 7
        int y = random (5,8); 
        wTrig.trackPlayPoly(y); 
        speeder1 = true;
      }
    }
    if (speeder2 == false && speeder1 == true){ 
      if (oneOrTwo > 12){  
        if (currentMillis - previousSpeederMillis >= rndmSpeederSpacerMillis) {  
          previousSpeederMillis = currentMillis;
          // Snowspeeder flyby = Files 4 - 7
          int z = random (5,8);          
          wTrig.trackPlayPoly(z); 
          speeder2 = true;
        } 
      }else{
        speeder2 = true;
      }
    }
    if (speeder1 == true && speeder2 == true){
      if (currentMillis - previousSpeederMillis >= rndmSpeederShotMillis) {  
        if (shootOrNot <= 17){
          previousSpeederMillis = currentMillis;
          while(speederShot<numberOfShots){
            currentMillis = millis();       // Get time while inside the while loop.. 
            if ((currentMillis - previousSpeederMillis >= rnmdSpeederShotSpacerMillis) && (fired == 0)) {
              previousSpeederMillis = currentMillis;
              wTrig.trackPlayPoly(8);
              if ((speederShot % 2) == 0) SoftPWMSet(snowspeederCannon0, 120); // Even
              if (speederShot % 2) SoftPWMSet(snowspeederCannon1, 120); // Odd
              rnmdSpeederShotSpacerMillis = random(100,300);
              fired = 1;    
            }
            if ((currentMillis - previousSpeederMillis >= 20) && (fired == 1)) {
               if ((speederShot % 2) == 0) SoftPWMSet(snowspeederCannon0, 0);   
               if (speederShot % 2) SoftPWMSet(snowspeederCannon1, 0); 
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
  }else if (paused == true) {
    // do nothing..we are holding.. 
  }
}