// AirHarp OS 3
// written in Arduino C by Pete Laric
// http://www.PeteLaric.com

/*
  NEW IN 3.11 Teensy:
  
  * experimented with pulseIn() rather than analogRead().  it's too slow.
  
  NEW IN 3.10 Teensy:
  
  * shoehorned code into running on Teensy 2.0-based AirHarp Pico.
  * code is now forked.

  NEW IN 3.09:

  * Deactivated patch change mode because it was causing stuck notes in Apple Logic.

  NEW IN 3.08:
  
  * Activation of "transpose down" button (added in AirHarp Lira - now the flagship model).
  
  NEW IN 3.06:
  
  * anti-jitter code for AirHarp creates sensorValuesOld[] table; requires substantial change to trigger note.

  NEW IN 3.05:
  
  * added support for new AirHarp Duo.
  * phasing out compile time switches in favor of separate functions for separate instruments.
  
  NEW IN 3.04:
  
  * the new Arduino 1.0 dropped support for the "byte" keyword, so byte variables
    had to be changed to "unsigned char" to compile under the new IDE.

  NEW IN 3.02:
  
  * expanded scale[] to include 2 octaves, thereby obviating the former wraparound,
    which caused chords 4-7 to experience undesired inversions.
    NUMSCALETONES now = 14 (formerly 7), and scale[] now contains 14 elements.

  NEW IN 3.01:
  
  * added device profile for HaloHarp to work with the new, re-imagined version of this exciting instrument!
    details to follow...  ;?)

  NEW IN 3.00:

  * various optimizations to minimize AirHarp strum jitter.  a good number to use for stringSpacing is 4.  no more averaging!

  NEW IN 2.99:
  
    * support for driverless interface with Lyrasynth cross-platform software synthesizer (does NOT require USB MIDI hack!)
    * automatic switching of majorSevenMode (AirHarp only).

  NEW IN 2.98:
  
    * beaconMode added for new AirHarp USB (first commercial AirHarp!)  the AirHarp Shield v1.2 now features an LED
    (with series resistor) between pins D12 & D13.  the LED's negative pin goes to 12 and its positive goes to 13.
    thus, the direction of current flow could be reversed in firmware to support a bi-color LED.

  NEW IN 2.97:
  
    * modified transpose button code to work with 5-button AirHarps.

  NEW IN 2.96:
  
    * alwaysTriggeredMode allows diagnostic use of an AirHarp lacking a rangefinder.
    
  NEW IN 2.95:
  
    * velocity keyed to strum speed (didn't like this mode as much as i thought i would).
    * multichannel MIDI output, w/ chordChannel playing fixed, root-position chords and strumChannel playing monophonic strum tones.

  NEW IN 2.94:
  
    * published to AirHarp.com for the first time.

  NEW IN 2.93:
  
    * added support for selecting MIDI channel (limited to 1 channel only, since only 1 midi velocities array exists).
    
  NEW IN 2.91:
  
    * gutted unused code to make LHOS smaller.
    * removed servo support.
    * discovered that aforementioned autoDetectUSB code was in fact executing, but serial debugging wasn't working
      because the debug data rate hadn't been set yet.  doh!
    * decided that autoDetectUSB was more trouble than it's worth from a hardware perspective.  i'll leave the code dormant for now.
    * Mr. K informed me that i could recompile the USB MIDI driver with a 31250 data rate.  problem solved.
    * PRINTED FOR HARD COPY REFERENCE!

  NEW IN 2.90:
  
    * numerous optimizations for the AirHarp; specifically, for the AirHarp v4 with integral synthesis.
    * MIDI program changes are now possible with the AirHarp by using the "modifier" button alone.
    * autoDetectUSB: when active, monitors supply voltage via a dual 22k voltage divider on pin A7.  sets USB_MIDI high if below a threshold.
    * PROBLEM: sketch size (16024 of 30720 max) is likely culprit for lack of runtime code execution; specifically, autoDetectUSB feature.

  NEW IN 2.88:
  
    * better handling of debugMode (formerly "serialDebugOn"/"midiOutputOn").
    * added majorSevenMode (for Lascia ch'io pianga).  this converts the vii* chord to VII.

  NEW IN 2.87:
  
    * bug in AutoHarpMode's handing of moodInvertMode fixed.

  NEW IN 2.86:
  
    * all possible device-specific variables have been converted from #defines to integers.
    * device profiles are now configured by functions, selected by the DEVICE_PROFILE var.

  NEW IN 2.85:
    
    * MAJOR BUG FIX!!!! [in proxDetect()]
    * support for ultrasound AirHarps.

  NEW IN 2.83:

    * transpose on button press.
    * optimized to work with infrared backscatter (no major modifications to firmware required).

  NEW IN 2.82:

    * servo sweep lidar (single detector, single laser).
 
  NEW IN 2.8:
 
    * USB MIDI output thanks to the genius of MICHAEL KRZYZANIAK!!!  :D

  NEW IN 2.7:

    * rapid arpeggiation pseudo-polyphony (for piezo output).
    * multi-pass proximity detection for increased precision (allows threshold margins to be set lower, increasing sensitivity).
    * support for passive shadow harps such as the pinhole-based PocketHarp.

  NEW IN 2.6:

    * monophonic n-length arpeggiator.
    * piezo sound frequencies now derived from MIDI note numbers; allows transposition & arpeggiation to affect piezo output.

  TO DO LIST:
    * un-double midiPitchTable[]

*/


#include <stdio.h>
#include <math.h>

// device profile names
#define STICKHARP_PRO                 0
#define STICKHARP                     1
#define STICKHARP_MINI                2
#define SHADOW_HARP                   3
#define ULTRASOUND_AIRHARP            4
#define COMBO_AIRHARP                 5
#define HALOHARP                      6

// device profile
//int DEVICE_PROFILE = STICKHARP_PRO;
int DEVICE_PROFILE = COMBO_AIRHARP;
//int DEVICE_PROFILE = HALOHARP;

#define PRODUCT_ID 0xF0, 0x7D, 'H', 'a', 'l', 'o', ' ', 'H', 'a', 'r', 'p', 0xF7

#define MAXSTRINGS                    8
#define MAXSCALETONES                 7
#define MAXBUTTONS                    8
#define MAX_MIDI_CHANNELS             2

#define MIDI        0
#define USB_MIDI    1
#define LYRASYNTH   2

// ultrasonic rangefinder types
#define MAXBOTIX_LVEZ  0
#define MAXBOTIX_HRLV  1
#define PARALLAX_PING  2
#define SHARP_IR       3

// device profile vars - filled on setup by the appropriate function
int NUMSTRINGS,
    NUM_MIDI_CHANNELS = MAX_MIDI_CHANNELS,
    NUMSCALETONES,
    NUMBUTTONS,
    midiOutputType,
    autoDetectUSB,
    ACTIVE_HARP,
    ANALOG_SENSORS,
    PWM_SENSORS,
    TRIGGER_ON_POSITIVE,
    DELAY_BEFORE_AUTOCALIBRATION,
    RECALIBRATE_EVERY_SCAN,
    ULTRASOUND_HARP,
    ULTRASOUND_SENSOR_TYPE,
    ULTRASOUND_RANGE_THRESHOLD,
    ULTRASOUND_INSENSITIVITY_REGION, // Maxbotix rangefinder will not return a value less than this (first ~5-6 inches return this value)
    VISIBLE_INDICATORS,
    PROPORTIONAL_DYNAMICS,
    defaultMidiVelocity = 127,
    THEREMIN_MODE,
    NONLINEAR_MODE,
    AutoHarpMode,  // arpeggiate chords according to analog input (for AirHarp).
    AutoChordMode, // play a chord (or at least a chord root) when a note is triggered.  use with AutoHarpMode is optional.
    RandomArpMode, // choose a chord component randomly, but allow proximity-based octave and velocity selection.
    AirHarpStringSpacing,
    jitterCorrect,
    alwaysTriggeredMode,
    beaconMode,
    chordChannel,
    strumChannel,
    defaultChordPatch,
    defaultStrumPatch,
    currentChordPatch,
    currentStrumPatch,
    InsaneMode = 0,
    AutoFadeNotes = 0,
    logFade,
    fadeDecrement, // for AutoFadeNotes
    SCAN_ITERATIONS,  // number of passes to average for proxDetect() scan
    CALIBRATE_ITERATIONS,
    cycleDelay,
    capChargeDelay,
    SoundOutPinRestState,
    midiPitchMin = 21,
    midiPitchMax = 108,
    midiSendDelay = 20, // wait this many ms after sending any MIDI message (default=20)
    delayAfterRangingMicroseconds = 10;
int sensorPins[MAXSTRINGS];
int lampPins[MAXSTRINGS];
int buttonPins[MAXBUTTONS];
int buttonStates[MAXBUTTONS];
int buttonStatesOld[MAXBUTTONS];
float ThresholdCoefficient,
      ThresholdOffset;



void configureAirHarp()
{
  NUMSTRINGS = 1;
  NUM_MIDI_CHANNELS = MAX_MIDI_CHANNELS;
  midiOutputType = MIDI; // MIDI, USB_MIDI or LYRASYNTH
  autoDetectUSB = 0;
  chordChannel = 1;
  strumChannel = 0;
  defaultChordPatch = 92; // 92=bowed pad
  defaultStrumPatch = 46; // 46=Orchestral Harp; 8-15=Chromatic Perc
  defaultMidiVelocity = 117;
  NUMSCALETONES = 14;
  NUMBUTTONS = 6;
  ACTIVE_HARP = 0; // 1 for active IR; 0 for Maxbotix ultrasound
  ANALOG_SENSORS = 1;
  PWM_SENSORS = 0;
  TRIGGER_ON_POSITIVE = 0;
  DELAY_BEFORE_AUTOCALIBRATION = 0;
  RECALIBRATE_EVERY_SCAN = 0;
  ULTRASOUND_HARP = 1;
  ULTRASOUND_SENSOR_TYPE = MAXBOTIX_LVEZ;
  //ULTRASOUND_SENSOR_TYPE = SHARP_IR;
  //ULTRASOUND_SENSOR_TYPE = MAXBOTIX_HRLV;
  if (ULTRASOUND_SENSOR_TYPE == SHARP_IR)
  {
    ULTRASOUND_RANGE_THRESHOLD = 850; // 48 = 24" (2 ft.) for LV-EZ; 157 for HRLV
    ULTRASOUND_INSENSITIVITY_REGION = 500; // 11 is the standard value for LV-EZ1 !!!  (57 is standard for HRLV)
    AirHarpStringSpacing = 20; // 4 is the standard value!
  }    
  else if (ULTRASOUND_SENSOR_TYPE == MAXBOTIX_HRLV)
  {
    ULTRASOUND_RANGE_THRESHOLD = 157; // 48 = 24" (2 ft.) for LV-EZ; 157 for HRLV
    ULTRASOUND_INSENSITIVITY_REGION = 57; // 11 is the standard value for LV-EZ1 !!!  (57 is standard for HRLV)
    AirHarpStringSpacing = 4; // 4 is the standard value!
  }
  else
  {
    ULTRASOUND_RANGE_THRESHOLD = 56; // 48 = 24" (2 ft.) for LV-EZ; 157 for HRLV
    ULTRASOUND_INSENSITIVITY_REGION = 11; // 11 is the standard value for LV-EZ1 !!!  (57 is standard for HRLV)
    AirHarpStringSpacing = 4; // 4 is the standard value!
  }
  jitterCorrect = 1; // try to correct jitter?
  VISIBLE_INDICATORS = 0;
  PROPORTIONAL_DYNAMICS = 0;
  InsaneMode = 0;
  THEREMIN_MODE = 0;
  NONLINEAR_MODE = 0;
  SCAN_ITERATIONS = 3; // number of passes to average for proxDetect() scan
  CALIBRATE_ITERATIONS = 0;
  AutoHarpMode = 1; // play individual note in current chord based on proximity (set to '1' for Combo AirHarp)
  AutoChordMode = 0; // play constant chords
  RandomArpMode = 0;
  alwaysTriggeredMode = 0;
  beaconMode = 1;
  AutoFadeNotes = 0; // fade each note at a predefined rate; notes decay automatically independent of performance input
  logFade = 0; // fade logarithmically (divide by 2 every iteration)
  fadeDecrement = 1; // for AutoFadeNotes
  cycleDelay = 0; // 20
  capChargeDelay = 0;
  sensorPins[0] = 0;
  sensorPins[1] = 0;
  sensorPins[2] = 0;
  sensorPins[3] = 0;
  sensorPins[4] = 0;
  sensorPins[5] = 0;
  sensorPins[6] = 0;
  sensorPins[7] = 0;
  lampPins[0] = 9;
  lampPins[1] = 9;
  lampPins[2] = 9;
  lampPins[3] = 9;
  lampPins[4] = 9;
  lampPins[5] = 9;
  lampPins[6] = 9;
  lampPins[7] = 9;
  buttonPins[0] = 6; // transpose up button
  buttonPins[5] = 7; // transpose down button
  buttonPins[1] = 2; // chord select buttons
  buttonPins[2] = 3; // "
  buttonPins[3] = 4; // "
  buttonPins[4] = 5; // pinkie button (flips chord mood)
  SoundOutPinRestState = 0;
  ThresholdCoefficient = 0;
  ThresholdOffset = 0;
}



// major keys (for auto-transposition)
#define KEY_OF_C        0
#define KEY_OF_C_SHARP  1
#define KEY_OF_D_FLAT   1
#define KEY_OF_D        2
#define KEY_OF_D_SHARP  3
#define KEY_OF_E_FLAT   3
#define KEY_OF_E        4
#define KEY_OF_F        5
#define KEY_OF_F_SHARP  6
#define KEY_OF_G_FLAT   6
#define KEY_OF_G        7
#define KEY_OF_G_SHARP  8
#define KEY_OF_A_FLAT   8
#define KEY_OF_A        9
#define KEY_OF_A_SHARP  10
#define KEY_OF_B_FLAT   10
#define KEY_OF_B        11

#define DEFAULT_MIDI_TONIC    60 // 60 = middle C


// I/O pins
int SoundOutPin = 13, // attach piezo speaker to this pin
    supplyVoltageMonitorPin = A7,
    pwmPin = 10; // for reading ultrasound with pulseIn()
// note: the MIDI output pin is, by necessity, pin 1 (a.k.a. "TX1"),
// because this is the hardwired serial transmit pin on the Arduino Nano.
// (use whichever pin says "TX" for your MIDI output).

// global vars
int midiNote,
    TRANSPOSITION_OFFSET = KEY_OF_C,
    midiTonic = DEFAULT_MIDI_TONIC + TRANSPOSITION_OFFSET,
    PiezoSoundOn = 0,
    midiDebugOn = 0,
    debugMode = 0,
    ChordTriggerMode = 1,
    modeSwitchable = 0,
    aftertouchOn = 0,
    arpState = 0,
    moodInvertMode = 0,
    majorSevenMode = 0, // turns the 7 chord from vii* (diminished) to VII (Major)
    piezoDelay = 50, //50
    standardDelay = 10, // delay per scan cycle (post-scan)
    longDelay = 100;

int scale[] = { 0, 2, 4, 5, 7, 9, 11, 12, 14, 16, 17, 19, 21, 23 };  // scale mode expressed in semitone offsets from tonic
int invertTable[] = { -1, 1, 1, -1, -1, 1, 1 }; // semitone offsets to invert mood of thirds
char midiVelocities[MAX_MIDI_CHANNELS][128];
char midiVelocitiesOld[MAX_MIDI_CHANNELS][128];

float thresholds[MAXSTRINGS];
int triggered[MAXSTRINGS];
float sensorValues[MAXSTRINGS],
      sensorValuesOld[MAXSTRINGS],
      lampDarkVals[MAXSTRINGS],
      lampLitVals[MAXSTRINGS];

unsigned int patchButtonPressDuration = 0,
             patchButtonPressThreshold = 100000; // continuous cycles before button hold results in patch change
                                                 // 100 was good value to activate; set to 100,000 to effectively deactivate patch changes

int arpLen = 16;
int arpPattern[] = { 1, 1, 3, 3, 5, 5, 1, 1, 3, 3, 5, 5, 1, 1, 3, 3 // "Anticipation"
                     //1, 3, 5, 1, 3, 5, 1, 3 // "What I Could Never Say"
                   }; 


// pitch formula: f = 2 ^ (tone / 12) * 440
// scale pitches, calculated by me:
float pitchTable[12] = { 440.000000, // A
                         466.163757, // A#/Bb
                         493.883301, // B
                         523.251160, // C
                         554.365234, // C#/Db
                         587.329529, // D
                         622.253967, // D#/Eb
                         659.255127, // E
                         698.456482, // F
                         739.988831, // F#/Gb
                         783.990845, // G
                         830.609375 // G#/Ab
                       };

unsigned int midiPitchTable[128]; // was "double"


void setup()
{
  analogReference(DEFAULT);
  //pinMode(SoundOutPin, OUTPUT);
  //digitalWrite(SoundOutPin, SoundOutPinRestState);  // shut up!  :?P
  
  // configure device-specific variables for appropriate device profile
  if (DEVICE_PROFILE == COMBO_AIRHARP) configureAirHarp();
  
  if (autoDetectUSB) detectUSBbyInputVoltage();

  if (debugMode)
  {
    Serial.begin(9600);
  }
  /*
  else if (midiOutputType == USB_MIDI)
  {
    //Serial.begin(115200);
    Serial.begin(31250);
    const uint8_t productID[] = {
      PRODUCT_ID    };
    Serial.write(productID, sizeof(productID));
  }
  else if (midiOutputType == LYRASYNTH)
  {
    Serial.begin(9600);
  }
  else Serial.begin(31250); //  Set MIDI baud rate:
  */

  if (!ULTRASOUND_HARP) delay(DELAY_BEFORE_AUTOCALIBRATION); // gives performer time to stand clear
  
  // autocalibrate
  autoCalibrate();
  
  zeroMidiVelocities();
  copyNewToOld(); // zeroes midiVelocitiesOld as well

  currentChordPatch = defaultChordPatch;

  currentStrumPatch = defaultStrumPatch;
  
  if (beaconMode) activateBeaconMode();
}


void loop()
{
  //allLampsOn(); // photogenic mode
  lightHarp();
  //airHarpDuo();
}


// turns on the device's LED
void activateBeaconMode()
{
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  digitalWrite(8, HIGH);
  digitalWrite(9, LOW);
}


void calibrate(unsigned int i)  // i = light string #
{
  // poll sensors by method appropriate to sensor type;
  // store values in sensorValues[] and thresholds[]
  triggered[i] = 0;
  
  float totalValue = 0,
  maxValue = 0,
  minValue = 1000000;   
  for (unsigned int index = 0; index < CALIBRATE_ITERATIONS; index++)
  {
    float value = 0;
    //if (ACTIVE_HARP) value = proxDetect(i);
    //else value = passiveDetect(i);
    value = proxDetect(i);
    totalValue += value;
    if (value > maxValue) maxValue = value;
    if (value < minValue) minValue = value;
  }
  float avgValue = totalValue / CALIBRATE_ITERATIONS;    
  if (debugMode)
  {
    Serial.print(" min: ");
    Serial.print(minValue);
    Serial.print("; max: ");
    Serial.print(maxValue);
    Serial.print("; avg: "); 
    Serial.print(avgValue);
    Serial.println();
  }
  sensorValues[i] = avgValue;
  if (ACTIVE_HARP) // always triggers on a positive differential (though directionality of differntial does matter)
  {
    thresholds[i] = maxValue; // set the treshold to the highest value obtained during calibration
    ///////////////thresholds[i] = lampLitVals[i]; // if higher-than-ambient rather than differential triggering
    ThresholdOffset = thresholds[i] * ThresholdCoefficient; //diff(avgValue, maxValue); // now add a little margin to prevent spurrious false positives
  }
  else if (TRIGGER_ON_POSITIVE)  // passive harp that triggers when sensors read high
  {
    thresholds[i] = maxValue; // set the treshold to the highest value obtained during calibration
    ThresholdOffset = ThresholdCoefficient * thresholds[i]; // now add a little margin to prevent spurrious false positives
  }
  else if (!ULTRASOUND_HARP) // passive harp that triggers when sensors read low
  {
    thresholds[i] = minValue; // set the treshold to the lowest value obtained during calibration    
    ThresholdOffset = ThresholdCoefficient * thresholds[i]; // now subtract a little margin to prevent spurrious false positives
  }
}


void autoCalibrate()
{
  for (int i = 0; i < NUMSTRINGS; i++)
  {     
    if (ULTRASOUND_HARP) thresholds[i] = ULTRASOUND_RANGE_THRESHOLD;
    else calibrate(i);
    //delay(standardDelay);
  }
  if (debugMode) Serial.println();
}

void recalibrate()
{
  // all this does is set the thresholds to the current light levels
  // it doesn't perform a new scan - it just uses the data from the last scan
  for (int i = 0; i < NUMSTRINGS; i++)
  {     
    //triggered[i] = 0;
    //if (ACTIVE_HARP) sensorValues[i] = proxDetect(i);
    //else sensorValues[i] = passiveDetect(i);
    thresholds[i] = sensorValues[i];
    //delay(standardDelay);
  }
  //if (debugMode) Serial.println();
}



void lightHarp()
{
  scanLightStrings(); // does all proximity detection; determines whether notes have been triggered
  
  zeroMidiVelocities();
  
  int holdThisNote = 0;
  
  //float sensorValuesOld[MAXSTRINGS];
  
  scanButtons();
  
  // transpose button stuff
  int triggerOnButtonState = 0;
  
  // transpose up
  if (buttonStates[0] != buttonStatesOld[0])
  {
    if (debugMode)
    {
      Serial.print("buttonStates[0]: ");
      Serial.print(buttonStates[0]);
      Serial.print(" ");
    }
    if (buttonStates[0] == triggerOnButtonState)
    {
      if (!debugMode) killAllMidi(); // prevents stuck notes
      TRANSPOSITION_OFFSET++;
      //if (TRANSPOSITION_OFFSET > midiPitchMax) TRANSPOSITION_OFFSET = midiPitchMax;
      midiTonic = DEFAULT_MIDI_TONIC + TRANSPOSITION_OFFSET;
      if (debugMode)
      {
        Serial.print("transposing +");
        Serial.print(TRANSPOSITION_OFFSET);
      }
    }
    if (debugMode) Serial.println();
  }
  
  // transpose down
  if (buttonStates[5] != buttonStatesOld[5])
  {
    if (debugMode)
    {
      Serial.print("buttonStates[5]: ");
      Serial.print(buttonStates[5]);
      Serial.print(" ");
    }
    if (buttonStates[5] == triggerOnButtonState)
    {
      if (!debugMode) killAllMidi(); // prevents stuck notes
      TRANSPOSITION_OFFSET--;
      //if (TRANSPOSITION_OFFSET < midiPitchMin) TRANSPOSITION_OFFSET = midiPitchMin;
      midiTonic = DEFAULT_MIDI_TONIC + TRANSPOSITION_OFFSET;
      if (debugMode)
      {
        Serial.print("transposing +");
        Serial.print(TRANSPOSITION_OFFSET);
      }
    }
    if (debugMode) Serial.println();
  }
  
  if (buttonStates[0] == triggerOnButtonState &&
      buttonStates[5] == triggerOnButtonState)
      {
        // both transpose buttons are being depressed simultaneously
        // reset transpose
        TRANSPOSITION_OFFSET = 0;
        midiTonic = DEFAULT_MIDI_TONIC + TRANSPOSITION_OFFSET;
      }
  

  // now act on data gathered during scan

  // check for control combination
  if (controlCombo() && modeSwitchable) toggleMode(); // switch between chord trigger, arpeggiator and single note modes

  // check for modulation cue
  if (ChordTriggerMode && modulationCue()) activateMoodInvert();
  else deactivateMoodInvert();

  //if (!AutoFadeNotes) zeroMidiVelocities();

  for (int i = 0; i < NUMSTRINGS; i++)
  {
    unsigned int velocity;
    int scaleNote = i;

    if (triggered[i])
    {
      /*
      if (THEREMIN_MODE)
      {
        int inches_per_segment = 2;
        int increments_per_inch = 2; // approximate value for Maxbotix LV-EZ4
        int increments_per_segment = inches_per_segment * increments_per_inch;
        if (NONLINEAR_MODE) scaleNote = int(sqrt(sensorValues[i])) % 7;
        else scaleNote = (int)((sensorValues[i] - (float)ULTRASOUND_INSENSITIVITY_REGION) / (float)increments_per_segment);
        /////else scaleNote = int(sensorValues[i] - ULTRASOUND_INSENSITIVITY_REGION) / increments_per_segment;
        int octave_offset = 0;
        while (scaleNote >= 7)
        {
          scaleNote -= 7;
          octave_offset++;
        }
        midiNote = midiTonic + scale[scaleNote]; // + (octave_offset * 12);
        if (debugMode)
        {
          Serial.print("scaleNote = ");
          Serial.print(scaleNote);
          Serial.print("; ");
        }
      }
      */
      //else
      if (DEVICE_PROFILE == COMBO_AIRHARP)
      {
        THEREMIN_MODE = 0; // reset this every time!
        
        // decode button combination; determine scale tone
        int digits[4];
        digits[0] = 1 - buttonStates[1]; // flip from "active low" to "active high"
        digits[1] = 1 - buttonStates[2]; // (makes subsequent math more intuitive)
        digits[2] = 1 - buttonStates[3];
        digits[3] = 1 - buttonStates[4];
        scaleNote = (digits[0] * 4) + (digits[1] * 2) + digits[2];
        majorSevenMode = 0;
        if (digits[3]) // mood flip button
        {
          if (scaleNote == 7) majorSevenMode = 1; // you're pressing all 4 chord buttons; make a VII chord (special case)
          else activateMoodInvert(); // not a VII chord; just flip the mood
        }
        else deactivateMoodInvert(); // blue (mood) button not pressed
        if (scaleNote)
        {
          scaleNote--; // convert to zero-indexed
          midiNote = midiTonic + scale[scaleNote];
        }
        else // none of the 3 main chord buttons are depressed
        {
          triggered[i] = 0;
          if (digits[3]) // modifier button alone is being pressed
          {
            // EXPERIMENTAL theremin mode!
            triggered[i] = 1;
            THEREMIN_MODE = 1;
            // patch change code
            //patchButtonPressDuration++;
            /*
            if (patchButtonPressDuration >= patchButtonPressThreshold)
            {
              currentStrumPatch++;
              if (currentStrumPatch > 127) currentStrumPatch = 0;
              // the following bit is so the MIDI handling code recognizes the test tone as a new note
              killAllMidi();
              zeroMidiVelocities();
              copyNewToOld(); // zeroes midiVelocitiesOld as well

              triggered[i] = 1;
              sensorValues[i] = 1;
              midiNote = midiTonic + scale[scaleNote];
              //patchButtonPressDuration = 0;
              holdThisNote = 1;
            }
            */
          }
          //else patchButtonPressDuration = 0; // no mod button, so start counter over
        }
      }
      else midiNote = midiTonic + scale[i]; // assign note (or chord root) according to light string # (default)
      
      // the above determined midi note is the root of our chord
      int rootNote = midiNote;
      // harmonize 3rd and 5th notes using current scale, accounting for possible half-step modulation of third
      int thirdNote = midiTonic + scale[((scaleNote+2) %NUMSCALETONES)];
      if (majorSevenMode && scaleNote == 6) thirdNote++; // change minor third to major third
      if (moodInvertMode) thirdNote += invertTable[scaleNote]; // invert chord mood if necessary
      int fifthNote = midiTonic + scale[((scaleNote+4) %NUMSCALETONES)];
      if (majorSevenMode && scaleNote == 6) fifthNote++; // unflat the fifth

      if (PROPORTIONAL_DYNAMICS)
      {
        //velocity = sqrt(sensorValues[i]);
        // assume ultrasound
        /*
        float range = sensorValues[i] - 12;
        if (range > 0) velocity = ((1 / range) * 64) + 64;
        else velocity = 127;
        if (velocity > 127) velocity = 127; // limit range
        */
        // attenuate velocity with distance
        int range = sensorValues[i] - ULTRASOUND_INSENSITIVITY_REGION;
        if (range < 0) range = 0;
        velocity = 117 - (range * 3);
        /*
        // velocity proportional to strum          speed
        float strumVelocity = diff(sensorValues[i], sensorValuesOld[i]);
        if (sensorValuesOld[i] == 0 || sensorValuesOld[i] > ULTRASOUND_RANGE_THRESHOLD) strumVelocity = 1; // reject extreme readings due to initial beam reflection.
        velocity = 0 + (strumVelocity * 32);
        if (velocity > 127) velocity = 127;
        sensorValuesOld[i] = sensorValues[i];
        */
      }
      else
        velocity = defaultMidiVelocity;

      if (InsaneMode) // select one note to play this time around - next time, different note.
      {
        int r = rand() %3;
        if (r == 0) midiNote = rootNote;
        else if (r == 1) midiNote = thirdNote;
        else midiNote = fifthNote;
      }
      else if (AutoHarpMode) // choose appropriate note to play based on proximity
      {
        // the root note (midiNote) has already been established.  this routine
        // merely computes an offset;
        //unsigned int distVal = sensorValues[i] / 10;
        unsigned int distVal = (sensorValues[i] - ULTRASOUND_INSENSITIVITY_REGION) / AirHarpStringSpacing;  // 2 // 8 for debut performance videos.
        if (THEREMIN_MODE)
        {
          //int octave_offset = 12 * (distVal / 3);
          int octave_offset = 12 * (distVal / NUMSCALETONES);
          midiNote = midiTonic + scale[distVal % NUMSCALETONES];
          midiNote += octave_offset;
        }
        else
        {
          int chordComponent = distVal % 3; // this will be 0, 1 or 2 - representing the tonic, third and fifth respectively
          if (RandomArpMode) chordComponent = rand() % 3;
          int octave_offset = 12 * (distVal / 3);
          //scaleNote = i + arpPattern[arpState] - 1;
          //int ccScaleNote = scaleNote; // chord component scale note - if "scaleNote" is the root, this will either be the root, the third of the fifth
          if (chordComponent == 0) midiNote = rootNote; // still tonic
          else if (chordComponent == 1) midiNote = thirdNote; // play third
          else midiNote = fifthNote; // play fifth
          midiNote += octave_offset;
        }
      }

      //int piezoPitch = midiPitchTable[midiNote];

      if (triggered[i])
      {
        if (AutoHarpMode) noteOn(strumChannel, midiNote, velocity);
        
        if (AutoChordMode)
        {
          int chordVelocity = 77;
          
          ////if (PROPORTIONAL_DYNAMICS) chordVelocity = velocity;
  
          if (!holdThisNote) noteOn(chordChannel, rootNote, chordVelocity); // play root (or, if InsaneMode/AutoHarpMode, play one of the chord's component notes).
  
          if (ChordTriggerMode && !InsaneMode && !holdThisNote)
          {
            // play 3rd
            noteOn(chordChannel, thirdNote, chordVelocity);
            // play 5th
            noteOn(chordChannel, fifthNote, chordVelocity);
          }
        }
        
        if (PiezoSoundOn)
        {
          freqout(SoundOutPin, midiPitchTable[midiNote], piezoDelay);
          //synthesizeMidi();
          //delay(piezoDelay);
        }
        else if (InsaneMode) delay(piezoDelay);  
      }

    } // end if(triggered[i])

  } //end for (int i = 0; i < NUMSTRINGS; i++)

  if (!debugMode)
    refreshMidiOutput();
  else if (midiDebugOn)
  {
    debugMidiStream();
    Serial.println();
  }
  
  if (holdThisNote) delay(500);
  else delay(cycleDelay);

  if (RECALIBRATE_EVERY_SCAN) recalibrate();
  if (AutoFadeNotes) fadeAllActiveNotes();

  //randomizeMidi();

} // end loop()


void detectUSBbyInputVoltage()
{
  pinMode(supplyVoltageMonitorPin, INPUT);
  int inputReading = analogRead(A7);
  float inputVoltage = float(inputReading) / 1024 * 5;
  inputVoltage *= 2; // double to correct for voltage divider stepdown
  float USBvoltageThreshold = 7;
  if (inputVoltage < USBvoltageThreshold) midiOutputType = USB_MIDI;
  else midiOutputType = MIDI;
  // debug stuff
  if (debugMode)
  {
    Serial.begin(9600);
    Serial.println();
    Serial.print("supply voltage: ");
    Serial.println(inputVoltage);
    Serial.print("voltage threshold: ");
    Serial.println(USBvoltageThreshold);
    Serial.print("serial mode: ");
    if (midiOutputType == USB_MIDI)
      Serial.println("USB_MIDI");
    else
      Serial.println("Standard MIDI");
  }
}


void scanButtons()
{
  for (unsigned int i = 0; i < NUMBUTTONS; i++)
  {
    pinMode(buttonPins[i], INPUT);
    //delay(1);
    digitalWrite(buttonPins[i], HIGH);
    int buttonState = digitalRead(buttonPins[i]);
    buttonStatesOld[i] = buttonStates[i];
    buttonStates[i] = buttonState;
    //////buttonStates[i] = digitalRead(buttonPins[i]);
  }
}



void randomizeMidi()
{
  for (unsigned int i = 0; i < NUM_MIDI_CHANNELS; i++)
  {
    randomizeMidiOnChannel(i);
  }
}

void randomizeMidiOnChannel(int channel)
{
  for (unsigned int i = 0; i < 128; i++)
  {
    if (rand() %100 < 1)
      midiVelocities[channel][i] = rand() %128;
  }
}



void zeroMidiVelocities()
{
  for (unsigned int i = 0; i < NUM_MIDI_CHANNELS; i++)
  {
    zeroMidiVelocitiesOnChannel(i);
  }
}


void zeroMidiVelocitiesOnChannel(int channel)
{
  for (unsigned int i = 0; i < 128; i++)
  {
    midiVelocities[channel][i] = 0;
  }
}


void fadeAllActiveNotes()
{
  for (unsigned int i = 0; i < NUM_MIDI_CHANNELS; i++)
  {
    fadeAllActiveNotesOnChannel(i);
  }
}


void fadeAllActiveNotesOnChannel(int channel)
{
  for (unsigned int i = 0; i < 128; i++)
  {
    if (midiVelocities[channel][i] > 0)
    {
      int velocity;
      if (logFade) velocity = midiVelocities[channel][i] / 2;
      else velocity = midiVelocities[channel][i] - fadeDecrement;
      if (velocity < 0) velocity = 0;
      midiVelocities[channel][i] = velocity;
    }
    //if (velocity > 0)
    //  midiVelocities[i]--;
    //midiSend(0x90 + channel, midiNote, velocity);
  }
}


void refreshMidiOutput()
{
  for (unsigned int i = 0; i < NUM_MIDI_CHANNELS; i++)
  {
    refreshMidiOutputOnChannel(i);
  }
}


void refreshMidiOutputOnChannel(int channel)
{
  for (unsigned int i = 0; i < 128; i++)
  {
    int pitch = i;
    int velocity = midiVelocities[channel][pitch];
    if (velocity == midiVelocitiesOld[channel][pitch])
    {
      // do nothing
    }
    else if (velocity == 0)
    {
      // this note needs to be turned off
      //midiSend(0x80 + channel, pitch, 64); // turn off at medium speed
      midiSend(0x90 + channel, pitch, 0); // "note on" with velocity=0 is equivalent to "note off"
    }
    else if (midiVelocitiesOld[channel][pitch] == 0 && velocity > 0)
    {
      // the new velocity is a positive value, but the note isn't on yet.  send a MIDI "note on" message
      midiSend(0x90 + channel, pitch, velocity);
    }
    else
    {
      // the only remaining possibility is that both the previously triggered note and the current note
      // are positive values, but the values don't match.  send a MIDI "aftertouch" message instead of
      // re-triggering the note.
      if (aftertouchOn)
      {
        midiSend(0xA0 + channel, pitch, velocity); // A0 = aftertouch (channel 0)
        //midiSend(0xD0 + channel, velocity, 0); // D0 = channel pressure (channel 0)
        // since the above doesn't seem to be doing anything (with Apple Logic as the softsynth), let's just restrike:
        //midiSend(0x90 + channel, pitch, velocity);
      }
    }
    midiVelocitiesOld[channel][i] = midiVelocities[channel][i];
    if (!AutoFadeNotes) midiVelocities[channel][i] = 0;
  }
  //copyNewToOld();
}


void copyNewToOld()
{
  for (unsigned int i = 0; i < NUM_MIDI_CHANNELS; i++)
  {
    copyNewToOldOnChannel(i);
  }
}


void copyNewToOldOnChannel(int channel)
{
  for (unsigned int i = 0; i < 128; i++)
  {
    midiVelocitiesOld[channel][i] = midiVelocities[channel][i];
  }
}


void debugMidiStream()
{
  for (unsigned int i = 0; i < NUM_MIDI_CHANNELS; i++)
  {
    debugMidiStreamOnChannel(i);
  }
}


void debugMidiStreamOnChannel(int channel)
{
  if (debugMode)
  {
    //Serial.println();
    Serial.print("  active MIDI notes: ");
    for (unsigned int i = 0; i < 128; i++)
    {
      if (midiVelocities[channel][i])
      {
        Serial.print(i);
        Serial.print(" ");
      }
    }
    Serial.println();
  }
}


int objectProximal(int i) // i = light string #
{
    if (ACTIVE_HARP)
    {
      if (TRIGGER_ON_POSITIVE)
      {
        if (lampLitVals[i] > lampDarkVals[i] && sensorValues[i] > (thresholds[i] + ThresholdOffset)) return 1;
        //////////////////if (lampLitVals[i] > lampDarkVals[i] && lampLitVals[i] > (thresholds[i] + ThresholdOffset)) return 1; // if higher-than-ambient rather than differential triggering
      }
      else
      {
        if (lampLitVals[i] < lampDarkVals[i] && sensorValues[i] > (thresholds[i] + ThresholdOffset)) return 1;
        //////////////////if (lampLitVals[i] < lampDarkVals[i] && lampLitVals[i] < (thresholds[i] - ThresholdOffset)) return 1; // if higher-than-ambient rather than differential triggering
      }
    }
    else // passive harp
    {
      if (TRIGGER_ON_POSITIVE)
      {
        if (sensorValues[i] > (thresholds[i] + ThresholdOffset)) return 1;
      }
      else
      {
        if (sensorValues[i] < (thresholds[i] - ThresholdOffset)) return 1;
      }
    }
    return 0;
}


void scanLightStrings()
{
  // main scan loop
  for (int i = 0; i < NUMSTRINGS; i++)
  {
    // now do prox detection or passive detection
    //if (ACTIVE_HARP) sensorValues[i] = proxDetect(i);
    //else sensorValues[i] = passiveDetect(i);
    sensorValues[i] = proxDetect(i);
    
    if (ULTRASOUND_SENSOR_TYPE == SHARP_IR) sensorValues[i] = 1023 - sensorValues[i];
    
    if (jitterCorrect)
    {
      int jitterBuffer = AirHarpStringSpacing * .75; //- 1; // 3; // in testing (with LV-EZ1), 3 is the minimum value required to eliminate jitter
      if (diff(sensorValues[i], sensorValuesOld[i]) <= jitterBuffer) sensorValues[i] = sensorValuesOld[i];
      else sensorValuesOld[i] = sensorValues[i];
    }
    
    if (objectProximal(i) || alwaysTriggeredMode)
    {
      triggered[i] = 1;
      if (VISIBLE_INDICATORS) digitalWrite(lampPins[i], HIGH); // "note on" indicator
    }
    else
    {
      triggered[i] = 0;
      //if (VISIBLE_INDICATORS) digitalWrite(lampPins[i], LOW); // "note off" indicator
    }
    
    if (debugMode)
    {
      //Serial.print(i);
      //Serial.print(": ");
      //Serial.print(lampDarkVals[i]);
      //Serial.print("-");
      //Serial.print(lampLitVals[i]);
      Serial.print("val: ");
      Serial.print(sensorValues[i]);
      Serial.print("; ");
      if (triggered[i])
      {
        Serial.print("TRIGGERED (threshold = ");
        Serial.print(thresholds[i]);
        Serial.print(")");
      }
      Serial.println();
      //else Serial.print("  ");
    }
  
  } // end main scan loop
  
  if (debugMode) Serial.println();
}


double proxDetect(unsigned int i) // i = light string #; returns backscatter level OR ambient light level
{     
  if (debugMode)
  {
    Serial.print(i);
    Serial.print(": ");
  }
  
  //allLampsOff();
  
  //int sensorPin = sensorPins[i];
  int lampPin = lampPins[i];
  if (ACTIVE_HARP || VISIBLE_INDICATORS) pinMode(lampPin, OUTPUT);

  double valueTotal = 0;

  for (unsigned int passes = 0; passes < SCAN_ITERATIONS; passes++)
  {
    if (ACTIVE_HARP)
    {
      // take reading with lamp on
      digitalWrite(lampPin, HIGH);
      delay(capChargeDelay); // capChargeDelay = zero for AirHarp
      lampLitVals[i] = passiveDetect(i);
      if (debugMode)
      {    
        Serial.print("l: ");
        Serial.print(lampLitVals[i]);
        Serial.print("; ");
      }
    }
    // take reading with lamp off
    if (ACTIVE_HARP || VISIBLE_INDICATORS) digitalWrite(lampPin, LOW);
    delay(capChargeDelay); // capChargeDelay = zero for AirHarp
    /////////////lampDarkVals[i] = analogRead(sensorPin);//rcTime(SensorPin);
    lampDarkVals[i] = passiveDetect(i);
    if (debugMode)
    {
      Serial.print("d: ");
      Serial.print(lampDarkVals[i]);
      Serial.print("; ");
    }
    unsigned long int value;
    if (ACTIVE_HARP) value = diff(lampDarkVals[i], lampLitVals[i]);
    else value = lampDarkVals[i];
    valueTotal += value;
  }
  
  /*
  if (VISIBLE_INDICATORS)
  {
    allTriggeredLampsOn();
    //allLampsOn();
    //allTriggeredLampsOff();
  }
  */
  
  double valueAverage;
  valueAverage = (double)valueTotal / (double)SCAN_ITERATIONS;

  //printf("%u : %u \n", i, difference);

  return valueAverage;
}


unsigned long int passiveDetect(unsigned int i) // i = light string #; returns passive light level
{     
  int sensorPin = sensorPins[i];
  unsigned long int lightLevel;

  if (ANALOG_SENSORS) // use the Arduino's analog inputs
  {
    lightLevel = analogRead(sensorPin);
    delayMicroseconds(delayAfterRangingMicroseconds);
  }
  else if (PWM_SENSORS)
  {
    long pulse1, sensor1;
    pulse1 = pulseIn(pwmPin, HIGH, 100000);
    sensor1 = pulse1 / 50; //pulse1/147;
    lightLevel = (unsigned int)sensor1;
  }
  else // use the Arduino's digital inputs
  {
    lightLevel = rcTime(sensorPin);
  }

  /*
     if (debugMode)
   {
   //Serial.print(i);
   //Serial.print(": ");
   Serial.print(lightLevel);
   if (triggered[i]) Serial.print("* ");
   else Serial.print("  ");
   }
   */

  return lightLevel;
}

float diff(float a, float b)
{
  if (a >= b) return (a - b);
  else return (b - a);
}


void toggleMode()
{
  killAllMidi(); // prevents stuck notes
  if (ChordTriggerMode)
  {
    if (!InsaneMode) InsaneMode = 1;
    else
    {
      ChordTriggerMode = 0;
      InsaneMode = 0;
    }
  }
  else ChordTriggerMode = 1;
}


void activateMoodInvert()
{
  //killAllMidi();
  moodInvertMode = 1;
}

void deactivateMoodInvert()
{
  //killAllMidi();
  moodInvertMode = 0;
}



unsigned int countStringsTriggered(void)
{
  unsigned int stringsTriggered = 0;
  for (unsigned int i = 0; i < NUMSTRINGS; i++)
  {
    stringsTriggered += triggered[i];
  }
  return stringsTriggered;
}  

int controlCombo(void)
{
  unsigned int stringsTriggered = countStringsTriggered();
  if (stringsTriggered == 2 &&
    triggered[2] &&
    triggered[3])
  {
    if (modeSwitchable)
    {
      triggered[2] = 0;
      triggered[3] = 0;
    }
    return 1;
  }
  else return 0;
}


int modulationCue(void)
{
  unsigned int stringsTriggered = countStringsTriggered();
  if (stringsTriggered == 2 &&
    triggered[6])
  {
    triggered[6] = 0;  // clear light string 7 so it doesn't sound
    sensorValues[6] = 0;
    return 1;
  }
  else return 0;
}


void noteOn(int channel, int pitch, int velocity)
{
  midiVelocities[channel][pitch] = velocity;
}


void noteOff(int channel, int pitch)
{
  midiVelocities[channel][pitch] = 0;
  //midiSend(0x90 + channel, pitch, 0);
}


void killAllMidi()
{
  for (unsigned int i = 0; i < NUM_MIDI_CHANNELS; i++)
  {
    killAllMidiOnChannel(i);
  }
}


void killAllMidiOnChannel(int channel) // sends noteOff messages for all notes listed as active in the table
{
  for (unsigned int i = 0; i < 128; i++)  
  {
    if (midiVelocities[channel][i] != 0)
    {
      noteOff(channel, i);
    }
  }
}


//  plays a MIDI note.  Doesn't check to see that
//  cmd is greater than 127, or that data values are less than 127:
void midiSend(int cmd, int pitch, int velocity)
{
  /*
  Serial.write(cmd);
  Serial.write(pitch);
  Serial.write(velocity);
  */
  if (pitch >= midiPitchMin &&
      pitch <= midiPitchMax &&
      velocity >= 0 &&
      velocity <= 127)
      {
        usbMIDI.sendNoteOn(pitch, velocity, strumChannel);
        delay(midiSendDelay);
      }
}


unsigned long int rcTime(unsigned int pin)
{
  // charge cap
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delay(capChargeDelay); // allow time to charge
  // measure discharge time
  unsigned long int i, SensorVal;
  pinMode(pin, INPUT);
  digitalWrite(pin, LOW);  // THIS IS THE PART I WAS MISSING!!!  why do i even need this?
  for (i = 1; i < 65535; i++)
  {
    //delayMicroseconds(10);
    SensorVal = digitalRead(pin);
    //flash(13);
    if (SensorVal == 0) break;
  }
  // discharge cap
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delay(capChargeDelay); // allow time to discharge
  return i;
}


void flash(int pin)
{
  int dt = 1; // delay time
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delay(dt);
  digitalWrite(pin, LOW);
  delay(dt);
}


void beep()
{
  /*
    for (unsigned long int i = 0; i < 100; i++)
   {
   for (unsigned long int j = 0; j < 256; j++)
   {
   analogWrite(SoundOutPin, j);
   }
   for (unsigned long int j = 255; j > 0; j--)
   {
   analogWrite(SoundOutPin, j);
   }
   }
   */
  freqout(SoundOutPin, 2000, longDelay);
}


void sound(unsigned int frequency, unsigned int duration)
{
  if (!PiezoSoundOn) return;
  if (frequency < 20 || frequency > 20000) return; // filter ultra/infrasound
  if (frequency < 1) frequency = 1; // don't divide by zero
  unsigned long int wavelength = 1000000 / frequency;
  unsigned long int halfwave = wavelength / 2; 
  unsigned long int iterations = (frequency * duration) / 1000;
  for (unsigned long int i = 0; i < iterations; i++)
  {
    digitalWrite(SoundOutPin, HIGH);
    delayMicroseconds(halfwave);
    digitalWrite(SoundOutPin, LOW);
    delayMicroseconds(halfwave);
  }
}

void playTone(int tone, int duration) {
  for (long i = 0; i < duration * 1000L; i += tone * 2) {
    digitalWrite(SoundOutPin, HIGH);
    delayMicroseconds(tone);
    digitalWrite(SoundOutPin, LOW);
    delayMicroseconds(tone);
  }
}

void playNote(int note, int duration)
{
  /*
'Freqs DATA @0, Word 1047, '2093, '1047, '523, 'C
   '               Word 1175, '2349, '1175, '587, 'D
   '               Word 1319, '2637, '1319, '659, 'E
   '               Word 1397, '2794, '1397, '698, 'F
   '               Word 1568, '3136, '1568, '784, 'G
   '               Word 1760, '3520, '1760, '880, 'A
   '               Word 1976  '3951 '1976 '988 '494 'B
   */
  int tones[] = { 
    1047, 1175, 1319, 1397, 1568, 1760, 1976   };
  //playTone(tones[note], duration);
  freqout(SoundOutPin, tones[note], duration);
  //wavoutmulti(SoundOutPin, tones[note], tones[(note + 2) % NUMSTRINGS]);
}

/* old
 void playNote(char note, int duration) {
 char names[] = { 'c', 'd', 'e', 'f', 'g', 'a', 'b', 'C' };
 int tones[] = { 1915, 1700, 1519, 1432, 1275, 1136, 1014, 956 };
 
 // play the tone corresponding to the note name
 for (int i = 0; i < 8; i++) {
 if (names[i] == note) {
 playTone(tones[i], duration);
 }
 }
 }
 */


// freqout from Arduino.cc
// http://www.arduino.cc/playground/Main/Freqout
void freqout(int outpin, int freq, int t)  // freq in hz, t in ms
{
  int hperiod;                               //calculate 1/2 period in us
  long cycles, i;
  pinMode(outpin, OUTPUT);                   // turn on output pin

  hperiod = (500000 / freq) - 7;             // subtract 7 us to make up for digitalWrite overhead

  cycles = ((long)freq * (long)t) / 1000;    // calculate cycles
  // Serial.print(freq);
  // Serial.print((char)9);                   // ascii 9 is tab - you have to coerce it to a char to work 
  // Serial.print(hperiod);
  // Serial.print((char)9);
  // Serial.println(cycles);

  for (i=0; i<= cycles; i++){              // play note for t ms 
    digitalWrite(outpin, HIGH); 
    delayMicroseconds(hperiod);
    digitalWrite(outpin, LOW); 
    delayMicroseconds(hperiod - 1);     // - 1 to make up for digitaWrite overhead
  }
  digitalWrite(SoundOutPin, SoundOutPinRestState);
  //pinMode(outpin, INPUT);                // shut off pin to avoid noise from other operations
}


void wavoutmulti(int outpin, unsigned int wl1, unsigned int wl2)
{
  unsigned long int counter1 = 0, counter2 = 0;
  for (unsigned long int i = 0; i < 50000; i++)
  {
    counter1++;
    counter2++;
    if (counter1 > wl1)
    {
      pulse(outpin);
      counter1 = 0;
    }
    if (counter2 > wl2)
    {
      pulse(outpin);
      counter2 = 0;
    }
  }
}

void pulse(int outpin)
{
  digitalWrite(outpin, HIGH);
  delayMicroseconds(10);
  digitalWrite(outpin, LOW);
  //delayMicroseconds(1);
}
