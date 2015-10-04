/*
 * Mode list:
 * BeatController - runs off beats, standard mode
 *     - beatControllerOn
 * Manual - accepted manual pixel setting commands
 *     - cSetPixels, cSetPixelsUniform, cSetStripColor
 * AutoMood - accepted mood selection
 *     - netflixAndChill, plasma, slowFade, restMode
 *
 * Mode will automatically be set when using command associated with that mode
 * 
 * special cases:
 * When BeatController hasn't seen audio in 5 minutes
 * restMode will be activated, and resumeOnAudio will be set
 * once audio is seen, then BeatController Mode will resume
 * if getMode is called during this point, it will still return BeatController
 */


//Enums are set from the client using their number
//so 0, 1, 2 etc

/* Command Format
 * commandName arg1 arg2 argX...
 * 
 * successful command returns:
 *   OK
 *   <Command response, if there is one>
 * 
 * failed command returns:
 *   BAD
 *   <Error>
 *
 * command not found returns:
 *   404 Command <Command name> not found
 */
//get current mode
void cGetMode() {
  Serial.println("OK");
  Serial.println(mode);
}

//State changing functions below

//BeatController
void cBeatControllerOn() {
  mode = BeatController;
}

//Manual

//set pixels
//arguments:
// strip pixel #, RGB value
void cSetPixelUniform() {
  int pixel, color;
  mode = Manual;
  pixel = atoi(sCmd.next());
  color = atoi(sCmd.next());
  setPixelUniform(pixel, color);
}

//set pixels
//arguments:
// strip, strip pixel #, RGB value
void cSetPixelStrip() {
  int strip, pixel, color;
  mode = Manual;
  strip = atoi(sCmd.next());
  pixel = atoi(sCmd.next());
  color = atoi(sCmd.next());
  setPixelStrip(strip, pixel, color);
}

//set pixels manually
void cSetPixelManual() {
  int pixel, color;
  mode = Manual;
  pixel = atoi(sCmd.next());
  color = atoi(sCmd.next());
  pixels.setPixel(pixel, color);
}

//set color of entire strip
void cSetStripColor() {
  int strip, color;
  mode = Manual;
  strip = atoi(sCmd.next());
  color = atoi(sCmd.next());
  for (int i = 0; i < BAR_LENGTH; ++i) {
    setPixelStrip(strip, i, color);
  }
}

//set color of entire strip
void cSetUniformColor() {
  int color;
  mode = Manual;
  color = atoi(sCmd.next());
  for (int i = 0; i < BAR_LENGTH; ++i) {
    setPixelUniform(i, color);
  }
}

void cShowPixels() {
  pixels.show();
}

