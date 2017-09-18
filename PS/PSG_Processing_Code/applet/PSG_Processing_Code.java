import processing.core.*; 
import processing.xml.*; 

import JMyron.*; 
import blobDetection.*; 
import processing.serial.*; 
import ddf.minim.*; 
import java.awt.Frame; 
import processing.opengl.*; 
import procontroll.*; 
import net.java.games.input.*; 
import org.apache.commons.math.optimization.linear.*; 
import org.apache.commons.math.stat.clustering.*; 
import org.apache.commons.math.stat.descriptive.summary.*; 
import org.junit.runner.manipulation.*; 
import junit.runner.*; 
import com.stromberglabs.util.*; 
import org.apache.commons.math.analysis.polynomials.*; 
import org.hamcrest.internal.*; 
import junit.extensions.*; 
import org.apache.commons.math.analysis.integration.*; 
import junit.framework.*; 
import org.apache.commons.math.stat.inference.*; 
import org.junit.runners.*; 
import org.apache.commons.math.stat.*; 
import org.apache.commons.math.complex.*; 
import org.apache.commons.math.stat.descriptive.*; 
import org.apache.commons.math.ode.jacobians.*; 
import org.junit.experimental.max.*; 
import org.junit.runners.model.*; 
import org.apache.commons.math.special.*; 
import org.apache.commons.math.stat.descriptive.rank.*; 
import org.junit.internal.runners.statements.*; 
import org.apache.commons.math.*; 
import org.apache.commons.math.stat.correlation.*; 
import org.junit.internal.matchers.*; 
import org.junit.experimental.theories.internal.*; 
import org.junit.runner.notification.*; 
import org.apache.commons.math.stat.regression.*; 
import org.junit.runner.*; 
import org.junit.internal.runners.model.*; 
import org.junit.*; 
import org.apache.commons.math.util.*; 
import org.junit.experimental.theories.suppliers.*; 
import org.apache.commons.math.analysis.solvers.*; 
import org.apache.commons.math.optimization.general.*; 
import org.junit.experimental.results.*; 
import org.junit.experimental.*; 
import org.junit.experimental.theories.*; 
import com.stromberglabs.jopensurf.*; 
import org.apache.commons.math.analysis.interpolation.*; 
import org.apache.commons.math.analysis.*; 
import org.apache.commons.math.stat.descriptive.moment.*; 
import org.apache.commons.math.ode.sampling.*; 
import org.apache.commons.math.optimization.univariate.*; 
import org.apache.commons.math.estimation.*; 
import org.apache.commons.math.geometry.*; 
import org.apache.commons.math.fraction.*; 
import org.apache.commons.math.ode.events.*; 
import org.junit.internal.*; 
import org.apache.commons.math.genetics.*; 
import junit.textui.*; 
import org.junit.internal.runners.*; 
import org.junit.experimental.runners.*; 
import org.apache.commons.math.ode.*; 
import org.apache.commons.math.random.*; 
import org.apache.commons.math.transform.*; 
import com.stromberglabs.cluster.*; 
import org.apache.commons.math.optimization.direct.*; 
import org.apache.commons.math.ode.nonstiff.*; 
import org.apache.commons.math.linear.*; 
import org.junit.matchers.*; 
import org.junit.internal.requests.*; 
import org.junit.internal.builders.*; 
import org.hamcrest.*; 
import org.hamcrest.core.*; 
import org.apache.commons.math.optimization.*; 
import org.apache.commons.math.distribution.*; 
import org.apache.commons.math.stat.ranking.*; 
import org.apache.commons.math.optimization.fitting.*; 
import java.awt.image.BufferedImage; 
import java.io.File; 
import java.io.IOException; 
import java.util.Map; 
import javax.imageio.ImageIO; 
import guicomponents.*; 

import java.applet.*; 
import java.awt.Dimension; 
import java.awt.Frame; 
import java.awt.event.MouseEvent; 
import java.awt.event.KeyEvent; 
import java.awt.event.FocusEvent; 
import java.awt.Image; 
import java.io.*; 
import java.net.*; 
import java.text.*; 
import java.util.*; 
import java.util.zip.*; 
import java.util.regex.*; 

public class PSG_Processing_Code extends PApplet {

/*
 -------------------- Project Sentry Gun --------------------
 ============================================================
 ----- An Open-Source Project, initiated by Bob Rudolph -----
   
 
 Help & Reference: http://projectsentrygun.rudolphlabs.com/make-your-own/using-the-software
 Forum: http://projectsentrygun.rudolphlabs.com/forum
 
   
 A few keyboard shortcuts:
 press 'p' for a random sound effect
 press 'b' to set background image
 hold 'r' and click+drag to form a rectangle  "fire-restricted" zone
 press SPACEBAR to toggle manual/autonomous modes
 press SHIFT to toggle arrow-key aiming in manual mode
 
 */


 //   <===============================================================================================>
 //   Begin custom values - change these camera dimensions to work with your turret
 //   <===============================================================================================>

public int camWidth = 640;                   //   camera width (pixels),   usually 160*n
public int camHeight = 480;                  //   camera height (pixels),  usually 120*n

public boolean use_surf = true; //Use Surf Classifier to search for Reference Object
public int surf_sensivity = 3; //Recognition Sensitivity lower=higher sensivity=higher false positives
public String surfRefFile = "C:/Users/Britt/Desktop/Sentry/robot_live2.jpg"; //Reference frame to recognize

 //   <===============================================================================================>
 //   End custom values
 //   <===============================================================================================>

boolean PRINT_FRAMERATE = true;     // set to true to print the framerate at the bottom of the IDE window

int[] diffPixelsColor = {
  255, 255, 0
};  // Red, green, blue values (0-255)  to show pixel as marked as target
public int effect = 0;                // Effect

public boolean mirrorCam = false;            //   set true to mirror camera image

public float xMin = 0.0f;      //  Actual calibration values are loaded from "settings.txt".
public float xMax = 180.0f;    //  If "settings.txt" is borken / unavailable, these defaults are used instead - 
public float yMin = 0.0f;      //  otherwise, changing these lines will have no effect on your gun's calibration.
public float yMax = 180.0f;    //






                  // see note on OpenGL in void setup() 













































































public int minBlobArea = 30;                    //   minimum target size (pixels)
public int tolerance = 100;                      //   sensitivity to motion

public boolean runWithoutArduino = true;
public boolean connecting = false;

public Surf mSurfRef;
public Surf mSurfCap;
public Map<SURFInterestPoint,SURFInterestPoint> mAMatchingPoints;
public Map<SURFInterestPoint,SURFInterestPoint> mBMatchingPoints;
public Map<SURFInterestPoint,SURFInterestPoint> pointsA;
public Map<SURFInterestPoint,SURFInterestPoint> pointsB;
public BufferedImage imageRef;
public  BufferedImage captureImage;

public Serial arduinoPort;
JMyron camInput;
BlobDetection target;
Blob blob;
Blob biggestBlob;


int[] Background;
int[] rawImage;
int[] rawBackground;
int[] currFrame;
int[] screenPixels;
public int targetX = camWidth/2;
public int targetY = camHeight/2;
int fire = 0;
int[] prevFire = {
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

float xRatio;
float yRatio;

int possibleX = camWidth/2;
int possibleY = camHeight/2;

int displayX = camWidth/2;
int displayY = camHeight/2;

int oldX = camWidth/2; // smoothing (contributed by Adam S.)
int oldY = camHeight/2; // smoothing
int xdiff; // smoothing
int ydiff; // smoothing
public float smoothingFactor = 0.8f; // smoothing
public boolean activeSmoothing = true;

String strTargetx;
String strTargety;
String fireSelector;
String scanSelector;


public boolean showDifferentPixels = false;
public boolean showTargetBox = true;
public boolean showCameraView = true;
public boolean firingMode = true;             // true = semi,        false = auto
public boolean safety = true;
public boolean controlMode = false;           // true = autonomous,  false = manual
public boolean soundEffects =  false;         // set to true to enable sound effects by default
public boolean scanWhenIdle = true;
public boolean trackingMotion = true;

int idleTime = 10000;          // how many milliseconds to wait until scanning (when in scan mode)
int idleBeginTime = 0;
boolean scan = false;

public String serPortUsed;

int[][] fireRestrictedZones = new int[30][4];
int restrictedZone = 1;
boolean showRestrictedZones = false;

boolean selectingColor = false;
boolean trackingColor = false;
int trackColorTolerance = 100;
int trackColorRed = 255;
int trackColorGreen = 255;
int trackColorBlue = 255;

boolean selectingSafeColor = false;
boolean safeColor = false;
int safeColorMinSize = 500;
int safeColorTolerance = 100;
int safeColorRed = 0;
int safeColorGreen = 255;
int safeColorBlue = 0;

boolean useArrowKeys = false;   // use arrow keys to finely adjust the aiming (in manual mode)

public boolean useInputDevice = false;  // use a joystick or game controller as input (in manual mode)
public boolean inputDeviceIsSetup = false;

public ControllIO controlIO;         // more stuff for using a joystick or game controller for input
public ControllDevice inputDevice;

public ControllButton[] buttons = new ControllButton[30];
public ControllSlider[] sliders = new ControllSlider[10];

public ControllButton[] fire_buttons = new ControllButton[0];
public ControllButton[] preciseAim_buttons = new ControllButton[0];
public ControllButton[] centerGun_buttons = new ControllButton[0];
public ControllButton[] autoOn_buttons = new ControllButton[0];
public ControllButton[] autoOff_buttons = new ControllButton[0];
public ControllButton[] inputToggle_buttons = new ControllButton[0];
public ControllButton[] randomSound_buttons = new ControllButton[0];

public ControllSlider[] pan_sliders = new ControllSlider[0];
public ControllSlider[] tilt_sliders = new ControllSlider[0];
public ControllSlider[] panInvert_sliders = new ControllSlider[0];
public ControllSlider[] tiltInvert_sliders = new ControllSlider[0];


public float xPosition = camWidth/2;
public float yPosition = camHeight/2;


String[] inStringSplit;  // buffer for backup
int controlMode_i, safety_i, firingMode_i, scanWhenIdle_i, trackingMotion_i, trackingColor_i, leadTarget_i, safeColor_i, 
showRestrictedZones_i, showDifferentPixels_i, showTargetBox_i, showCameraView_i, mirrorCam_i, soundEffects_i;


public void setup() {
  
  loadSettings();
  
  size(camWidth, camHeight);                  // some users have reported a faster framerate when the code utilizes OpenGL. To try this, comment out this line and uncomment the line below.
//  size(camWidth, camHeight, OPENGL);
  minim = new Minim(this);
  loadSounds();
  playSound(18);
  camInput = new JMyron();
  camInput.start(camWidth, camHeight);
  camInput.findGlobs(0);
  camInput.adaptivity(1.01f);
  camInput.update();
  currFrame = camInput.image();
  rawImage = camInput.image();
  Background = camInput.image();
  rawBackground = camInput.image();
  screenPixels = camInput.image();
  target = new BlobDetection(camWidth, camHeight);
  target.setThreshold(0.9f);
  target.setPosDiscrimination(true);
  
  if (use_surf) {  
    captureImage = new BufferedImage(camWidth, camHeight, BufferedImage.OPAQUE);
    try {
    imageRef = ImageIO.read(new File(surfRefFile));
    } catch (IOException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
    }
    mSurfRef = new Surf(imageRef);
  }
  
  retryArduinoConnect();

  xRatio = (camWidth / (xMax - xMin));                         // used to allign sights with crosshairs on PC
  yRatio = (camHeight/ (yMax - yMin));                         //
  drawControlPanel();
}



public void draw() {
  if (PRINT_FRAMERATE) {
    println(frameRate);
  }

  if (controlMode) {              // autonomous mode
    autonomousMode();            //
  }
  else if (!controlMode) {        // manual mode
    manualMode();                //
  }

  if (fire == 1) {
    idleBeginTime = millis();
    scan = false;
  }
  else {
    if (millis() > idleBeginTime + idleTime && controlMode && scanWhenIdle) {
      scan = true;
    }
    else {
      scan = false;
    }
  }

  if (!safety) {
    fire = 0;
  }

  strTargetx = "000" + str(targetX);                   // make into 3-digit numbers
  strTargetx = strTargetx.substring(strTargetx.length()-3);
  strTargety = "000" + str(targetY);
  strTargety = strTargety.substring(strTargety.length()-3);
  fireSelector = str(0);
  if (firingMode) {
    fireSelector = str(1);
  }
  else {
    fireSelector = str(3);
  }
  if (scan) {
    scanSelector = str(1);
  }
  else {
    scanSelector = str(0);
  }
  //println('a' + strTargetx + strTargety  + str(fire) + fireSelector + scanSelector);
  if (!runWithoutArduino && !connecting) {
    arduinoPort.write('a' + strTargetx + strTargety + str(fire) + fireSelector + scanSelector);   // send to arduino
  }

  if ((keyPressed && key == 't') || showRestrictedZones) {
    for (int col = 0; col <= restrictedZone; col++) {
      noStroke();
      fill(0, 255, 0, 100);
      rect(fireRestrictedZones[col][0], fireRestrictedZones[col][2], fireRestrictedZones[col][1]-fireRestrictedZones[col][0], fireRestrictedZones[col][3]-fireRestrictedZones[col][2]);
    }
  }
  if (selectingColor) {
    stroke(190, 0, 190);
    strokeWeight(2);
    fill(red(currFrame[(mouseY*width)+mouseX]), green(currFrame[(mouseY*width)+mouseX]), blue(currFrame[(mouseY*width)+mouseX]));
    rect(mouseX+2, mouseY+2, 30, 30);
  }

  if (selectingSafeColor) {
    stroke(0, 255, 0);
    strokeWeight(2);
    fill(red(currFrame[(mouseY*width)+mouseX]), green(currFrame[(mouseY*width)+mouseX]), blue(currFrame[(mouseY*width)+mouseX]));
    rect(mouseX+2, mouseY+2, 30, 30);
  }

  soundTimer++;
  if (soundTimer == soundInterval) {
    randomIdleSound();
    soundTimer = 0;
  }

  for (int i = 9; i >= 1; i--) {
    prevFire[i] = prevFire[i-1];
  }
  prevFire[0] = fire;
  int sumNewFire = prevFire[0] + prevFire[1] + prevFire[2] + prevFire[3] + prevFire[4];
  int sumPrevFire = prevFire[5] + prevFire[6] + prevFire[7] + prevFire[8] + prevFire[9];

  if (sumNewFire == 0 && sumPrevFire == 5) {     // target departed screen
    int s = PApplet.parseInt(random(0, 6));
    if (s == 0)
      playSound(1);
    if (s == 1)
      playSound(5);
    if (s == 2)
      playSound(9);
    if (s == 3)
      playSound(12);
    if (s == 4)
      playSound(13);
    if (s == 5)
      playSound(20);
  }

  if (fire == 1)
    strokeWeight(3);
  if (fire == 0)
    strokeWeight(1);
  stroke(255, 0, 0);                     //draw crosshairs
  noFill();                            // 
  line(displayX, 0, displayX, camHeight);  //
  line(0, displayY, camWidth, displayY);   //
  ellipse(displayX, displayY, 20, 20);     //
  ellipse(displayX, displayY, 28, 22);     //
  ellipse(displayX, displayY, 36, 24);     //

  updateControlPanels();
  prevTargetX = targetX;
  prevTargetY = targetY;
}

public void autonomousMode() {
  if(inputDeviceIsSetup) {
    checkInputDevice();
  }
  
  if (selectingColor || selectingSafeColor) {
    cursor(1);
  }
  else {
    cursor(0);
  }
  camInput.update();
  rawBackground = camInput.retinaImage();
  rawImage = camInput.image();
  if (mirrorCam) {
    for (int i = 0; i < camWidth*camHeight; i++) {
      int y = floor(i/camWidth);
      int x = i - (y*camWidth);
      x = camWidth-x;
      currFrame[i] = rawImage[(y*camWidth) + x-1];
      Background[i] = rawBackground[(y*camWidth) + x-1];
    }
  }
  else {
    currFrame = rawImage;
    Background = rawBackground;
  }

  loadPixels();
  int safeColorPixelsCounter = 0;

  for (int i = 0; i < camWidth*camHeight; i++) {
    if (showCameraView) {
      pixels[i] = currFrame[i];
    }
    else {
      pixels[i] = color(0, 0, 0);
    }        

    boolean motion = (((abs(red(currFrame[i])-red(Background[i])) + abs(green(currFrame[i])-green(Background[i])) + abs(blue(currFrame[i])-blue(Background[i]))) > (200-tolerance)) && trackingMotion);
    boolean isTrackedColor = (((abs(red(currFrame[i])-trackColorRed) + abs(green(currFrame[i])-trackColorGreen) + abs(blue(currFrame[i])-trackColorBlue)) < trackColorTolerance) && trackingColor);

    boolean isSafeColor = (((abs(red(currFrame[i])-safeColorRed) + abs(green(currFrame[i])-safeColorGreen) + abs(blue(currFrame[i])-safeColorBlue)) < safeColorTolerance) && safeColor);

    if (motion || isTrackedColor) {
      screenPixels[i] = color(255, 255, 255);
      if (showDifferentPixels) {
        if (effect == 0) {
          pixels[i] = color(diffPixelsColor[0], diffPixelsColor[1], diffPixelsColor[2]);
        }
        else if (effect == 1) {
          pixels[i] = color((diffPixelsColor[0] + red(currFrame[i]))/2, (diffPixelsColor[1] + green(currFrame[i]))/2, (diffPixelsColor[2] + blue(currFrame[i]))/2);
        }
        else if (effect == 2) {
          pixels[i] = color(255-red(currFrame[i]), 255-green(currFrame[i]), 255-blue(currFrame[i]));
        }
        else if (effect == 3) {
          pixels[i] = color((diffPixelsColor[0] + (255-red(currFrame[i])))/2, (diffPixelsColor[1] + (255-green(currFrame[i])))/2, (diffPixelsColor[2] + (255-blue(currFrame[i])))/2);
        }
      }
    }
    else {
      screenPixels[i] = color(0, 0, 0);
    }

    if (isSafeColor) {
      safeColorPixelsCounter++;
      pixels[i] = color(0, 255, 0);
      screenPixels[i] = color(0, 0, 0);
    }
  }



  updatePixels();

  int biggestBlobArea = 0;
  target.computeBlobs(screenPixels);
  for (int i = 0; i < target.getBlobNb()-1; i++) {
    blob = target.getBlob(i);
    int blobWidth = PApplet.parseInt(blob.w*camWidth);
    int blobHeight = PApplet.parseInt(blob.h*camHeight);
    if (blobWidth*blobHeight >= biggestBlobArea) {
      biggestBlob = target.getBlob(i);
      biggestBlobArea = PApplet.parseInt(biggestBlob.w*camWidth)*PApplet.parseInt(biggestBlob.h*camHeight);
    }
  }
  possibleX = 0;
  possibleY = 0;

  if (biggestBlobArea >= minBlobArea) {
    possibleX = PApplet.parseInt(biggestBlob.x * camWidth);
    possibleY = PApplet.parseInt(biggestBlob.y * camHeight);
  }


  if ((biggestBlobArea >= minBlobArea)) {
    fire = 1;
    if (showTargetBox) {
      stroke(255, 50, 50);
      strokeWeight(3);
      fill(255, 50, 50, 150);
      rect(PApplet.parseInt(biggestBlob.xMin*camWidth), PApplet.parseInt(biggestBlob.yMin*camHeight), PApplet.parseInt((biggestBlob.xMax-biggestBlob.xMin)*camWidth), PApplet.parseInt((biggestBlob.yMax-biggestBlob.yMin)*camHeight));
    }

    anticipation();

    if (activeSmoothing) {
      xdiff = possibleX - oldX; // smoothing
      ydiff = possibleY - oldY; // smoothing
      possibleX = PApplet.parseInt(oldX + xdiff*(1.0f-smoothingFactor)); // smoothing
      possibleY = PApplet.parseInt(oldY + ydiff*(1.0f-smoothingFactor)); // smoothing
    }

    displayX = possibleX;
    displayY = possibleY;
    if (displayX < 0)
      displayX = 0;
    if (displayX > camWidth)
      displayX = camWidth;
    if (displayY < 0)
      displayY = 0;
    if (displayY > camHeight)
      displayY = 0;  
    targetX = PApplet.parseInt((possibleX/xRatio)+xMin);         
    targetY = PApplet.parseInt(((camHeight-possibleY)/yRatio)+yMin);
    oldX = possibleX; // smoothing
    oldY = possibleY; // smoothing
  }
  else {
    fire = 0;
  }

  boolean clearOfZones = true;
  for (int col = 0; col <= restrictedZone; col++) {
    if (possibleX > fireRestrictedZones[col][0] && possibleX < fireRestrictedZones[col][1] && possibleY > fireRestrictedZones[col][2] && possibleY < fireRestrictedZones[col][3]) {
      clearOfZones = false;
      fire = 0;
    }
  }


  if (safeColorPixelsCounter > safeColorMinSize && safeColor) {
    noStroke();
    fill(0, 255, 0, 150);
    rect(0, 0, width, height);
    fire = 0;
    targetX = PApplet.parseInt((xMin+xMax)/2.0f);
    targetY = PApplet.parseInt(yMin);
    displayX = camWidth/2;
    displayY = camHeight;
  }
  
  if (use_surf) {
    captureImage.setRGB(0, 0, camWidth, camHeight, currFrame, 0, camWidth);
    mSurfCap = new Surf(captureImage);
    mBMatchingPoints = mSurfCap.getMatchingPoints(mSurfRef,true);
    System.out.println(mBMatchingPoints.size());
    
    if (mBMatchingPoints.size()>=surf_sensivity) {
      noStroke();
      fill(0, 255, 0, 150);
      rect(0, 0, width, height);
      fire = 0;
      targetX = PApplet.parseInt((xMin+xMax)/2.0f);
      targetY = PApplet.parseInt(yMin);
      displayX = camWidth/2;
      displayY = camHeight;
    }
  }
  
  
  
}

public void manualMode() {
//  cursor(1);
  camInput.update();
  rawBackground = camInput.retinaImage();
  rawImage = camInput.image();
  if (mirrorCam) {
    for (int i = 0; i < camWidth*camHeight; i++) {
      int y = floor(i/camWidth);
      int x = i - (y*camWidth);
      x = camWidth-x;
      currFrame[i] = rawImage[(y*camWidth) + x-1];
      Background[i] = rawBackground[(y*camWidth) + x-1];
    }
  }
  else {
    currFrame = rawImage;
    Background = rawBackground;
  }

  loadPixels();                                 //draw camera view to screen
  for (int i = 0; i < camWidth*camHeight; i++) {  //
    pixels[i] = currFrame[i];                   //
  }                                             //
  updatePixels();                               //

  if(inputDeviceIsSetup) {
    checkInputDevice();
  }
  if(useInputDevice) {
    updateInputDevice();                        // determine control values using the input device (see declaration in Input_Device tab)
    if(useArrowKeys) {   // use the arrow keys to aim one pixel at a time
      // use arrow keys to aim - see keyReleased() below
      if(keyPressed) {
        if (keyCode == 37) {                       // left arrow
          xPosition -= 1;
        }
        if (keyCode == 38) {                       // up arrow
          yPosition -= 1;
        }
        if (keyCode == 39) {                       // right arrow
          xPosition += 1;
        }
        if (keyCode == 40) {                       // down arrow
          yPosition += 1;
        }
        fire = 0;
        
      }
    }
  }else{  
    if(useArrowKeys) {   // use the arrow keys to aim one pixel at a time
      // use arrow keys to aim - see keyReleased() below
      if(keyPressed) {
        if (keyCode == 37) {                       // left arrow
          displayX -= 1;
        }
        if (keyCode == 38) {                       // up arrow
         displayY -= 1;
        }
        if (keyCode == 39) {                       // right arrow
          displayX += 1;
        }
        if (keyCode == 40) {                       // down arrow
          displayY += 1;
        }
        fire = 0;
      }
    }else{    
      displayX = mouseX;
      displayY = mouseY;
      if (mousePressed) {
        fire = 1;
      }
      else {
        fire = 0;
      }
    }
    targetX = constrain(PApplet.parseInt((displayX/xRatio)+xMin), 0, 180);                 // calculate position to go to based on mouse position
    
    targetY = constrain(PApplet.parseInt(((camHeight-displayY)/yRatio)+yMin), 0, 180);     //
    
  }
}


public void mousePressed() {
  if (keyPressed && key == 'r') {
    print("constraints:" + mouseX + ", " + mouseY);
    fireRestrictedZones[restrictedZone][0] = mouseX;
    fireRestrictedZones[restrictedZone][2] = mouseY;
  }
  if (selectingColor) {
    trackColorRed = PApplet.parseInt(red(currFrame[(mouseY*width)+mouseX]));
    trackColorBlue = PApplet.parseInt(blue(currFrame[(mouseY*width)+mouseX]));
    trackColorGreen = PApplet.parseInt(green(currFrame[(mouseY*width)+mouseX]));
    selectingColor = false;
  }

  if (selectingSafeColor) {
    safeColorRed = PApplet.parseInt(red(currFrame[(mouseY*width)+mouseX]));
    safeColorBlue = PApplet.parseInt(blue(currFrame[(mouseY*width)+mouseX]));
    safeColorGreen = PApplet.parseInt(green(currFrame[(mouseY*width)+mouseX]));
    selectingSafeColor = false;
  }
}

public void mouseReleased() {
  if (keyPressed && key == 'r') {
    println(" ... " + mouseX + ", " + mouseY);
    fireRestrictedZones[restrictedZone][1] = mouseX;
    fireRestrictedZones[restrictedZone][3] = mouseY;
    if (fireRestrictedZones[restrictedZone][1]>fireRestrictedZones[restrictedZone][0] && fireRestrictedZones[restrictedZone][1]>fireRestrictedZones[restrictedZone][2]) {
      restrictedZone++;
    }
  }
}

public void keyReleased() {
  if ( key == 'p') {
    randomIdleSound();
  }

  if (key == ' ') {
    controlMode = !controlMode;
  }

  if (key == 'b') {
    camInput.adapt();
    playSound(15);
  }
  if (key == 'a') {
    xMin = PApplet.parseFloat(targetX);
    xRatio = (camWidth / (xMax - xMin));                         // used to allign sights with crosshairs on PC
  }
  if (key == 'd') {
    xMax = PApplet.parseFloat(targetX);
    xRatio = (camWidth / (xMax - xMin));                         // used to allign sights with crosshairs on PC
  }
  if (key == 's') {
    yMin = PApplet.parseFloat(targetY);
    yRatio = (camHeight/ (yMax - yMin));                         //
  }
  if (key == 'w') {
    yMax = PApplet.parseFloat(targetY);
    yRatio = (camHeight/ (yMax - yMin));                         //
  }
  if(key == CODED && keyCode == 16) {      // shift key was pressed, toggle aim with arrow keys
    useArrowKeys = !useArrowKeys; 
  }
  
}



public void viewCameraSettings() {
  camInput.settings();
  playSound(21);
}

public void openWebsite() {
  link("http://psg.rudolphlabs.com/");
  playSound(15);
}

public void setBackground() {
  camInput.adapt();
  playSound(11);
}

public void playRandomSound() {
  randomIdleSound();
}

public void selectColor() {
  selectingColor = true;
}

public void selectSafeColor() {
  selectingSafeColor = true;
}

public void radioEffect(int ID) {
  effect = ID + 1;
}


public void stop() {
  if(soundEffects) {
    s1.rewind();
    s1.play();
    delay(2500);
    s1.close();
    s2.close();
    s3.close();
    s4.close();
    s5.close();
    s7.close();
    s6.close();
    s8.close();
    s9.close();
    s10.close();
    s11.close();
    s12.close();
    s13.close();
    s14.close();
    s15.close();
    s16.close();
    s17.close();
    s18.close();
    s19.close();
    s20.close();
    s21.close();
    minim.stop();
  }
  if (!runWithoutArduino) {
    arduinoPort.write("z0000000");
    delay(500);
    arduinoPort.stop();
  }
  camInput.stop();
  super.stop();
}

//  contributed by Hugo K.


boolean leadTarget = true;

int nbDot = 10;                      // nomber of dot for anticipation minimum 2
int antSens = 10;                   // sensitivity of anticipation
float propX = 0.67f;                  // proportionality of anticipation 
float propY = 0.11f;                    // 1 is Hight / more is Less

int[] oldPossibleX = new int[nbDot+1];  // 0 is actual position
int[] oldPossibleY = new int[nbDot+1];

int[] accX = new int[nbDot -1];  
int[] accY = new int[nbDot -1];

int[] travelX = new int[nbDot -1];  
int[] travelY = new int[nbDot -1];

float antX = 0;
float antY = 0;

int prevTargetX = targetX;
int prevTargetY = targetY;


public void anticipation() {

  if (leadTarget) {

    if (oldPossibleX.length != nbDot+1) {
      oldPossibleX = expand(oldPossibleX, nbDot+1);
    }
    if (oldPossibleY.length != nbDot+1) {
      oldPossibleY = expand(oldPossibleY, nbDot+1);
    }
    if (accX.length != nbDot-1) {
      accX = expand(accX, nbDot-1);
    }
    if (accY.length != nbDot-1) {
      accY = expand(accY, nbDot-1);
    }
    if (travelX.length != nbDot-1) {
      travelX = expand(travelX, nbDot-1);
    }
    if (travelY.length != nbDot-1) {
      travelY = expand(travelY, nbDot-1);
    }

    oldPossibleX[0] = possibleX;
    oldPossibleY[0] = possibleY;

    // Acceleration between oldPossibleX and old possibleX-1  
    for (int i=0; i<=nbDot-2; i++) {
      if ( abs(oldPossibleX[i] - oldPossibleX[i+1]) < camWidth/antSens && abs(oldPossibleX[i+1] - oldPossibleX[i+2]) < camWidth/antSens) {
        accX[i] = (oldPossibleX[i] - oldPossibleX[i+1]) - (oldPossibleX[i+1] - oldPossibleX[i+2]);
      }
      if ( abs(oldPossibleY[i] - oldPossibleY[i+1]) < camHeight/antSens && abs(oldPossibleY[i+1] - oldPossibleY[i+2]) < camHeight/antSens) {
        accY[i] = (oldPossibleY[i] - oldPossibleY[i+1]) - (oldPossibleY[i+1] - oldPossibleY[i+2]);
      }
    }

    // Travel between oldPossibleX and old possibleX-1  
    for (int i=0; i<= nbDot-2; i++) {
      if ( abs(oldPossibleX[i] - oldPossibleX[i+1]) < camWidth/antSens ) {
        travelX[i] = oldPossibleX[i] - oldPossibleX[i+1];
      }
      else {
        travelX[i] = 0;
      }
      if ( abs(oldPossibleY[i] - oldPossibleY[i+1]) < camHeight/antSens ) {
        travelY[i] = oldPossibleY[i] - oldPossibleY[i+1];
      }
      else {
        travelY[i] = 0;
      }
    }

    // addition of speed and acceleration
    // Each term can be weighted to have an improved algorithm

    antX = 0;
    antY = 0;

    for (int i=0; i<=nbDot-2; i++) {
      antX = antX + travelX[i] + accX[i];
      antY = antY + travelY[i] + accY[i];
    }

    antX = antX * propX;
    antY = antY * propY;

    // Updating  positions

    for (int i = nbDot; i>=1; i--) {
      oldPossibleX[i] = oldPossibleX[i-1];
      oldPossibleY[i] = oldPossibleY[i-1];
    }

    possibleX = PApplet.parseInt(possibleX + antX);
    possibleY = PApplet.parseInt(possibleY + antY);
  }
}
// current issues: cannot show the frame around control panel while using a picture background. to see areas of code related to this, do a search/find for "(tag frame)"

public int controlPanelWindowX = 50;         // x position on screen of upper-left corner of control panel
public int controlPanelWindowY = 100;        // y position on screen of upper-left corner of control panel



GPanel panel_main; // control panel
PImage panelBackgroundImg;

GLabel label_serialOut, label_targetX, label_targetY, label_fire, label_fireSelector, label_scanSelector, label_runWithoutArduino, label_xMin, label_xMax, label_yMin, label_yMax, label_setxMin, label_setxMax, label_setyMin, label_setyMax;   // text labels on control panel
GCheckbox checkbox_leadTarget, checkbox_showRestrictedZones, checkbox_trackingColor, checkbox_safeColor, checkbox_trackingMotion, checkbox_showDifferentPixels, checkbox_showTargetBox, checkbox_mirrorCam, checkbox_controlMode, checkbox_safety, checkbox_showCameraView, checkbox_scanWhenIdle, checkbox_soundEffects, checkbox_activeSmoothing, checkbox_useInputDevice, checkbox_useArrowKeys;// checkboxes
GButton button_viewCameraSettings, button_setBackground, button_selectColor, button_selectSafeColor, button_openWebsite, button_playRandomSound, button_saveSettings, button_loadSettings, button_retryArduinoConnect, button_saveAndExit, button_configJoystick, button_resetCalibration, button_flipX, button_flipY;	// buttons
GWSlider slider_tolerance, slider_trackColorTolerance, slider_safeColorTolerance, slider_safeColorMinSize, slider_minBlobArea, slider_nbDot, slider_antSens, slider_propX, slider_propY, slider_smoothingFactor; //sliders
GLabel label_slider_tolerance, label_slider_trackColorTolerance, label_slider_safeColorTolerance, label_slider_safeColorMinSize, label_slider_minBlobArea, label_slider_nbDot, label_slider_antSens, label_slider_propX, label_slider_propY, label_smoothingFactor; // value readouts for sliders
// GTextField txfSomeText;   // textfield
GCombo dropdown_effect, dropdown_firingMode, dropdown_comPort;   // dropdown menus
//GActivityBar acyBar;   // activity bar
//GTimer tmrTimer;       // timer

GOptionGroup opgMouseOver;
GOption optHand, optXhair, optMove, optText, optWait;

// G4P components for second windowl
GWindow window_main;

int sliderInertia = 3;

public void drawControlPanel() {

  G4P.setColorScheme(this, GCScheme.GREY_SCHEME);
  G4P.messagesEnabled(false);

  // create Panels
  panel_main = new GPanel(this, "Main", 0, 0, 600, 600);
  panel_main.setOpaque(false);
  panel_main.setCollapsed(false);

  // create labels
  label_serialOut = new GLabel(this, "Serial Out:           ", 300, 475, 150, 20);
  label_serialOut.setBorder(0);
  label_serialOut.setOpaque(false);
  label_serialOut.setColorScheme(GCScheme.GREY_SCHEME);
  panel_main.add(label_serialOut);

  label_targetX = new GLabel(this, "Pan Servo Position:    ", 300, 495, 150, 20);
  label_targetX.setBorder(0);
  label_targetX.setOpaque(false);
  label_targetX.setColorScheme(GCScheme.GREY_SCHEME);
  panel_main.add(label_targetX);

  label_targetY = new GLabel(this, "Tilt Servo Position:    ", 300, 515, 150, 20);
  label_targetY.setBorder(0);
  label_targetY.setOpaque(false);
  label_targetY.setColorScheme(GCScheme.GREY_SCHEME);
  panel_main.add(label_targetY);	

  label_fire = new GLabel(this, "Not Firing", 300, 535, 150, 20);
  label_fire.setBorder(0);
  label_fire.setOpaque(false); 
  label_fire.setColorScheme(GCScheme.RED_SCHEME);
  panel_main.add(label_fire);	

  label_fireSelector = new GLabel(this, "Automatic", 300, 555, 150, 20);
  label_fireSelector.setBorder(0);
  label_fireSelector.setOpaque(false);
  label_fireSelector.setColorScheme(GCScheme.GREY_SCHEME);
  panel_main.add(label_fireSelector);	

  label_scanSelector = new GLabel(this, "Scan When Idle", 300, 575, 150, 20);
  label_scanSelector.setBorder(0);
  label_scanSelector.setOpaque(false);
  label_scanSelector.setColorScheme(GCScheme.GREY_SCHEME);
  panel_main.add(label_scanSelector);	

  label_runWithoutArduino = new GLabel(this, "No Controller", 460, 475, 120, 20);
  label_runWithoutArduino.setBorder(0);
  label_runWithoutArduino.setOpaque(false);
  label_runWithoutArduino.setColorScheme(GCScheme.YELLOW_SCHEME);
  panel_main.add(label_runWithoutArduino);	

  label_xMin = new GLabel(this, "xMin: 000", 35, 362, 150, 20);
  label_xMin.setBorder(0);
  label_xMin.setOpaque(false);
  label_xMin.setColorScheme(GCScheme.YELLOW_SCHEME);
  panel_main.add(label_xMin);	

  label_xMax = new GLabel(this, "xMax: 180", 145, 362, 150, 20);
  label_xMax.setBorder(0);
  label_xMax.setOpaque(false);
  label_xMax.setColorScheme(GCScheme.YELLOW_SCHEME);
  panel_main.add(label_xMax);	

  label_yMin = new GLabel(this, "yMin: 000", 35, 392, 150, 20);
  label_yMin.setBorder(0);
  label_yMin.setOpaque(false);
  label_yMin.setColorScheme(GCScheme.YELLOW_SCHEME);
  panel_main.add(label_yMin);	

  label_yMax = new GLabel(this, "yMax: 180", 145, 392, 150, 20);
  label_yMax.setBorder(0);
  label_yMax.setOpaque(false);
  label_yMax.setColorScheme(GCScheme.YELLOW_SCHEME);
  panel_main.add(label_yMax);	

  label_setxMin = new GLabel(this, "to set xMin press A", 10, 375, 120, 10);
  panel_main.add(label_setxMin);

  label_setxMax = new GLabel(this, "to set xMax press D", 120, 375, 120, 10);
  panel_main.add(label_setxMax);

  label_setyMin = new GLabel(this, "to set yMin press S", 10, 405, 120, 10);
  panel_main.add(label_setyMin);

  label_setyMax = new GLabel(this, "to set yMax press W", 120, 405, 120, 10);
  panel_main.add(label_setyMax);

  // create checkboxes
  checkbox_leadTarget = new GCheckbox(this, "Enable Target Anticipation", 310, 325, 10);
  checkbox_leadTarget.setSelected(leadTarget);
  checkbox_leadTarget.setBorder(0);
  panel_main.add(checkbox_leadTarget);

  checkbox_showRestrictedZones = new GCheckbox(this, "Show Restricted Zones  (to set, hold R and click+drag)", 10, 480, 10);
  checkbox_showRestrictedZones.setSelected(showRestrictedZones);
  checkbox_showRestrictedZones.setBorder(0);
  panel_main.add(checkbox_showRestrictedZones);

  checkbox_trackingColor = new GCheckbox(this, "Track A Color", 10, 255, 10);
  checkbox_trackingColor.setSelected(trackingColor);
  checkbox_trackingColor.setBorder(0);
  panel_main.add(checkbox_trackingColor);

  checkbox_safeColor = new GCheckbox(this, "Enable Safe Color", 310, 175, 10);
  checkbox_safeColor.setSelected(safeColor);
  checkbox_safeColor.setBorder(0);
  panel_main.add(checkbox_safeColor);

  checkbox_trackingMotion = new GCheckbox(this, "Track Motion", 10, 205, 10);
  checkbox_trackingMotion.setSelected(trackingMotion);
  checkbox_trackingMotion.setBorder(0);
  panel_main.add(checkbox_trackingMotion);

  checkbox_showDifferentPixels = new GCheckbox(this, "Show Different Pixels", 10, 500, 10);
  checkbox_showDifferentPixels.setSelected(showDifferentPixels);
  checkbox_showDifferentPixels.setBorder(0);
  panel_main.add(checkbox_showDifferentPixels);

  checkbox_showTargetBox = new GCheckbox(this, "Show Target Box", 10, 520, 10);
  checkbox_showTargetBox.setSelected(showTargetBox);
  checkbox_showTargetBox.setBorder(0);
  panel_main.add(checkbox_showTargetBox);

  checkbox_mirrorCam = new GCheckbox(this, "Mirror Webcam", 10, 540, 10);
  checkbox_mirrorCam.setSelected(mirrorCam);
  checkbox_mirrorCam.setBorder(0);
  panel_main.add(checkbox_mirrorCam);

  checkbox_controlMode = new GCheckbox(this, "Enable Autonomous Mode (press SPACE to toggle)", 10, 25, 10);
  checkbox_controlMode.setSelected(controlMode);
  checkbox_controlMode.setBorder(0);
  panel_main.add(checkbox_controlMode);

  checkbox_safety = new GCheckbox(this, "Enable Weapon", 10, 325, 10);
  checkbox_safety.setSelected(safety);
  checkbox_safety.setBorder(0);
  panel_main.add(checkbox_safety);

  checkbox_showCameraView = new GCheckbox(this, "Show Camera View", 10, 560, 10);
  checkbox_showCameraView.setSelected(showCameraView);
  checkbox_showCameraView.setBorder(0);
  panel_main.add(checkbox_showCameraView);

  checkbox_scanWhenIdle = new GCheckbox(this, "Scan When Idle", 10, 345, 10);
  checkbox_scanWhenIdle.setSelected(scanWhenIdle);
  checkbox_scanWhenIdle.setBorder(0);
  panel_main.add(checkbox_scanWhenIdle);

  checkbox_soundEffects = new GCheckbox(this, "Enable Sounds Effects", 315, 25, 10);
  checkbox_soundEffects.setSelected(soundEffects);
  checkbox_soundEffects.setBorder(0);
  panel_main.add(checkbox_soundEffects);

  checkbox_activeSmoothing = new GCheckbox(this, "Smoothing", 460, 25, 10);
  checkbox_activeSmoothing.setSelected(activeSmoothing);
  checkbox_activeSmoothing.setBorder(0);
  panel_main.add(checkbox_activeSmoothing);

  checkbox_useInputDevice = new GCheckbox(this, "Use Joystick/Game Controller Input", 10, 45, 10);
  checkbox_useInputDevice.setSelected(useInputDevice);
  checkbox_useInputDevice.setBorder(0);
  panel_main.add(checkbox_useInputDevice);
  
  checkbox_useArrowKeys = new GCheckbox(this, "Use Arrow Keys to Fine Adjust (press SHIFT to toggle)", 10, 65, 10);
  checkbox_useArrowKeys.setSelected(useArrowKeys);
  checkbox_useArrowKeys.setBorder(0);
  panel_main.add(checkbox_useArrowKeys);


  // create buttons
  button_viewCameraSettings = new GButton(this, "Webcam Settings", 460, 75, 120, 10);
  panel_main.add(button_viewCameraSettings);

  button_setBackground = new GButton(this, "Save Current Image as Background", 110, 205, 185, 10);
  panel_main.add(button_setBackground);

  button_selectColor = new GButton(this, "Select Color to Track", 110, 255, 185, 10);
  panel_main.add(button_selectColor);

  button_selectSafeColor = new GButton(this, "Select Safe Color  ", 310, 200, 20, 10);
  panel_main.add(button_selectSafeColor);

  // with image
  button_openWebsite = new GButton(this, "", "Sentry_Logo_Tiny.png", 1, 545, 560, 36, 24);
  panel_main.add(button_openWebsite);

  button_playRandomSound = new GButton(this, "Play a Random Sound", 320, 75, 120, 10);
  panel_main.add(button_playRandomSound);

  button_saveSettings = new GButton(this, "Save Settings", 320, 110, 120, 10);
  panel_main.add(button_saveSettings);

  button_loadSettings = new GButton(this, "Re-Load Settings", 460, 110, 120, 10);
  panel_main.add(button_loadSettings);

  button_saveAndExit = new GButton(this, "Save Settings & Exit", 10, 95, 280, 40);
  panel_main.add(button_saveAndExit);

  button_retryArduinoConnect = new GButton(this, "Retry/Connect", 460, 500, 120, 10);
  panel_main.add(button_retryArduinoConnect);

  button_configJoystick  = new GButton(this, "Configure", 200, 45, 70, 10);
  panel_main.add(button_configJoystick);
  
  button_resetCalibration = new GButton(this, "Reset Calibration", 10, 425, 220, 10);
  panel_main.add(button_resetCalibration);
  
  button_flipX = new GButton(this, "Flip X", 240, 370, 40, 10);
  panel_main.add(button_flipX);
  
  button_flipY = new GButton(this, "Flip Y", 240, 400, 40, 10);
  panel_main.add(button_flipY);

  // create sliders
  slider_tolerance = new GWSlider(this, 10, 225, 200);
  slider_tolerance.setLimits(tolerance, 0, 200);
  slider_tolerance.setRenderMaxMinLabel(false);
  slider_tolerance.setRenderValueLabel(false);
  slider_tolerance.setTickCount(10);
  slider_tolerance.setInertia(sliderInertia);
  panel_main.add(slider_tolerance);
  label_slider_tolerance = new GLabel(this, "Tolerance: ", 210, 225, 150, 20);
  label_slider_tolerance.setBorder(0);
  label_slider_tolerance.setOpaque(false);
  label_slider_tolerance.setColorScheme(GCScheme.YELLOW_SCHEME);
  panel_main.add(label_slider_tolerance);

  slider_trackColorTolerance = new GWSlider(this, 10, 275, 200);
  slider_trackColorTolerance.setLimits(trackColorTolerance, 0, 300);
  slider_trackColorTolerance.setRenderMaxMinLabel(false);
  slider_trackColorTolerance.setRenderValueLabel(false);
  slider_trackColorTolerance.setTickCount(12);
  slider_trackColorTolerance.setInertia(sliderInertia);
  panel_main.add(slider_trackColorTolerance);
  label_slider_trackColorTolerance = new GLabel(this, "Tolerance: ", 210, 275, 150, 20);
  label_slider_trackColorTolerance.setBorder(0);
  label_slider_trackColorTolerance.setOpaque(false);
  label_slider_trackColorTolerance.setColorScheme(GCScheme.YELLOW_SCHEME);
  panel_main.add(label_slider_trackColorTolerance);

  slider_safeColorTolerance = new GWSlider(this, 310, 230, 200);
  slider_safeColorTolerance.setLimits(safeColorTolerance, 0, 300);
  slider_safeColorTolerance.setRenderMaxMinLabel(false);
  slider_safeColorTolerance.setRenderValueLabel(false);
  slider_safeColorTolerance.setTickCount(12);
  slider_safeColorTolerance.setInertia(sliderInertia);
  panel_main.add(slider_safeColorTolerance);
  label_slider_safeColorTolerance = new GLabel(this, "Tolerance: ", 510, 230, 150, 20);
  label_slider_safeColorTolerance.setBorder(0);
  label_slider_safeColorTolerance.setOpaque(false);
  label_slider_safeColorTolerance.setColorScheme(GCScheme.YELLOW_SCHEME);
  panel_main.add(label_slider_safeColorTolerance);

  slider_safeColorMinSize = new GWSlider(this, 310, 260, 200);
  slider_safeColorMinSize.setLimits(safeColorMinSize, 0, 5000);
  slider_safeColorMinSize.setRenderMaxMinLabel(false);
  slider_safeColorMinSize.setRenderValueLabel(false);
  slider_safeColorMinSize.setTickCount(10);
  slider_safeColorMinSize.setInertia(sliderInertia);
  panel_main.add(slider_safeColorMinSize);
  label_slider_safeColorMinSize = new GLabel(this, "Min Area: ", 510, 260, 150, 20);
  label_slider_safeColorMinSize.setBorder(0);
  label_slider_safeColorMinSize.setOpaque(false);
  label_slider_safeColorMinSize.setColorScheme(GCScheme.YELLOW_SCHEME);
  panel_main.add(label_slider_safeColorMinSize);

  slider_minBlobArea = new GWSlider(this, 10, 175, 200);
  slider_minBlobArea.setLimits(minBlobArea, 0, 10000);
  slider_minBlobArea.setRenderMaxMinLabel(false);
  slider_minBlobArea.setRenderValueLabel(false);
  slider_minBlobArea.setTickCount(10);
  slider_minBlobArea.setInertia(sliderInertia);
  panel_main.add(slider_minBlobArea);
  label_slider_minBlobArea = new GLabel(this, "Min Size: ", 210, 175, 150, 20);
  label_slider_minBlobArea.setBorder(0);
  label_slider_minBlobArea.setOpaque(false);
  label_slider_minBlobArea.setColorScheme(GCScheme.YELLOW_SCHEME);
  panel_main.add(label_slider_minBlobArea);

  slider_nbDot = new GWSlider(this, 310, 350, 200);
  slider_nbDot.setLimits(nbDot, 2, 22);
  slider_nbDot.setRenderMaxMinLabel(false);
  slider_nbDot.setRenderValueLabel(false);
  slider_nbDot.setTickCount(10);
  slider_nbDot.setInertia(sliderInertia);
  panel_main.add(slider_nbDot);
  label_slider_nbDot = new GLabel(this, "Memory: ", 510, 350, 150, 20);
  label_slider_nbDot.setBorder(0);
  label_slider_nbDot.setOpaque(false);
  label_slider_nbDot.setColorScheme(GCScheme.YELLOW_SCHEME);
  panel_main.add(label_slider_nbDot);

  slider_antSens = new GWSlider(this, 310, 375, 200);
  slider_antSens.setLimits(antSens, 1, 100);
  slider_antSens.setRenderMaxMinLabel(false);
  slider_antSens.setRenderValueLabel(false);
  slider_antSens.setTickCount(10);
  slider_antSens.setInertia(sliderInertia);
  panel_main.add(slider_antSens);
  label_slider_antSens = new GLabel(this, "Sensitivity: ", 510, 375, 150, 20);
  label_slider_antSens.setBorder(0);
  label_slider_antSens.setOpaque(false);
  label_slider_antSens.setColorScheme(GCScheme.YELLOW_SCHEME);
  panel_main.add(label_slider_antSens);

  slider_propX = new GWSlider(this, 310, 400, 120);
  slider_propX.setLimits(propX, 0.00f, 3.00f);
  slider_propX.setValueType(GWSlider.DECIMAL);
  slider_propX.setRenderMaxMinLabel(false);
  slider_propX.setRenderValueLabel(false);
  slider_propX.setTickCount(7);
  slider_propX.setInertia(sliderInertia);
  panel_main.add(slider_propX);
  label_slider_propX = new GLabel(this, "X Degree of Anticipation: ", 430, 400, 300, 20);
  label_slider_propX.setBorder(0);
  label_slider_propX.setOpaque(false);
  label_slider_propX.setColorScheme(GCScheme.YELLOW_SCHEME);
  panel_main.add(label_slider_propX);

  slider_propY = new GWSlider(this, 310, 425, 120);
  slider_propY.setLimits(propY, 0.00f, 3.00f);
  slider_propY.setValueType(GWSlider.DECIMAL);
  slider_propY.setRenderMaxMinLabel(false);
  slider_propY.setRenderValueLabel(false);
  slider_propY.setTickCount(7);
  slider_propY.setInertia(sliderInertia);
  panel_main.add(slider_propY);
  label_slider_propY = new GLabel(this, "Y Degree of Anticipation: ", 430, 425, 300, 20);
  label_slider_propY.setBorder(0);
  label_slider_propY.setOpaque(false);
  label_slider_propY.setColorScheme(GCScheme.YELLOW_SCHEME);
  panel_main.add(label_slider_propY); 

  slider_smoothingFactor = new GWSlider(this, 310, 50, 180);
  slider_smoothingFactor.setLimits(smoothingFactor, 0.00f, 1.00f);
  slider_smoothingFactor.setValueType(GWSlider.DECIMAL);
  slider_smoothingFactor.setRenderMaxMinLabel(false);
  slider_smoothingFactor.setRenderValueLabel(false);
  slider_smoothingFactor.setTickCount(10);
  slider_smoothingFactor.setInertia(sliderInertia);
  panel_main.add(slider_smoothingFactor);
  label_smoothingFactor = new GLabel(this, "Smoothing Factor", 490, 50, 100, 20);
  label_smoothingFactor.setBorder(0);
  label_smoothingFactor.setOpaque(false);
  label_smoothingFactor.setColorScheme(GCScheme.YELLOW_SCHEME);
  panel_main.add(label_smoothingFactor); 


  // createCombos (dropdown boxes)
  String[] entries_1 = new String[] {
    "Opaque", "Transparent", "Negative", "Negative & Transparent"
  };
  dropdown_effect = new GCombo(this, entries_1, entries_1.length, 140, 500, 140);        // this, String[] of entries, dropdown # of enteries shown at once, xPosition, yPosition, width
  dropdown_effect.setSelected(effect);                                                  // which entry to show as selected (first entry is 0, second is 1, etc.)
  panel_main.add(dropdown_effect);

  String[] entries_2 = new String[] {
    "Automatic", "Semi-Auto"
  };
  dropdown_firingMode = new GCombo(this, entries_2, entries_2.length, 140, 320, 100);        // this, String[] of entries, dropdown # of enteries shown at once, xPosition, yPosition, width
  dropdown_firingMode.setSelected(PApplet.parseInt(firingMode));                                                  // which entry to show as selected (first entry is 0, second is 1, etc.)
  panel_main.add(dropdown_firingMode);


  if (Serial.list().length > 0) {
    String[] entries_3 = append(Serial.list(), "Select to Override");
    dropdown_comPort = new GCombo(this, entries_3, entries_3.length, 460, 520, 120);  
    dropdown_comPort.setSelected(entries_3.length-1);
    panel_main.add(dropdown_comPort);
  }

  // Enable mouse over image changes
  G4P.setMouseOverEnabled(true);


  panelBackgroundImg = loadImage("Panel_Background.png");
  // create new window  (tag frame)
  window_main = new GWindow(this, "Control Panel", controlPanelWindowX, controlPanelWindowY, panelBackgroundImg, true, null);
//  window_main.setBackground(180);
  window_main.setOnTop(false);
  window_main.add(panel_main);
  window_main.addDrawHandler(this, "drawController");
  panel_main.setXY(0, 0);
}


public void updateControlPanels() {
  setLabelText(label_serialOut, "Serial Out: " + 'a' + strTargetx + strTargety + str(fire) + fireSelector + scanSelector);
  setLabelText(label_targetX, "Pan Servo Position: " + strTargetx);
  setLabelText(label_slider_tolerance, "Tolerance: " + str(tolerance));
  setLabelText(label_slider_trackColorTolerance, "Tolerance: " + str(trackColorTolerance));
  setLabelText(label_slider_safeColorTolerance, "Tolerance: " + str(safeColorTolerance));
  setLabelText(label_slider_safeColorMinSize, "Min Area: " + str(safeColorMinSize)); 
  setLabelText(label_slider_minBlobArea, "Min Size: " + str(minBlobArea));
  setLabelText(label_slider_nbDot, "Memory: " + str(nbDot));
  setLabelText(label_slider_antSens, "Sensitivity: " + str(antSens));
  setLabelText(label_slider_propX, "X Degree of Anitcipation: " + str(propX));
  setLabelText(label_slider_propY, "Y Degree of Anitcipation: " + str(propY));
  setLabelText(label_xMin, "xMin: " + str(xMin));
  setLabelText(label_xMax, "xMax: " + str(xMax));
  setLabelText(label_yMin, "yMin: " + str(yMin));
  setLabelText(label_yMax, "yMax: " + str(yMax));

  if (prevTargetX != targetX) {
    label_targetX.setOpaque(true);
  }
  else {
    label_targetX.setOpaque(false);
  }
  if (prevTargetY != targetY) {
    label_targetY.setOpaque(true);
  }
  else {
    label_targetY.setOpaque(false);
  }
  setLabelText(label_targetY, "Tilt Servo Position: " + strTargety);
  if (PApplet.parseBoolean(fire)) {
    label_fire.setOpaque(true);
    setLabelText(label_fire, "Firing");
  }
  else {
    label_fire.setOpaque(false);
    setLabelText(label_fire, "Not Firing");
  }
  if (firingMode) {
    setLabelText(label_fireSelector, "Semi-Automatic");
  }
  else {
    setLabelText(label_fireSelector, "Automatic");
  }
  if (!runWithoutArduino) {
    label_runWithoutArduino.setOpaque(true);
    setLabelText(label_runWithoutArduino, "Controller on " + serPortUsed);
  }
  else {
    label_runWithoutArduino.setOpaque(false);
    setLabelText(label_runWithoutArduino, "No Controller");
  }
  if (connecting) {
    label_runWithoutArduino.setOpaque(true);
    setLabelText(label_runWithoutArduino, "connecting...");
  }
  if (scanWhenIdle) {
    setLabelText(label_scanSelector, "Scan When Idle");
  }
  else{
    setLabelText(label_scanSelector, "Don't Scan When Idle");
  }
  checkbox_controlMode.setSelected(controlMode);
  checkbox_useInputDevice.setSelected(useInputDevice);
  checkbox_useArrowKeys.setSelected(useArrowKeys);
}


public void handleComboEvents(GCombo combo) {
  if (combo == dropdown_effect) {
    effect = dropdown_effect.selectedIndex();
  }
  if (combo == dropdown_firingMode) {
    firingMode = PApplet.parseBoolean(dropdown_firingMode.selectedIndex());
  }
  if (combo == dropdown_comPort) {
    if (dropdown_comPort.selectedIndex() < Serial.list().length) {
      if (!runWithoutArduino) {
        connecting = true;
        println("Manual override. Stopping old serial connection...");
        arduinoPort.stop();
        println("Stopped old serial connection.");
      }
      println("New COM port selected manually: " + Serial.list()[dropdown_comPort.selectedIndex()]);
      arduinoPort = new Serial(this, Serial.list()[dropdown_comPort.selectedIndex()], 4800);
      println("Serial Port used = " + Serial.list()[dropdown_comPort.selectedIndex()]);
      serPortUsed = Serial.list()[dropdown_comPort.selectedIndex()];
      runWithoutArduino = false;
      connecting = false;
    }
  }
}

public void handleSliderEvents(GSlider slider) {
  if (slider == slider_tolerance) {
    tolerance = slider_tolerance.getValue();
  }
  if (slider == slider_trackColorTolerance) {
    trackColorTolerance = slider_trackColorTolerance.getValue();
  }
  if (slider == slider_safeColorTolerance) {
    safeColorTolerance = slider_safeColorTolerance.getValue();
  }
  if (slider == slider_minBlobArea) {
    minBlobArea = slider_minBlobArea.getValue();
  }
  if (slider == slider_safeColorMinSize) {
    safeColorMinSize = slider_safeColorMinSize.getValue();
  }
  if (slider == slider_nbDot) {
    nbDot = slider_nbDot.getValue();
  }
  if (slider == slider_antSens) {
    antSens = slider_antSens.getValue();
  }
  if (slider == slider_propX) {
    propX = slider_propX.getValuef();
  }
  if (slider == slider_propY) {
    propY = slider_propY.getValuef();
  }
  if (slider == slider_smoothingFactor) {
    smoothingFactor = slider_smoothingFactor.getValuef();
  }
}


public void handleButtonEvents(GButton button) {
  if (button == button_viewCameraSettings && button.eventType == GButton.CLICKED) {
    viewCameraSettings();
  }
  if (button == button_setBackground && button.eventType == GButton.CLICKED) {
    setBackground();
  }
  if (button == button_selectColor && button.eventType == GButton.CLICKED) {
    selectColor();
  }
  if (button == button_selectSafeColor && button.eventType == GButton.CLICKED) {
    selectSafeColor();
  }
  if (button == button_openWebsite && button.eventType == GButton.CLICKED) {
    openWebsite();
  }
  if (button == button_playRandomSound && button.eventType == GButton.CLICKED) {
    playRandomSound();
  }
  if (button == button_saveSettings && button.eventType == GButton.CLICKED) {
    saveSettings();
  }
  if (button == button_loadSettings && button.eventType == GButton.CLICKED) {
    loadSettings();
  }
  if (button == button_saveAndExit && button.eventType == GButton.CLICKED) {
    saveSettings();
    delay(100);
    exit();
  }
  if (button == button_retryArduinoConnect && button.eventType == GButton.CLICKED) {
    if (!runWithoutArduino) {
      arduinoPort.stop();
    }
    runWithoutArduino = false;
    retryArduinoConnect();
  }
  if (button == button_configJoystick  && button.eventType == GButton.CLICKED) {
    configJoystick();
  }
  if(button == button_resetCalibration && button.eventType == GButton.CLICKED) {
    xMin = 0.0f;
    xMax = 180.0f;
    yMin = 0.0f;
    yMax = 180;
    xRatio = (camWidth / (xMax - xMin));   
    yRatio = (camHeight/ (yMax - yMin));      
  }
  if(button == button_flipX && button.eventType == GButton.CLICKED) {
    float oldxMin = xMin;
    float oldxMax = xMax;
    xMin = oldxMax;
    xMax = oldxMin;
    xRatio = (camWidth / (xMax - xMin));     
  }
  if(button == button_flipY && button.eventType == GButton.CLICKED) {
    float oldyMin = yMin;
    float oldyMax = yMax;
    yMin = oldyMax;
    yMax = oldyMin;
    yRatio = (camHeight/ (yMax - yMin));                         
  }
}

public void handleCheckboxEvents(GCheckbox cbox) {
  if (cbox == checkbox_leadTarget) {
    leadTarget = checkbox_leadTarget.isSelected();
  }
  if (cbox == checkbox_showRestrictedZones) {
    showRestrictedZones = checkbox_showRestrictedZones.isSelected();
  }
  if (cbox == checkbox_trackingColor) {
    trackingColor = checkbox_trackingColor.isSelected();
  }
  if (cbox == checkbox_safeColor) {
    safeColor = checkbox_safeColor.isSelected();
  }
  if (cbox == checkbox_trackingMotion) {
    trackingMotion = checkbox_trackingMotion.isSelected();
  }
  if (cbox == checkbox_showDifferentPixels) {
    showDifferentPixels = checkbox_showDifferentPixels.isSelected();
  }
  if (cbox == checkbox_showTargetBox) {
    showTargetBox = checkbox_showTargetBox.isSelected();
  }
  if (cbox == checkbox_mirrorCam) {
    mirrorCam = checkbox_mirrorCam.isSelected();
  }
  if (cbox == checkbox_controlMode) {
    controlMode = checkbox_controlMode.isSelected();
  }
  if (cbox == checkbox_safety) {
    safety = checkbox_safety.isSelected();
  }
  if (cbox == checkbox_showCameraView) {
    showCameraView = checkbox_showCameraView.isSelected();
  }
  if (cbox == checkbox_scanWhenIdle) {
    scanWhenIdle = checkbox_scanWhenIdle.isSelected();
  }
  if (cbox == checkbox_soundEffects) {
    soundEffects = checkbox_soundEffects.isSelected();
  }
  if (cbox == checkbox_activeSmoothing) {
    activeSmoothing = checkbox_activeSmoothing.isSelected();
  }
  if (cbox == checkbox_useInputDevice) {
    useInputDevice = checkbox_useInputDevice.isSelected();
  }
  if (cbox == checkbox_useArrowKeys) {
    useArrowKeys = checkbox_useArrowKeys.isSelected();
  }
}

public void drawController(GWinApplet appc, GWinData data) {
  //  (tag frame)
}

public void setLabelText(GLabel label, String text) {
  try {
    label.setText(text);
  }
  catch (NullPointerException e) { // ignore
  }
}

public void retryArduinoConnect() {
  connecting = true;
  if (!runWithoutArduino) {
//    try{
//      arduinoPort.stop();
//    }catch(Exception e) {
//      delay(10);
//    }
    // Find Serial Port that the arduino is on
    // The arduino is sending out a 'T' every 100 millisecs. Contributed by Don K.
    long millisStart;
    int i = 0;
    int len = Serial.list().length;    //get number of ports available
    println(Serial.list());      //print list of ports to screen

    println("Serial Port Count = " + len);  //print count of ports to screen
    if (len == 0) {
      runWithoutArduino = true;
      println("no Arduino detected. Will run without Arduino. Cheers");
    }
    for (i = 0; i < len; i++) {
      println("Testing port " + Serial.list()[i]);
      arduinoPort = new Serial(this, Serial.list()[i], 4800);      // Open 1st port in list
      millisStart = millis();
      while ( (millis () - millisStart) < 2000) ;  //wait for USB port reset (Guessed at 3 secs)
      // can't use delay() call in setup()
      arduinoPort.clear();        // empty buffer(incase of trash)
      arduinoPort.bufferUntil('T');                   //buffer until there is a 'T'
      millisStart = millis();
      while ( (millis () - millisStart) < 100) ;  //collect some chars
      if (arduinoPort.available() > 0)      //if we have a character
      {
        char c = arduinoPort.readChar();  //get the character
        if (c == 'T')        //if we got a 'T'
        {
          break;        //leave for loop
        }
      }
      else 
        arduinoPort.stop();      //if no 'T', stop port
      if (i == len - 1) {
        runWithoutArduino = true;
        println("no Arduino detected. Will run without Arduino. Cheers");
      }
    }
    if (!runWithoutArduino) {
      println("Serial Port used = " + Serial.list()[i]);
      serPortUsed = Serial.list()[i];
      millisStart = millis();
      while ( (millis () - millisStart) < 5000) ;
    }
  }
  connecting = false;
}
/* this section allows you to use a game controller to control your sentry.
 Setup using "InputDeviceSetupTool.pde"
 
 */

public void configJoystick() {
  println("Opening InputDeviceSetupTool.exe...");
  useInputDevice = false;
  checkbox_useInputDevice.setSelected(useInputDevice);
  inputDeviceIsSetup = false;
  try {
    open(dataPath("Input Device Setup Tool/InputDeviceSetupTool.exe"));
  }
  catch (Exception e) {
    e.printStackTrace();
    println("Could not open InputDeviceSetupTool.exe");
    return;
  }
}


public void setupInputDevice() {
  if (!inputDeviceIsSetup) {

    String[] loadData = new String[49];
    loadData = loadStrings("data/Input Device Setup Tool/settings_inputDevice.txt");

    controlIO = ControllIO.getInstance(this);

    boolean error = false;
    try {
      inputDevice = controlIO.getDevice(loadData[2]);
      // println("Device loaded successfully!");
    }
    catch (Exception e) {
      println("ERROR: Specified input device is not connected!");
      useInputDevice = false;
      checkbox_useInputDevice.setSelected(useInputDevice);
//      configJoystick();
      return;
    }

    inputDevice.setTolerance(0.025f);

    println("Device Selected = " + inputDevice.getName());

    int numButtons = inputDevice.getNumberOfButtons();
    for (int i = 0; i < numButtons; i++) {
      if (i < buttons.length) {
        buttons[i] = inputDevice.getButton(i);
      }
    }
    // println("numButtons = " + numButtons);

    int numSliders = inputDevice.getNumberOfSliders();
    for (int i = 0; i < numSliders; i++) {
      if (i < sliders.length) {
        sliders[i] = inputDevice.getSlider(i);
      }
    }  
    // println("numSliders = " + numSliders);


    for (int i = 0; i <= 29; i++) {

      if (loadData[i+6].equals("Fire")) {
        fire_buttons = (ControllButton[]) append(fire_buttons, buttons[i]);
      }
      else if (loadData[i+6].equals("Precise Aim")) {
        preciseAim_buttons = (ControllButton[]) append(preciseAim_buttons, buttons[i]);
      }
      else if (loadData[i+6].equals("Center Gun")) {
        centerGun_buttons = (ControllButton[]) append(centerGun_buttons, buttons[i]);
      }
      else if (loadData[i+6].equals("Auto Aim On")) {
        autoOn_buttons = (ControllButton[]) append(autoOn_buttons, buttons[i]);
      }
      else if (loadData[i+6].equals("Auto Aim Off")) {
        autoOff_buttons = (ControllButton[]) append(autoOff_buttons, buttons[i]);
      }
      else if (loadData[i+6].equals("Input Dev On/Off")) {
        inputToggle_buttons = (ControllButton[]) append(inputToggle_buttons, buttons[i]);
      }
      else if (loadData[i+6].equals("Random Sound")) {
        randomSound_buttons = (ControllButton[]) append(randomSound_buttons, buttons[i]);
      }
    }


    for (int i = 0; i <= 9; i++) {
      if (loadData[i+39].equals("Pan")) {
        pan_sliders = (ControllSlider[]) append(pan_sliders, sliders[i]);
      }
      else if (loadData[i+39].equals("Tilt")) {
        tilt_sliders = (ControllSlider[]) append(tilt_sliders, sliders[i]);
      }
      else if (loadData[i+39].equals("Pan (Invert)")) {
        panInvert_sliders = (ControllSlider[]) append(panInvert_sliders, sliders[i]);
      }
      else if (loadData[i+39].equals("Tilt (Invert)")) {
        tiltInvert_sliders = (ControllSlider[]) append(tiltInvert_sliders, sliders[i]);
      }
    }

    inputDeviceIsSetup = true;
  }
}

public void updateInputDevice() {
  if (!inputDeviceIsSetup) {
    setupInputDevice();
  }
  else {
    float xMotionValue = 0;
    for (int i = 0; i < pan_sliders.length; i++) {
      xMotionValue = xMotionValue + pan_sliders[i].getValue();
    }
    for (int i = 0; i < panInvert_sliders.length; i++) {
      xMotionValue = xMotionValue - panInvert_sliders[i].getValue();
    }

    float yMotionValue = 0;
    for (int i = 0; i < tilt_sliders.length; i++) {
      yMotionValue = yMotionValue + tilt_sliders[i].getValue();
    }
    for (int i = 0; i < tiltInvert_sliders.length; i++) {
      yMotionValue = yMotionValue - tiltInvert_sliders[i].getValue();
    }




    boolean triggerButtonValue = false;
    for (int i = 0; i < fire_buttons.length; i++) {
      if (fire_buttons[i].pressed()) {
        triggerButtonValue = true;
      }
    }

    boolean centerButtonValue = false;
    for (int i = 0; i < centerGun_buttons.length; i++) {
      if (centerGun_buttons[i].pressed()) {
        centerButtonValue = true;
      }
    }

    boolean precisionButtonValue = false;
    for (int i = 0; i < preciseAim_buttons.length; i++) {
      if (preciseAim_buttons[i].pressed()) {
        precisionButtonValue = true;
      }
    }


    float aimSensitivityX = map(pow(abs(xMotionValue), 2), 0.0f, 1.0f, 1.0f, camWidth/10);    // set the sensitivity coeficcient for horizontal axis. Based on a quadratic correlation.
    float aimSensitivityY = map(pow(abs(yMotionValue), 2), 0.0f, 1.0f, 1.0f, camWidth/10);    // set the sensitivity coeficcient for vertical axis. Based on a quadratic correlation.

    if (precisionButtonValue) {         // aim precisely if appropriate button is pressed
      aimSensitivityX *= 0.25f;
      aimSensitivityY *= 0.25f;
    }

    xPosition += aimSensitivityX * xMotionValue;   // update the position of the crosshairs
    yPosition += aimSensitivityY * yMotionValue;

    xPosition = constrain(xPosition, 0, camWidth);   // don't let the crosshairs leave the camera view
    yPosition = constrain(yPosition, 0, camHeight);  

    if (centerButtonValue) {       // center the crosshairs if appropriate button is pressed
      xPosition = camWidth/2;
      yPosition = camHeight/2;
    }

    if (triggerButtonValue) {   // fire if appropriate button is pressed
      fire = 1;
    }
    else {
      fire = 0;
    }

    targetX = PApplet.parseInt((xPosition/xRatio)+xMin);                 // calculate position to go to based on mouse position
    targetY = PApplet.parseInt(((camHeight-yPosition)/yRatio)+yMin);     //
    displayX = PApplet.parseInt(xPosition);
    displayY = PApplet.parseInt(yPosition);
  }
}

public void checkInputDevice() {
  if (!inputDeviceIsSetup) {
    setupInputDevice();
  }
  else {
    boolean manualModeButtonValue = false;
    for (int i = 0; i < autoOff_buttons.length; i++) {
      if (autoOff_buttons[i].pressed()) {
        manualModeButtonValue = true;
      }
    }

    boolean autonomousModeButtonValue = false;
    for (int i = 0; i < autoOn_buttons.length; i++) {
      if (autoOn_buttons[i].pressed()) {
        autonomousModeButtonValue = true;
      }
    }

    boolean soundEffectButtonValue = false;
    for (int i = 0; i < randomSound_buttons.length; i++) {
      if (randomSound_buttons[i].pressed()) {
        soundEffectButtonValue = true;
      }
    }

    boolean activeButtonValue = false;
    for (int i = 0; i < inputToggle_buttons.length; i++) {
      if (inputToggle_buttons[i].pressed()) {
        activeButtonValue = true;
      }
    }

    if (manualModeButtonValue) { // go to manual mode if appropriate button is pressed
      controlMode = false;
    }
    if (autonomousModeButtonValue) { // go to autonomous mode if appropriate button is pressed
      controlMode = true;
    }
    if (soundEffectButtonValue) {  // play a random sound effect if appropriate button is pressed
      randomIdleSound();
      while (soundEffectButtonValue) {
        soundEffectButtonValue = false;
        for (int i = 0; i < randomSound_buttons.length; i++) {
          if (randomSound_buttons[i].pressed()) {
            soundEffectButtonValue = true;
          }
        }
      }
    }
    if (activeButtonValue) {
      useInputDevice = !useInputDevice;
      while (activeButtonValue) {
        activeButtonValue = false;
        for (int i = 0; i < inputToggle_buttons.length; i++) {
          if (inputToggle_buttons[i].pressed()) {
            activeButtonValue = true;
          }
        }
      }
    }
  }
}

public void saveSettings() {
  
  String[] saveData = new String[40];
  
  saveData[0] = str(camWidth);
  saveData[1] = str(camHeight);
  saveData[2] = str(xMin);
  saveData[3] = str(xMax);
  saveData[4] = str(yMin);
  saveData[5] = str(yMax);
  saveData[6] = str(effect);
  saveData[7] = str(mirrorCam);
  saveData[8] = str(minBlobArea);
  saveData[9] = str(tolerance); 
  saveData[10] = str(runWithoutArduino);
  saveData[11] = str(smoothingFactor);
  saveData[12] = str(activeSmoothing);
  saveData[13] = str(showDifferentPixels);
  saveData[14] = str(showTargetBox);
  saveData[15] = str(showCameraView);
  saveData[16] = str(firingMode);
  saveData[17] = str(safety);
  saveData[18] = str(controlMode);
  saveData[19] = str(soundEffects);
  saveData[20] = str(scanWhenIdle);
  saveData[21] = str(trackingMotion);
  saveData[22] = str(showRestrictedZones);
  saveData[23] = str(trackingColor);
  saveData[24] = str(trackColorTolerance);
  saveData[25] = str(trackColorRed);
  saveData[26] = str(trackColorGreen);
  saveData[27] = str(trackColorBlue);
  saveData[28] = str(safeColor);
  saveData[29] = str(safeColorMinSize);
  saveData[30] = str(safeColorTolerance);
  saveData[31] = str(safeColorRed);
  saveData[32] = str(safeColorGreen);
  saveData[33] = str(safeColorBlue);
  saveData[34] = str(useInputDevice);
  saveData[35] = str(leadTarget);
  saveData[36] = str(nbDot);
  saveData[37] = str(antSens);
  saveData[38] = str(propX);
  saveData[39] = str(propY);
  
  saveStrings("data/settings.txt", saveData);
  println("Successfully saved settings to \"settings.txt\"");
  
}

public void loadSettings() {
  
  String[] loadData = new String[40];
  loadData = loadStrings("settings.txt");
  
//  camWidth = int(loadData[0]);
//  camHeight = int(loadData[1]);

  xMin = PApplet.parseFloat(loadData[2]);
  xMax = PApplet.parseFloat(loadData[3]);
  yMin = PApplet.parseFloat(loadData[4]);
  yMax = PApplet.parseFloat(loadData[5]);
  
  xRatio = (camWidth / (xMax - xMin)); 
  yRatio = (camHeight/ (yMax - yMin));  
  
  effect = PApplet.parseInt(loadData[6]);
  mirrorCam = PApplet.parseBoolean(loadData[7]);
  minBlobArea = PApplet.parseInt(loadData[8]);
  tolerance =  PApplet.parseInt(loadData[9]);
  runWithoutArduino = PApplet.parseBoolean(loadData[10]);
  smoothingFactor = PApplet.parseFloat(loadData[11]);
  activeSmoothing = PApplet.parseBoolean(loadData[12]);
  showDifferentPixels = PApplet.parseBoolean(loadData[13]);
  showTargetBox = PApplet.parseBoolean(loadData[14]);
  showCameraView = PApplet.parseBoolean(loadData[15]);
  firingMode = PApplet.parseBoolean(loadData[16]);
  safety = PApplet.parseBoolean(loadData[17]);
  controlMode = PApplet.parseBoolean(loadData[18]);
  soundEffects = PApplet.parseBoolean(loadData[19]);
  scanWhenIdle = PApplet.parseBoolean(loadData[20]);
  trackingMotion = PApplet.parseBoolean(loadData[21]);
  showRestrictedZones = PApplet.parseBoolean(loadData[22]);
  trackingColor = PApplet.parseBoolean(loadData[23]);
  trackColorTolerance = PApplet.parseInt(loadData[24]);
  trackColorRed = PApplet.parseInt(loadData[25]);
  trackColorGreen = PApplet.parseInt(loadData[26]);
  trackColorBlue = PApplet.parseInt(loadData[27]);
  safeColor = PApplet.parseBoolean(loadData[28]);
  safeColorMinSize = PApplet.parseInt(loadData[29]);
  safeColorTolerance = PApplet.parseInt(loadData[30]);
  safeColorRed = PApplet.parseInt(loadData[31]);
  safeColorGreen = PApplet.parseInt(loadData[32]);
  safeColorBlue = PApplet.parseInt(loadData[33]);
  useInputDevice = PApplet.parseBoolean(loadData[34]);
  leadTarget = PApplet.parseBoolean(loadData[35]);
  nbDot = PApplet.parseInt(loadData[36]);
  antSens = PApplet.parseInt(loadData[37]);
  propX = PApplet.parseFloat(loadData[38]);
  propY = PApplet.parseFloat(loadData[39]);
  
  println("Successfully loaded settings from \"settings.txt\"");
}

Minim minim;

AudioSnippet s1;
AudioSnippet s2;
AudioSnippet s3;
AudioSnippet s4;
AudioSnippet s5;
AudioSnippet s6;
AudioSnippet s7;
AudioSnippet s8;
AudioSnippet s9;
AudioSnippet s10;
AudioSnippet s11;
AudioSnippet s12;
AudioSnippet s13;
AudioSnippet s14;
AudioSnippet s15;
AudioSnippet s16;
AudioSnippet s17;
AudioSnippet s18;
AudioSnippet s19;
AudioSnippet s20;
AudioSnippet s21;

int soundTimer = 0;
int soundInterval = 1000;


public void loadSounds() {
  s1 = minim.loadSnippet("data/your business is appreciated.wav");
  s2 = minim.loadSnippet("data/who's there.wav");
  s3 = minim.loadSnippet("data/there you are.wav");
  s4 = minim.loadSnippet("data/there you are(2).wav");
  s5 = minim.loadSnippet("data/target lost.wav");
  s6 = minim.loadSnippet("data/target aquired.wav");
  s7 = minim.loadSnippet("data/sleep mode activated.wav");
  s8 = minim.loadSnippet("data/sentry mode activated.wav");
  s9 = minim.loadSnippet("data/no hard feelings.wav");
  s10 = minim.loadSnippet("data/is anyone there.wav");
  s11 = minim.loadSnippet("data/i see you.wav");
  s12 = minim.loadSnippet("data/i dont hate you.wav");
  s13 = minim.loadSnippet("data/i dont blame you.wav");
  s14 = minim.loadSnippet("data/hey its me.wav");
  s15 = minim.loadSnippet("data/hello.wav");
  s16 = minim.loadSnippet("data/gotcha.wav");
  s17 = minim.loadSnippet("data/dispensing product.wav");
  s18 = minim.loadSnippet("data/deploying.wav");
  s19 = minim.loadSnippet("data/could you come over here.wav");
  s20 = minim.loadSnippet("data/are you still there.wav");
  s21 = minim.loadSnippet("data/activated.wav");
}

public void playSound(int sound) {
  if(soundEffects) {
    if(sound == 1) {
      s1.rewind();
      s1.play();
    }
    if(sound == 2) {
      s2.rewind();
      s2.play();
    }
    if(sound == 3) {
      s3.rewind();
      s3.play();
    }
    if(sound == 4) {
      s4.rewind();
      s4.play();
    }
    if(sound == 5) {
      s5.rewind();
      s5.play();
    }
    if(sound == 6) {
      s6.rewind();
      s6.play();
    }
    if(sound == 7) {
      s7.rewind();
      s7.play();
    }
    if(sound == 8) {
      s8.rewind();
      s8.play();
    }
    if(sound == 9) {
      s9.rewind();
      s9.play();
    }
    if(sound == 10) {
      s10.rewind();
      s10.play();
    }
    if(sound == 11) {
      s11.rewind();
      s11.play();
    }
    if(sound == 12) {
      s12.rewind();
      s12.play();
    }
    if(sound == 13) {
      s13.rewind();
      s13.play();
    }
    if(sound == 14) {
      s14.rewind();
      s14.play();
    }
    if(sound == 15) {
      s15.rewind();
      s15.play();
    }
    if(sound == 16) {
      s16.rewind();
      s16.play();
    }
    if(sound == 17) {
      s17.rewind();
      s17.play();
    }
    if(sound == 18) {
      s18.rewind();
      s18.play();
    }
    if(sound == 19) {
      s19.rewind();
      s19.play();
    }
    if(sound == 20) {
      s20.rewind();
      s20.play();
    }
    if(sound == 21) {
      s21.rewind();
      s21.play();
    }
  }
}

public void randomIdleSound() {
  if(soundEffects) {
    int sound = PApplet.parseInt(random(1, 11));
    if(sound == 1) {
      s2.rewind();
      s2.play();
    }
    if(sound == 2) {
      s7.rewind();
      s7.play();
    }
    if(sound == 3) {
      s9.rewind();
      s9.play();
    }
    if(sound == 4) {
      s10.rewind();
      s10.play();
    }
    if(sound == 5) {
      s11.rewind();
      s11.play();
    }
    if(sound == 6) {
      s12.rewind();
      s12.play();
    }
    if(sound == 7) {
      s13.rewind();
      s13.play();
    }
    if(sound == 8) {
      s14.rewind();
      s14.play();
    }
    if(sound == 9) {
      s19.rewind();
      s19.play();
    }
    if(sound == 10) {
      s20.rewind();
      s20.play();
    }
  }
}
  static public void main(String args[]) {
    PApplet.main(new String[] { "--bgcolor=#F0F0F0", "PSG_Processing_Code" });
  }
}
