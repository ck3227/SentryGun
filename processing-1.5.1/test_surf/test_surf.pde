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
import JMyron.*;

public Surf mSurfRef;
public Surf mSurfCap;
public Map<SURFInterestPoint,SURFInterestPoint> mAMatchingPoints;
public Map<SURFInterestPoint,SURFInterestPoint> mBMatchingPoints;
public Map<SURFInterestPoint,SURFInterestPoint> pointsA;
public Map<SURFInterestPoint,SURFInterestPoint> pointsB;
public BufferedImage imageRef;
public  BufferedImage captureImage;
JMyron camInput;
public int camWidth = 640;                   //   camera width (pixels),   usually 160*n
public int camHeight = 480;                  //   camera height (pixels),  usually 120*n
int[] currFrame;

void setup() {
  size(camWidth, camHeight);
  camInput = new JMyron();
  camInput.start(camWidth, camHeight);
  captureImage = new BufferedImage(camWidth, camHeight, BufferedImage.OPAQUE);
  
  try {
  imageRef = ImageIO.read(new File("C:/Users/Martin/Pictures/robot_live.jpg"));
  } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
  }
  mSurfRef = new Surf(imageRef);
  camInput.update();
  currFrame = camInput.image();
  captureImage.setRGB(0, 0, camWidth, camHeight, currFrame, 0, camWidth);
 }


void draw() {    
   camInput.update();
   currFrame = camInput.image();
   captureImage.setRGB(0, 0, camWidth, camHeight, currFrame, 0, camWidth);
   mSurfCap = new Surf(captureImage);
   mBMatchingPoints = mSurfCap.getMatchingPoints(mSurfRef,true);
   System.out.println(mBMatchingPoints.size());
   }


public void stop() {
    camInput.stop();
}
