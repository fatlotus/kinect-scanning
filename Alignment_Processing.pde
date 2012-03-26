import librarytests.*;
import org.openkinect.*;
import org.openkinect.processing.*;
import processing.video.*;

class FakeKinect extends Kinect {
  int frame;
  
  public FakeKinect(PApplet parent) {
    super(parent);
  }
  
  public void start() {
    frame = 0;
  }
  
  public void tilt(float angle) { }
  
  public void enableDepth(boolean v) { }
  public void processDepthImage(boolean v) { }
  public void enableIR(boolean v) { }
  public void enableRGB(boolean v) { }
  
  public PImage getVideoImage() {
    return loadImage("/Users/jarcher/Desktop/Frames/Color/" + frame + ".jpg");
  }
  
  public int[] getRawDepth() {
    frame++;
    
    int[] depths = new int[w * h];
    PImage loaded = loadImage("/Users/jarcher/Desktop/Frames/Depth/" + frame + ".jpg");
    
    loaded.loadPixels();
    
    for (int i = 0; i < depths.length; i++) {
      int r = (loaded.pixels[i] >>> 16) & 0xff;
      int g = (loaded.pixels[i] >>> 8) & 0xff;
      
      depths[i] = (r | (g << 8));
      
      if (i == 10000) {
        println("[depth: " + (r | (gp << 8)) + "]");
      }
    }
    
    return depths;
  }
}

class Particle {
  float x, y, z;
  color shade;
  int age, weight, framesInExistence;

  public Particle(int px, int py, int depth, PMatrix3D cam) {
    this(px, py, depth, null, cam);
  }

  public Particle(int px, int py, int depth, PImage rgbImage, PMatrix3D cam) {
    PVector world = depthToWorldWithTransform(px, py, depth, cam);

    x = world.x;
    y = world.y;
    z = world.z;

    weight = 1;

    framesInExistence = 0;
    age = 0;
    
    // shade = #FFFFFF;
    
    if (rgbImage != null) {
      shade = extractColorFromWorld(rgbImage, world);
      // if (alpha(shade) == 0) age = 5000;
    } else {
      shade = #FFFFFF;
    }
  }

  public int getDistanceToCamera() {
    return metersToRawDepth(z);
  }

  public float distanceTo(float fx, float fy, float fz) {
    float dx = x - fx; if (dx < 0) dx = -dx;
    float dy = y - fy; if (dy < 0) dy = -dy;
    float dz = z - fz; if (dz < 0) dz = -dz;
    
    return (dx + dy + dz) / 3f;
    // return sqrt(dx*dx + dy*dy + dz*dz);
  }

  public float colorDifference(Particle p2) { // higher values imply less correlation, and thus less likelihood to be removed.
    float r = red(p2.shade) - red(shade);
    float g = green(p2.shade) - green(shade);
    float b = blue(p2.shade) - blue(shade);

    return (r*r + b*b + g*g) * weight*p2.weight;
  }

  public void factorIn(Particle p2) {
    float r = red(shade) * weight   + red(p2.shade) * p2.weight;
    float g = green(shade) * weight + green(p2.shade) * p2.weight;
    float b = blue(shade) * weight  + blue(p2.shade) * p2.weight;

    int sum = weight + p2.weight;
    r /= sum;
    g /= sum;
    b /= sum;

    shade = color((int)r, (int)g, (int)b);

    x = (x * weight + p2.x * p2.weight) / sum;
    y = (y * weight + p2.y * p2.weight) / sum;
    z = (z * weight + p2.z * p2.weight) / sum;

    weight = sum;
  }
}

class ParticleBox {
  ArrayList<Particle> particles;
  int x, y, z;
  int lastTimePruned;

  public ParticleBox(float fx, float fy, float fz) {
    x = (int)(floor(fx * 2));
    y = (int)(floor(fy * 2));
    z = (int)(floor(fz * 2));
    particles = new ArrayList<Particle>();
    lastTimePruned = 0;
  }

  public boolean contains(float xs, float ys, float zs) {
    return ((int)(floor(xs * 2)) == x) && ((int)(floor(ys * 2)) == y) && ((int)(floor(zs * 2)) == z);
  }

  public int hashCode() {
    return x ^ y ^ z;
  }

  public void pruneRedundantParticles(int[] rawDepths) {
    if (lastTimePruned == time) return;
    lastTimePruned = time;
    
    int i, j;

    for (i = 0; i < particles.size(); i++) {
      Particle p = particles.get(i);
      p.framesInExistence++;

      if (p.age > 50) { 
        particles.remove(i--); 
        continue;
      }

      PVector world = transformWorldToCamera(new PVector(p.x, p.y, p.z));

      int px = worldToX(world);
      int py = worldToY(world);
      int depth = metersToRawDepth(world.z);

      if (px < 0 || px >= w || py < 0 || py >= h) {
        /* do nothing */
      } 
      else if (depth <= rawDepths[py * w + px] - 2) {
        p.age += 10;
      } 
      else if (depth > rawDepths[py * w + px] + 10) {
        /* do nothing */
      } 
      else {
        p.age++;
      }

      for (j = i + 1; j < particles.size(); j++) {
        Particle p2 = particles.get(j);
        if (p.distanceTo(p2.x, p2.y, p2.z) < 0.03 && p.colorDifference(p2) < 100000) {
          particles.remove(j--);
          p.factorIn(p2);
        }
      }
    }
  }

  public Particle getNearestParticleInsideBox(float fx, float fy, float fz) {
    float distance = 1e10;
    Particle minParticle = null;

    for (Particle p : particles) {
      if (p.distanceTo(fx, fy, fz) < distance) {
        minParticle = p;
        distance = p.distanceTo(fx, fy, fz);
      }
    }

    return minParticle;
  }

  public color averageColor() {
    if (particles.size() == 0) return color(0, 0, 0, 0);

    int sumRed = 0, sumGreen = 0, sumBlue = 0;
    
    for (Particle p : particles) {
      sumRed += red(p.shade);
      sumGreen += green(p.shade);
      sumBlue += blue(p.shade);
    }

    return color(sumRed / particles.size(), sumGreen / particles.size(), sumBlue / particles.size(), 255 * max(0, 1 - 3.0/sqrt(particles.size())));
  }
}

class ParticleCollection implements Iterable<Particle> {
  class Address {
    private int x, y, z;
    
    public Address(float fx, float fy, float fz) {
      this((int)(fx * 2), (int)(fy * 2), (int)(fz * 2));
    }
    
    public Address(int px, int py, int pz) {
      x = px;
      y = py;
      z = pz;
    }
    
    public int hashCode() {
      return (x << 16) ^ (y << 8) ^ z;
    }
    
    public boolean equals(Object other) {
      if (other instanceof Address) {
        Address a = (Address)other;
        return a.x == x && a.y == y && a.z == z;
      } else {
        return false;
      }
    }
  }
  HashMap<Address, ParticleBox> particleBoxes;

  public ParticleCollection() {
    particleBoxes = new HashMap<Address, ParticleBox>();
  }

  public ParticleBox getParticleBoxAt(float x, float y, float z) {
    return particleBoxes.get(new Address(x, y, z));
    
    /*
    
    for (ParticleBox particleBox : particleBoxes) {
      if (particleBox.contains(x, y, z)) {
        return particleBox;
      }
    }

    return null;
    */
  }

  public Particle getNearestParticle(float x, float y, float z) {
    ParticleBox b = getParticleBoxAt(x, y, z);
    
    if (b == null) return null;
    
    return b.getNearestParticleInsideBox(x, y, z);
  }

  public void addParticle(Particle p) {
    ParticleBox b = getParticleBoxAt(p.x, p.y, p.z);
    if (b == null) {
      b = new ParticleBox(p.x, p.y, p.z);
      particleBoxes.put(new Address(p.x, p.y, p.z), b);
    }
    
    b.particles.add(p);
  }

  public void pruneRedundantParticles(int[] rawDepths) {
    for (int px = 0; px < w; px += 30) {
      for (int py = 0; py < h; py += 30) {
        PVector world = depthToWorldWithTransform(px, py, rawDepths[px + py * w], cameraTransform); 
        ParticleBox b = getParticleBoxAt(world.x, world.y, world.z);
        
        if (b != null)
          b.pruneRedundantParticles(rawDepths);
      }
    }
    
    /*
    for (ParticleBox b : particleBoxes.values()) {
      b.pruneRedundantParticles(rawDepths);
    }
    */
  }

  public Iterator<Particle> iterator() {
    final Iterator<ParticleBox> boxes = particleBoxes.values().iterator();
    
    return new Iterator<Particle>() {
      private Iterator<Particle> intern = boxes.next().particles.iterator();
      
      public boolean hasNext() {
        return intern.hasNext() || (boxes.hasNext());
      }
      
      public void remove() {
        throw new UnsupportedOperationException();
      }

      public Particle next() {
        while (!intern.hasNext() && boxes.hasNext())
          intern = boxes.next().particles.iterator(); // FIXME
        
        if (intern.hasNext()) {
          Particle next = intern.next();
          
          while (!intern.hasNext () && boxes.hasNext())
            intern = boxes.next().particles.iterator(); // FIXME

          return next;
        } 
        else {
          throw new java.util.NoSuchElementException();
        }
      }
    };
  }

  public int size() {
    int s = 0;
    for (ParticleBox b : particleBoxes.values()) {
      s += b.particles.size();
    }
    return s;
  }

  public int getNumberOfBoxes() {
    return particleBoxes.size();
  }
}

float[] depthToWorldTable = new float[2048];
ParticleCollection particles;
Kinect kinect;
int w = 640, h = 480;
PFont mediumHelvetica;
int time, delta, previousFrame;
PMatrix3D cameraTransform;
PMatrix3D antiCameraTransform;
MovieMaker movie;

void keyPressed() {
  if (key == 'e') {
    movie.finish();
    movie = null;
  }
}

void setup() {
  size(800, 600, P3D);
  kinect = new FakeKinect(this);
  kinect.start();
  kinect.enableDepth(true);
  kinect.tilt(0);
  kinect.processDepthImage(false);
  kinect.enableRGB(true);

  particles = new ParticleCollection();

  for (int i = 0; i < 2048; i++) {
    depthToWorldTable[i] = rawDepthToMeters(i);
  }

  mediumHelvetica = loadFont("Helvetica-48.vlw");

  cameraTransform = new PMatrix3D();
  antiCameraTransform = new PMatrix3D();
  
  beginCamera();
  perspective();
  endCamera();
  
  frame.setTitle("Kinect Alignment Processing");
  
  movie = new MovieMaker(this, width, height, "/Users/jarcher/Desktop/Recordings/Current.mov", 15);
  
  previousFrame = millis();
}

void draw() {
  int currentTime = millis();
  delta = currentTime - previousFrame;
  previousFrame = currentTime;
  
  frame.setTitle("Kinect Alignment Processing (" + (int)frameRate + " Hz)");
  
  int[] depths = kinect.getRawDepth();
  PImage videoImage = kinect.getVideoImage();
  videoImage.loadPixels();
  
  if ((time % 1) == 0)
    particles.pruneRedundantParticles(depths);
  
  int indexOfNextTransform;
  
  int transformLimit = 5; 
  
  if (time > 40) {
    do {
      PMatrix3D[] transforms = new PMatrix3D[13];
      
      for (int i = 0; i < transforms.length; i++) {
        transforms[i] = cameraTransform.get();
      }
      
      transforms[1 ].translate(0, 0, -0.0005 * delta);
      transforms[2 ].translate(0, 0,  0.0005 * delta);
      transforms[3 ].translate(0, -0.0005 * delta, 0);
      transforms[4 ].translate(0,  0.0005 * delta, 0);
      transforms[5 ].translate(-0.0005 * delta, 0, 0);
      transforms[6 ].translate( 0.0005 * delta, 0, 0); 
      transforms[7 ].rotateY( 0.0002f * delta);
      transforms[8 ].rotateY(-0.0002f * delta);
      transforms[9 ].rotateX( 0.0002f * delta);
      transforms[10].rotateX(-0.0002f * delta);
      transforms[11].rotateZ( 0.0002f * delta);
      transforms[12].rotateZ(-0.0002f * delta);
      
      indexOfNextTransform = chooseOptimalTransform(transforms, depths);
      
      cameraTransform = transforms[indexOfNextTransform];
    } while (indexOfNextTransform != 0 && transformLimit-- > 0);
  }
  
  antiCameraTransform = cameraTransform.get();
  antiCameraTransform.invert();

  for (int x = ((time % 17) % 10); x < w; x += 10) {
    for (int y = ((time % 13) % 10); y < h; y += 10) {
      particles.addParticle(new Particle(x, y, depths[x + y * w], videoImage, cameraTransform));
    }
  }

  background(0);
  fill(255);
  pushMatrix();
  translate(width/2, height/2, 0);
  // rotateX(-PI / 2);
  scale(200);
  scale(1, 1, -1);
  // translate(0, 2, 0);
  
  PMatrix3D t = antiCameraTransform;
  
  /* */
  applyMatrix(
  t.m00, t.m01, t.m02, t.m03, 
  t.m10, t.m11, t.m12, t.m13, 
  t.m20, t.m21, t.m22, t.m23, 
  t.m30, t.m31, t.m32, t.m33
    );
  /* */
  
  translate(0, 0, +0.1f);
  
  /* */
  for (ParticleBox b : particles.particleBoxes.values()) {
   pushMatrix();
   translate(b.x / 2, b.y / 2, b.z / 2);
   translate(0.5, 0.5, 0.5);
   noFill();
   stroke(b.averageColor());
   box(1);
   popMatrix();
   }
  /* */
  
  /* */
  for (Particle p : particles) {
    if (p.framesInExistence <= 5) continue;

    pushMatrix();
    // noStroke();
    noStroke(); // (p.weight % 127) + 127, (p.age % 127) + 127, 0);
    fill(p.shade); // color(p.getDistanceToCamera() % 255, (p.getDistanceToCamera() % 2) * 100 + 100, 0));
    translate(p.x, p.y, p.z);
    // point(0, 0);
    box(0.05f); // * (1 - 1/(sqrt(p.weight))));
    popMatrix();
  }
  /* */

  pushMatrix();

  t = cameraTransform;

  applyMatrix(
  t.m00, t.m01, t.m02, t.m03, 
  t.m10, t.m11, t.m12, t.m13, 
  t.m20, t.m21, t.m22, t.m23, 
  t.m30, t.m31, t.m32, t.m33
    );

  fill(255, 255, 0);
  stroke(200, 200, 0);
  box(0.1f);

  line(0, 0, 0, 0, 0, 10);

  popMatrix();

  popMatrix();

  fill(color(230));
  textFont(mediumHelvetica, 20);
  text("Particles: " + particles.size(), 30, 40);
  text("Voxel Containers: " + particles.getNumberOfBoxes(), 30, 80);
  text("Particles per Voxel: " + particles.size() / particles.getNumberOfBoxes(), 30, 120);
  
  int framesToGenerate = ((delta - 1) / 60) + 1;
  
  for (int i = 0; i < framesToGenerate; i++) {
    movie.addFrame();
  }
  
  time++;
}

int chooseOptimalTransform(PMatrix3D[] candidates, int[] rawDepths) {
  float smallestDifferenceSum = MAX_FLOAT;
  PMatrix3D bestTransform = null;
  int i = 0;
  int bestIndex = 0;

  for (PMatrix3D candidate : candidates) {
    float sum = 0;
    for (int x = ((time % 17) % 10); x < w; x += 20) {
      for (int y = ((time % 13) % 10); y < h; y += 20) {
        PVector world = depthToWorldWithTransform(x, y, rawDepths[x + y * w], candidate);
        
        Particle nearest = particles.getNearestParticle(world.x, world.y, world.z);

        if (nearest != null) {
          sum += nearest.distanceTo(world.x, world.y, world.z);
        } 
        else {
          sum += 0.1;
        }
      }
    }

    if (sum < smallestDifferenceSum) {
      bestTransform = candidate;
      smallestDifferenceSum = sum;
      bestIndex = i;
    }
    
    i++;
  }

  // println("Best (index): " + bestIndex + " w/" + smallestDifferenceSum);

  return bestIndex;
}

// Taken from: http://nicolas.burrus.name/index.php/Research/KinectCalibration
color extractColorFromWorld(PImage rgbImage, PVector world2) {
  PVector world = new PVector();
  antiCameraTransform.mult(world2, world);
  
  double x = (
  9.9984628826577793e-01 * world.x +
    1.2635359098409581e-03 * world.y + 
    -1.7487233004436643e-02 * world.z +
    1.9985242312092553e-02
    );
  double y = (
  -1.4779096108364480e-03 * world.x +
    9.9992385683542895e-01 * world.y +
    -1.2251380107679535e-02 * world.z +
    -7.4423738761617583e-04
    );
  double z = (
  1.7470421412464927e-02 * world.x + 
    1.2275341476520762e-02 * world.y +
    9.9977202419716948e-01 * world.z +
    -1.0916736334336222e-02
    );
  double fx_rgb = 5.2921508098293293e+02;
  double fy_rgb = 5.2556393630057437e+02;
  double cx_rgb = 3.2894272028759258e+02;
  double cy_rgb = 2.6748068171871557e+02;

  double mx = (x * fx_rgb / z) + cx_rgb;
  double my = (y * fy_rgb / z) + cy_rgb;

  int px = (int)mx;
  int py = (int)my;

  if (px < 0 || py < 0 || px >= rgbImage.width || py >= rgbImage.height) return color(0, 0, 0, 0);

  return rgbImage.pixels[px + py * w];
}

// These functions come from: http://graphics.stanford.edu/~mdfisher/Kinect.html
float rawDepthToMeters(int depthValue) {
  if (depthValue < 2047) {
    return (float)(1.0 / ((double)(depthValue) * -0.0030711016 + 3.3309495161));
  }
  return 0.0f;
}

int metersToRawDepth(float depth) {
  if (depth == 0) return 2048;

  float z = ((1.0 / depth) - 3.3309495161) / -0.0030711016;

  if (z > 2048 || z <= 0) {
    z = 0;
  }
  return (int)z;
}

PVector depthToWorldPrevious(int x, int y, int depthValue) {
  final double fx_d = 1.0 / 5.9421434211923247e+02;
  final double fy_d = 1.0 / 5.9104053696870778e+02;
  final double cx_d = 3.3930780975300314e+02;
  final double cy_d = 2.4273913761751615e+02;

  PVector result = new PVector();
  double depth =  depthToWorldTable[depthValue];//rawDepthToMeters(depthValue);
  result.x = (float)((x - cx_d) * depth * fx_d);
  result.y = (float)((y - cy_d) * depth * fy_d);
  result.z = (float)(depth);
  return result;
}

PVector depthToWorldWithTransform(int x, int y, int depthValue, PMatrix3D cameraTransform) {
  PVector world = depthToWorldPrevious(x, y, depthValue);
  PVector result = new PVector();

  cameraTransform.mult(world, result);
  return result;
}

PVector transformWorldToCamera(PVector world) {
  PVector newWorld = new PVector();
  antiCameraTransform.mult(world, newWorld);

  return newWorld;
}

int worldToX(PVector world) {
  final double fx_d = 1.0 / 5.9421434211923247e+02;
  final double fy_d = 1.0 / 5.9104053696870778e+02;
  final double cx_d = 3.3930780975300314e+02;
  final double cy_d = 2.4273913761751615e+02;

  return (int)(world.x / fx_d / world.z + cx_d);
}

int worldToY(PVector world) {
  final double fx_d = 1.0 / 5.9421434211923247e+02;
  final double fy_d = 1.0 / 5.9104053696870778e+02;
  final double cx_d = 3.3930780975300314e+02;
  final double cy_d = 2.4273913761751615e+02;

  return (int)(world.y / fy_d / world.z + cy_d);
}

