import processing.serial.*;
Serial myPort;

float yaw = 0.0;
float pitch = 0.0;
float roll = 0.0;

float[][] M = new float[3][3];

class EulerAngle
{
  float X, Y, Z;
  EulerAngle()
  {
    X = 0;
    Y = 0;
    Z = 0;
  }
};

// Euler Order enum.
enum EEulerOrder
{
  ORDER_XYZ, 
    ORDER_YZX, 
    ORDER_ZXY, 
    ORDER_ZYX, 
    ORDER_YXZ, 
    ORDER_XZY;
}


void EulerAnglesToMatrix(EulerAngle inEulerAngle, EEulerOrder EulerOrder)
{
  // Convert Euler Angles passed in a vector of Radians
  // into a rotation matrix.  The individual Euler Angles are
  // processed in the order requested.

  float Sx = sin(inEulerAngle.X);
  float Sy = sin(inEulerAngle.Y);
  float Sz = sin(inEulerAngle.Z);
  float Cx = cos(inEulerAngle.X);
  float Cy = cos(inEulerAngle.Y);
  float Cz = cos(inEulerAngle.Z);

  switch(EulerOrder)
  {
  case ORDER_XYZ:
    M[0][0]=Cy*Cz;
    M[0][1]=-Cy*Sz;
    M[0][2]=Sy;
    M[1][0]=Cz*Sx*Sy+Cx*Sz;
    M[1][1]=Cx*Cz-Sx*Sy*Sz;
    M[1][2]=-Cy*Sx;
    M[2][0]=-Cx*Cz*Sy+Sx*Sz;
    M[2][1]=Cz*Sx+Cx*Sy*Sz;
    M[2][2]=Cx*Cy;
    break;

  case ORDER_YZX:
    M[0][0]=Cy*Cz;
    M[0][1]=Sx*Sy-Cx*Cy*Sz;
    M[0][2]=Cx*Sy+Cy*Sx*Sz;
    M[1][0]=Sz;
    M[1][1]=Cx*Cz;
    M[1][2]=-Cz*Sx;
    M[2][0]=-Cz*Sy;
    M[2][1]=Cy*Sx+Cx*Sy*Sz;
    M[2][2]=Cx*Cy-Sx*Sy*Sz;
    break;

  case ORDER_ZXY:
    M[0][0]=Cy*Cz-Sx*Sy*Sz;
    M[0][1]=-Cx*Sz;
    M[0][2]=Cz*Sy+Cy*Sx*Sz;
    M[1][0]=Cz*Sx*Sy+Cy*Sz;
    M[1][1]=Cx*Cz;
    M[1][2]=-Cy*Cz*Sx+Sy*Sz;
    M[2][0]=-Cx*Sy;
    M[2][1]=Sx;
    M[2][2]=Cx*Cy;
    break;

  case ORDER_ZYX:
    M[0][0]=Cy*Cz;
    M[0][1]=Cz*Sx*Sy-Cx*Sz;
    M[0][2]=Cx*Cz*Sy+Sx*Sz;
    M[1][0]=Cy*Sz;
    M[1][1]=Cx*Cz+Sx*Sy*Sz;
    M[1][2]=-Cz*Sx+Cx*Sy*Sz;
    M[2][0]=-Sy;
    M[2][1]=Cy*Sx;
    M[2][2]=Cx*Cy;
    break;

  case ORDER_YXZ:
    M[0][0]=Cy*Cz+Sx*Sy*Sz;
    M[0][1]=Cz*Sx*Sy-Cy*Sz;
    M[0][2]=Cx*Sy;
    M[1][0]=Cx*Sz;
    M[1][1]=Cx*Cz;
    M[1][2]=-Sx;
    M[2][0]=-Cz*Sy+Cy*Sx*Sz;
    M[2][1]=Cy*Cz*Sx+Sy*Sz;
    M[2][2]=Cx*Cy;
    break;

  case ORDER_XZY:
    M[0][0]=Cy*Cz;
    M[0][1]=-Sz;
    M[0][2]=Cz*Sy;
    M[1][0]=Sx*Sy+Cx*Cy*Sz;
    M[1][1]=Cx*Cz;
    M[1][2]=-Cy*Sx+Cx*Sy*Sz;
    M[2][0]=-Cx*Sy+Cy*Sx*Sz;
    M[2][1]=Cz*Sx;
    M[2][2]=Cx*Cy+Sx*Sy*Sz;
    break;
  }
}

void setup()
{
  size(600, 600, P3D);

  // if you have only ONE serial port active
  myPort = new Serial(this, Serial.list()[0], 115200); // if you have only ONE serial port active

  // if you know the serial port name
  //myPort = new Serial(this, "COM5:", 9600);                    // Windows
  //myPort = new Serial(this, "/dev/ttyACM0", 9600);             // Linux
  //myPort = new Serial(this, "/dev/cu.usbmodem1217321", 9600);  // Mac

  textSize(16); // set text size
  textMode(SHAPE); // set text mode to shape
}

void draw()
{
  serialEvent();  // read and parse incoming serial message
  background(255); // set background to white
  lights();
  
  fill(0);
  stroke(0);
  textSize(20);
  // Quick fix don't care.
  text("roll: " + pitch, 10, 50);
  text("pitch: " + roll, 10, 100);
  text("yaw: " + yaw, 10, 150);
  

  rotateY(-PI/2);
  translate(0, width/2, -height/2); // set position to centre

  pushMatrix(); // begin object

  //float c1 = cos(radians(roll));
  //float s1 = sin(radians(roll));
  //float c2 = cos(radians(pitch));
  //float s2 = sin(radians(pitch));
  //float c3 = cos(radians(yaw));
  //float s3 = sin(radians(yaw));

  //applyMatrix( c2*c3, s1*s3+c1*c3*s2, c3*s1*s2-c1*s3, 0, 
  //  -s2, c1*c2, c2*s1, 0, 
  //  c2*s3, c1*s2*s3-c3*s1, c1*c3+s1*s2*s3, 0, 
  //  0, 0, 0, 1);

  EulerAngle angle = new EulerAngle();
  angle.X = radians(pitch);
  angle.Y = radians(yaw);
  angle.Z = radians(roll);

  EulerAnglesToMatrix(angle, EEulerOrder.ORDER_YXZ);

  applyMatrix(M[0][0], M[0][1], M[0][2], 0, 
    M[1][0], M[1][1], M[1][2], 0, 
    M[2][0], M[2][1], M[2][2], 0, 
    0, 0, 0, 1);

  //applyMatrix(1, 0, 0, 0, 
  //  0, c1, -s1, 0, 
  //  0, s1, c1, 0, 
  //  0, 0, 0, 1);

  //  applyMatrix(c2, s2, 0, 0,
  //  -s2, c2, 0, 0,
  //  0, 0, 1, 0,
  //  0, 0, 0, 1);

  //applyMatrix(c3, 0, s3, 0, 
  //  0, 1, 0, 0, 
  //  -s3, 0, c3, 0,
  //  0, 0, 0, 1);

  //rotateZ(PI/2);
  //rotateY(PI/2);
  //rotateX(PI/2);

  drawArduino();

  popMatrix(); // end of object

  // Print values to console
  print(roll);
  print("\t");
  print(pitch);
  print("\t");
  print(yaw);
  println();
}

void serialEvent()
{
  int newLine = 13; // new line character in ASCII
  String message;
  do {
    message = myPort.readStringUntil(newLine); // read from port until new line
    if (message != null) {
      String[] list = split(trim(message), " ");
      if (list.length >= 4 && list[0].equals("Orientation:")) {
        yaw = float(list[1]); // convert to float yaw
        pitch = float(list[2]); // convert to float pitch
        roll = float(list[3]); // convert to float roll
      }
    }
  } while (message != null);
}

void drawArduino()
{
  /* function contains shape(s) that are rotated with the IMU */
  stroke(0, 90, 90); // set outline colour to darker teal
  fill(0, 130, 130); // set fill colour to lighter teal
  box(50, 50, 50); // draw Arduino board base shape

  stroke(150, 30, 30);
  strokeWeight(3);
  line(0, 0, 0, 0, -100, 0);
}
