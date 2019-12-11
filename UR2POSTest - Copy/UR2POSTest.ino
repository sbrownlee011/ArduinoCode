#include <Servo.h>

// variables used in Arduino Code
const int magPin = 5; // Magnet pin
const int ledPin = 13; // the pin that the LED is attached to
const byte buffSize = 40;
unsigned int inputBuffer[buffSize];
const char startMarker = '<';
const char endMarker = '>';
byte bytesRecvd = 0;
boolean readInProgress = false;
boolean newDataFromPC = false;
byte coordinates[3];

// servo names
Servo baseServo;
Servo armL;
Servo armU;

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  pinMode(magPin, OUTPUT);

  // pins the servos are attached to
  baseServo.attach (11);
  armL.attach (10);
  armU.attach (9);
}

void loop()
{
  // starts the robot in its home position
  armL.write(60);
  delay(50);
  baseServo.write (90);
  armU.write(0);
  // put your main code here, to run repeatedly:
  getDataFromPC(); // receives data from C# on PC
  
  if(newDataFromPC)
  {
    sendSuspendCmd(); // sends a command to stop additional communication until Arduino chip is ready
    //digitalWrite(ledPin, HIGH);
    if(newDataFromPC = true)
    {
      sendCoordinatesToPC(); // This does most of the work turning a position point into the relevent angles and controls the magnet
    }
    // return to home between each shape
    armL.write(60);
    delay(50);
    baseServo.write (90);
    armU.write(0);
    delay(1000);
    //digitalWrite(ledPin, LOW);
    newDataFromPC = false;
    sendEnableCmd(); // allows new data to be sent from C#
  }
}

void sendSuspendCmd()
{
  // send the suspend-true command
  Serial.println("<S1>");
}

void sendEnableCmd()
{
  // send the suspend-false command
  Serial.println("<S0>");
}

void sendCoordinatesToPC()
{
  // variables for the function
  int x, y;
  double a, b, c, h;
  double cph, cps;
  int th, ph, ps, ps2, ps3;

  // simplifies reading the code with shorter variable names
  x = coordinates[0];
  y = coordinates[1];

  // position to angle code for left of center
  if (x < 29)
  {
    a = ((29 - x) * 0.193); // conversion from arbitrary units to inches for calculations

    b = ((43 - y) * 0.198); // conversion from arbitrary units to inches for calculations
    b += 10; // adjusts for distance from the paper

    // code for angle theta for base servo with filters for home position
    th = round(atan2 (a, b) * 180/3.14159265); // converts radians to degrees
    th = round(th * 0.75); // an important multiplier
    th = 90 + th; // adjusts for home position
    
    c = sqrt((a*a)+(b*b)); // hypotenuse of the triangle created by position info to center of base
    
    // the angle the robot arm needs to drop to get from servo base plane to the sheet of paper on the table
    ps2 = round(atan2 (3, c) * 180/PI); // converts radians to degrees
    ps2 = round(ps2 * 0.65); // an important multiplier
    
    h = sqrt((3*3)+(c*c)); // creates the hypotenuse of the trianglle from the robot base to the shape on the table

    // uses reverse law of cosine to get the angle phi for the elbow servo
    cph = ((11.5*11.5)+(10.5*10.5)-(h*h))/(2*11.5*10.5);
    ph = round(acos(cph) * 180/3.14159265); // converts radians to degrees
    ph = ph - 36; // adjusts for the home position
    ph = round(ph * 0.85); // an important multiplier

    // uses reverse law of cosines to get the angle psi for the shoulder servo
    cps = ((10.5*10.5)+(h*h)-(11.5*11.5))/(2*10.5*h);
    ps = round(acos(cps) * 180/3.14159265); // converts radians to degrees
    ps = round(ps * 0.65); // an important multiplier
    ps = ps + 19; // adjusts for home position
  }

  // position to angle code for right of center
  else if (x > 29)
  {
    a = ((x - 29) * 0.193); // conversion from arbitrary units to inches for calculations

    b = ((43 - y) * 0.198); // conversion from arbitrary units to inches for calculations
    b += 10; // adjusts for distance from the paper

    // code for angle theta for base servo with filters for home position
    th = round(atan2 (a, b) * 180/3.14159265); // converts radians to degrees
    th = round(th * 0.75); // an important multiplier
    th = 90 - th; // adjusts for home position
    
    c = sqrt((a*a)+(b*b)); // hypotenuse of the triangle created by position info to center of base

    // the angle the robot arm needs to drop to get from servo base plane to the sheet of paper on the table
    ps2 = round(atan2 (3, c) * 180/PI); // converts radians to degrees
    ps2 = round(ps2 * 0.65); // an important multiplier
    
    h = sqrt((3*3)+(c*c)); // creates the hypotenuse of the trianglle from the robot base to the shape on the table

    // uses reverse law of cosine to get the angle phi for the elbow servo
    cph = ((11.5*11.5)+(10.5*10.5)-(h*h))/(2*11.5*10.5);
    ph = round(acos(cph) * 180/3.14159265); // converts radians to degrees
    ph = ph - 36; // adjusts for home position
    ph = round(ph * 0.85); // an important multiplier

    // uses reverse law of cosines to get the angle psi for the shoulder servo
    cps = ((10.5*10.5)+(h*h)-(11.5*11.5))/(2*10.5*h);
    ps = round(acos(cps) * 180/3.14159265); // converts radians to degrees
    ps = round(ps * 0.65); // an important multiplier
    ps = ps + 19; // adjusts for home position
  }
  
  else if (x == 29)
  {
    a = 0; // no distance from center of x axis of paper, no longer a triangle, but a line

    b = ((43 - y) * 0.198); // conversion from arbitrary units to inches for calculations
    b += 10; // adjusts for distance from the paper
  
    th = 90; // no angle theta for center of x axis of paper
    
    c = sqrt((a*a)+(b*b));// hypotenuse of the triangle created by position info to center of base

    // the angle the robot arm needs to drop to get from servo base plane to the sheet of paper on the table
    ps2 = round(atan2 (3, c) * 180/PI); // converts radians to degrees
    ps2 = round(ps2 * 0.65); // an important multiplier
    
    h = sqrt((3*3)+(c*c)); // creates the hypotenuse of the trianglle from the robot base to the shape on the table

    // uses reverse law of cosine to get the angle phi for the elbow servo
    cph = ((11.5*11.5)+(10.5*10.5)-(h*h))/(2*11.5*10.5);
    ph = round(acos(cph) * 180/3.14159265); // converts radians to degrees
    ph = ph - 36; // adjusts for home position
    ph = round(ph * 0.85); // an important multiplier

    // uses reverse law of cosines to get the angle psi for the shoulder servo
    cps = ((10.5*10.5)+(h*h)-(11.5*11.5))/(2*10.5*h);
    ps = round(acos(cps) * 180/3.14159265); // converts radians to degrees
    ps = round(ps * 0.65); // an important multiplier
    ps = ps + 19; // adjusts for home position
  }

  else // in case of error
  {
    ps = 60;
    delay(50);
    th = 90;
    ph = 0;
  }

  // assigns all the created angles to the arm to get the shape based on sent position
  armU.write(ph);
  delay(300);
  baseServo.write (th);
  armL.write(ps);

  delay(1000);

  ps3 = ps - (ps2 + 6); // creates a special angle to go from angle psi down to leve with the paper

  int m;
  for (m = ps; m >= ps3; m -= 1)
  { // goes from angle psi down to the paper without slamming the table
    armL.write(m);              // tell servo to go to position slower than full speed
    delay(25);                       // waits 25ms for the servo to reach the position 1 degree at a time
  }
  
  //armL.write(ps3);
  digitalWrite(magPin, HIGH); // turns magnet on when it is over the shape

  delay(1000);

  //ps = ps + (ps2 + 3);
  armL.write(ps); // lifts off the table (hopefully) with the shape
  
  delay(1000);

  // back to home position with the shape
  armL.write(60);
  delay(50);
  baseServo.write (90);
  armU.write(0);

  delay(1000);

  // turns left for triangles and right for squares
  if(coordinates[2] == 1)
  {
    baseServo.write (156);
  }

  if(coordinates[2] == 2)
  {
    baseServo.write(23);
  }
  
  delay(1000);

  digitalWrite(magPin, LOW); // turns magnet off
  delay(250);
  armL.write(52); // additional jostle to dislodge a part in case it got stuck on the magnet while off
  
  delay(1000);

  armL.write(60); // back up to home for shoulder servo

  delay(1000);

  baseServo.write (90); // back to home position

  delay(1000);
  
  // send the point data to the PC
  Serial.print("<P");
  Serial.print(ph);
  //Serial.print(coordinates[0]);
  Serial.print(",");
  Serial.print(ps);
  //Serial.print(coordinates[1]);
  Serial.println(">");
}

// alternative to the readBytes function:
void getDataFromPC() 
{
  // receive data from PC and save it into inputBuffer
  if(Serial.available() > 0)
  {
    char x = Serial.read();
    // the order of these IF clauses is significant
      if (x == endMarker) 
      { // receives the serial comunication from C# to the buffer
        readInProgress = false;
        newDataFromPC = true;
        inputBuffer[bytesRecvd] = 0;
        coordinates[0] = inputBuffer[0];
        coordinates[1] = inputBuffer[1];
        coordinates[2] = inputBuffer[2];
      }
    if(readInProgress) 
    { // part of reading and clearing buffer
      inputBuffer[bytesRecvd] = x;
      bytesRecvd ++;
        if (bytesRecvd == buffSize) 
        {
          bytesRecvd = buffSize - 1;
        }
    }
    if (x == startMarker) 
    {
      bytesRecvd = 0;
      readInProgress = true;
    }
  }
}
