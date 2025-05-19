#include <PID_v1_bc.h>
#include <Servo.h>

// Constants
#define SERVO_X_PIN 10     // Servo X control pin
#define SERVO_Y_PIN 9     // Servo Y control pin
#define SERVO_MIN_ANGLE 30  // Minimum angle for the servos
#define SERVO_MAX_ANGLE 150      // Maximum angle for the servos

// Global Variables
float ballPositionX = 0;  // Current position of the ball along the X-axis (in degrees)
float ballPositionY = 0;  // Current position of the ball along the Y-axis (in degrees)
// Dead zone parameters
const float deadZoneRadius = 22.0;//25.0;  // Radius of the dead zone around the setpoint (in degrees)a

// PID Parameters
double Kp = 4.4;//2.6;//3.8;  // Proportional gain
double Ki = .8;//12;//1.3;//5.4; //1.4;  // Integral gain
double Kd = 1.7;//2; //2.16;//1.66;     // Derivative gain

const int SAMPLE_TIME_MS = 40;
// Servo objects
Servo servoX;
Servo servoY;

// PID Variables

double SetpointX = 0;
double SetpointY = 0;
double InputX, OutputX;
double InputY, OutputY;
PID myPID_X(&InputX, &OutputX, &SetpointX, Kp, Ki, Kd, DIRECT);
PID myPID_Y(&InputY, &OutputY, &SetpointY, Kp, Ki, Kd, DIRECT);
const float alpha =0.9 ;//works fine.9;

float f_x = 0;

float f_y = 0;


void arr_saving() 
{
 
}

void setup() {
  servoX.attach(SERVO_X_PIN);
  servoY.attach(SERVO_Y_PIN);

  // Set initial servo positions
  servoX.write(90);  // Center position
  servoY.write(90);  // Center position

  // Initialize Serial communication
  Serial.begin(38400);

  // Initialize PID controllers
  myPID_X.SetSampleTime(SAMPLE_TIME_MS);
  myPID_Y.SetSampleTime(SAMPLE_TIME_MS);
  myPID_X.SetOutputLimits(-120.0, 120.0);//(-115.0, 115.0);
  myPID_Y.SetOutputLimits(-120.0, 120.0);
  myPID_X.SetMode(AUTOMATIC);
  myPID_Y.SetMode(AUTOMATIC);
}

void loop() {

  // unsigned long startTime = millis();
  // adjustParameters_serial();
  if (Serial.available() > 0)
  {
    
    //int x = Serial.parseInt();
    String x1=Serial.readStringUntil(',');
    int x=x1.toInt();
    
    
    f_x = alpha * x + (1 - alpha) * f_x;
    
    String y1=Serial.readStringUntil('\n');
    int y=y1.toInt();
    Serial.println(x1);
    Serial.println(y1);
    
    f_y = alpha * y + (1 - alpha) * f_y;
    if (1)
    { // Check for the end of the line
      //  ballPositionX = map(f_x, -140, 140, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE); //scale was -110 110
      //ballPositionY = map(f_y, -140, 140, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
      ballPositionX = f_x;
      ballPositionY = f_y;


      // Update PID input
      InputX = ballPositionX;
      InputY = ballPositionY;

      // Compute PID output
      myPID_X.Compute();
      myPID_Y.Compute();


    Serial.println(OutputX);
    Serial.println(OutputY);

      // Calculate servo angles
      int servoXAngle = map(OutputX, -120, 120, -70, 70);
      int servoYAngle = map(OutputY, -120, 120, -70, 70);

      // Adjust servo angles based on dead zone
      float distanceFromSetpointX = abs(ballPositionX - SetpointX);
      float distanceFromSetpointY = abs(ballPositionY - SetpointY);

      if (distanceFromSetpointX <= deadZoneRadius)
      {
        // Calculate smooth transition factor
     float smoothFactorX = 1 - (distanceFromSetpointX / deadZoneRadius);
        // Apply smooth transition to servo angle within the deadzone
        servoXAngle = servoXAngle + (smoothFactorX * (SetpointX - servoXAngle));
        // Set Ki to 1 within the dead zone
        myPID_X.SetTunings(Kp+4, Ki+10 , Kd);
      }
      else {
        // Set Ki back to the original value outside the dead zone
        myPID_X.SetTunings(Kp, Ki, Kd);
      }


      if (distanceFromSetpointY <= deadZoneRadius)
      {
        // Calculate smooth transition factor
        float smoothFactorY = 1 - (distanceFromSetpointY / deadZoneRadius);
        // Apply smooth transition to servo angle within the deadzone
       servoYAngle = servoYAngle + (smoothFactorY * (SetpointY - servoYAngle));
        // Set Ki to 1 within the dead zone
        myPID_Y.SetTunings(Kp+4, Ki+10, Kd);
      }
      else {
        // Set Ki back to the original value outside the dead zone
        myPID_Y.SetTunings(Kp, Ki, Kd);
      }
      if (abs(distanceFromSetpointX) <=10&& abs (distanceFromSetpointY) <= 10)
      {
        servoX.write(90);  // Center position
        servoY.write(90);  // Center position

      }
      else{

      servoX.write(-servoXAngle + 90);
      servoY.write(-servoYAngle + 90);
      
      }

    }


  
  }

}

void adjustParameters_serial() {
  if (Serial.available() > 0) {
    char key = Serial.read();
    switch (key) {
      case 'q':
        Kp += 0.2;  // Increase Kp by 0.1
        Serial.println("Kp increased");
        break;
      case 'a':
        Kp -= 0.2;  // Decrease Kp by 0.1
        Serial.println("Kp decreased");
        break;
      case 'w':
        Ki += 0.0001;  // Increase Ki by 0.1
        Serial.println("Ki increased");
        break;
      case 's':
        Ki -= 0.0001;  // Decrease Ki by 0.1
        Serial.println("Ki decreased");
        break;
      case 'e':
        Kd += 0.1;  // Increase Kd by 0.1
        Serial.println("Kd increased");
        break;
      case 'd':
        Kd -= 0.1;  // Decrease Kd by 0.1
        Serial.println("Kd decreased");
        break;
      case 'r':
        // Reset PID parameters to initial values
        Kp = 5;
        Ki = 0;
        Kd = 0.5;
        Serial.println("PID parameters reset");
        break;
      case 'f':
        // Print current values of Kp, Ki, and Kd
        Serial.print("Current PID : Kp = ");
        Serial.print(Kp);
        Serial.print(", Ki = ");
        Serial.print(Ki);
        Serial.print(", Kd = ");
        Serial.println(Kd);
        break;
      // case '+':
      //   // Increase sample time
      //   SAMPLE_TIME_MS += 5; // Increase sample time by 5 ms
      //   Serial.print("Sample time =");
      //   Serial.println(SAMPLE_TIME_MS);
      //   myPID_X.SetSampleTime(SAMPLE_TIME_MS);
      //   myPID_Y.SetSampleTime(SAMPLE_TIME_MS);
      //   break;
      // case '-':
      //   // Decrease sample time
      //   SAMPLE_TIME_MS -= 5; // Decrease sample time by 5 ms
      //   Serial.print("Sample time =");
      //   Serial.println(SAMPLE_TIME_MS);
      //   myPID_X.SetSampleTime(SAMPLE_TIME_MS);
      //   myPID_Y.SetSampleTime(SAMPLE_TIME_MS);
      //   break;
      default:
        break;
    }
  }
}
