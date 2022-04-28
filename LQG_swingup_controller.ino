// includes
#include <HardwareSerial.h>
#include <ODriveArduino.h>
#include "linalg_K16.h"
// Printing with stream operator helper functions
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

HardwareSerial& odrive_serial = Serial1;

// ODrive object
ODriveArduino odrive(odrive_serial);

#define limitSwitch      3
#define estop       21

#define radius 0.0189 // meters
#define track_length 0.80 // meters
#define safetyMargin 0.05 // meters
#define startMargin 0.2 // rad

#define PI 3.1415926535897932384626433832795

#define M 0.308
#define m 0.123
#define L 0.186
#define g 9.81
#define b 0.3

volatile bool halt = false;
volatile bool emergency = false;
volatile bool startup = true;
volatile bool outOfBounds = false; 

void rightLimiter(){
    //Serial.println("right limit hit!");
    odrive.SetVelocity(0, 0.5);
    halt = true;
}

void emergencyStop() {
  Serial.println("EMERGENCY");
  halt = true;
  emergency = true;
  odrive.SetVelocity(0, 1);
  
  Serial << "Axis" << 0 << ": Requesting state " << AXIS_STATE_IDLE << '\n';
  if(!odrive.run_state(0, AXIS_STATE_IDLE, false)) return;
}

void setup() {

  pinMode(limitSwitch, INPUT_PULLUP);
  pinMode(estop, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(limitSwitch), rightLimiter, CHANGE);
  attachInterrupt(digitalPinToInterrupt(estop), emergencyStop, FALLING);
  
  // ODrive uses 115200 baud
  odrive_serial.begin(115200);

  // Serial to PC
  Serial.begin(115200);
  while (!Serial) ; // wait for Arduino Serial Monitor to open

  Serial.println("ODriveArduino");
  Serial.println("Setting parameters...");

  for (int axis = 0; axis < 2; ++axis) {
    odrive_serial << "w axis" << axis << ".controller.config.vel_limit " << 10.0f << '\n';
    odrive_serial << "w axis" << axis << ".motor.config.current_lim " << 11.0f << '\n';
  }

  int requested_state;

  requested_state = AXIS_STATE_MOTOR_CALIBRATION;
  Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
  if(!odrive.run_state(0, requested_state, true)) return;

  requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
  Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
  if(!odrive.run_state(0, requested_state, true, 25.0f)) return;

  requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
  Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
  if(!odrive.run_state(0, requested_state, false)) return;

  odrive.SetVelocity(0, 0);
  delay(500);

  Serial.println("Ready! Go Startup!");
  odrive.SetVelocity(0, -1);
}

float startVel = 0.5;
float pendDown;
float t0 = -1.0;
float x0;
float v0;
float absth0;
float startPoint = 0.0;
bool killSwing = false;
void loop() {
  // debounce limit switch
  if(halt && !emergency) {
    if(startup) {
      odrive.SetVelocity(0, 0);

      y.elements[0][0] = (track_length/2) + .07; // initial position = 0.47 m
      x0 = y.elements[0][0];
      startPoint = y.elements[0][0];
      float dt = time_to_move(y.elements[0][0], startVel);
  
      Serial << "Track length: " << (track_length * 100) << " cm" << '\n';
      Serial << "Running for: " << dt << " milliseconds" << '\n';

      // Centering Cart on Rail
      odrive.SetVelocity(0, startVel);
      delay(dt);
      odrive.SetVelocity(0, 0);

      v0 = 0;

      Serial.println("Waiting for pendulum to settle...");
      delay(5000);

      // Get angle of down position
      Serial << "Axis" << 1 << ": Requesting state " << AXIS_STATE_ENCODER_OFFSET_CALIBRATION << '\n';
      if(!odrive.run_state(1, AXIS_STATE_ENCODER_OFFSET_CALIBRATION, true, 25.0f)) return;

      pendDown = odrive.GetPosition(1) + 500;
      y.elements[2][0] = theta();
      Serial << "Pendulum Position: " << y.elements[2][0] << '\n';
      Serial << "Cart Position: " << y.elements[0][0] << '\n';

      startup = false;
      
      // Send initial conditions:
      //   x, theta, reference x, reference theta
      // Sets up x, abk = (A-B*K), err = x - ref
      setStateSpace(y.elements[0][0], y.elements[2][0], 0.47, PI);
      
      Serial.println("ALL GO FOR NORMAL OPERATION");

      Serial.println("Givin 'er a wee kick!");
      float turns = uToTurns(20, 0, 0.5);
      //odrive.SetVelocity(0, turns);
      delay(155);
      //odrive.SetVelocity(0, 0);
    }
    
    halt = false;
  }

  // MODE: Normal Operation
  if(!halt && !startup) {
      if(t0 == -1.0) t0 = micros();
      float t1 = micros();
      float dt = (t1 - t0) / 1000000.0;
      float v1 = 0;

      // LIVE sensors vector
      y.elements[0][0] = x_pos(dt);
      y.elements[1][0] = 0;
      y.elements[2][0] = theta();
      y.elements[3][0] = 0;
      absth0 = odrive.GetPosition(1);

      // Full-state Estimation
      float curPos, posVel, curTheta, angVel;
      estimateState(dt);
      curPos =    senserr.elements[0][0] + y.elements[0][0]; // x
      posVel =    senserr.elements[1][0] + y.elements[1][0]; // xdot
      curTheta =  senserr.elements[2][0] + y.elements[2][0]; // th
      angVel =    senserr.elements[3][0] + y.elements[3][0]; // thdot 

      // IMPORTANT: Sometimes the encoder slips on the shaft
      //  Check to see that the pendulum is not slipping in the encoder.
      /*Serial.println("====");
      Serial.println(curPos);
      Serial.println(posVel);
      Serial.println(curTheta);
      Serial.println(angVel);*/
      //matshow(&xhat);
      //delay(3000);

      if(curPos > 0.0 + safetyMargin && curPos < track_length + 0.14)
      {

        if(curTheta > PI - 0.3 && curTheta < PI + 0.3) {
          //Serial.println("LQR!");
          Serial.println(angVel);
          killSwing = true;
          
          lqr_controller(curPos, posVel, curTheta, angVel, dt);
        }

        if(!killSwing &&
        (curTheta < PI - 0.6 || curTheta > PI + 0.6)) {
          //Serial.println("Go Swing!");
          
          // Cannot use Kalman filter for this because it is designed around
          //  the linearized system dynamics (centered around the up position).
          //  For this, we would need a nonlinear Kalman filter.
          //  So, swing up will remain noisy for the moment.
          //swingup_controller(x_pos(dt), x_dot(dt, x0, curPos), absth0, theta_dot(dt, v0, v1), dt);
          //swingup_controller(curPos, posVel, absth0, angVel, dt);
        }
        
      } else {
        if(!outOfBounds) {
          Serial << "Cart out of Bounds! @ "    << y.elements[0][0] << " m" << '\n';
        }
        odrive.SetVelocity(0, 0);
        outOfBounds = true;
      }

      t0 = t1;
      x0 = y.elements[0][0];
  }
}

// Kalman Filter
// Estimates full-state of system
// Given input: u, current state: xhat_old, sensors: y, time: dt
// Updates current state: xhat_new (xhat)
void estimateState(float dt) {
  
  // xhat = [ (A-Kf*C)*xhat_old + B*u + Kf*y ] * dt

  // (A - Kf*C) from Octave
  struct matrix akfcx = matmul(&akfc, &xhat);
  struct matrix bu = matmul(&bmat, &u); // using previous actuator input
  struct matrix kf_full = matmul(&kf, &cmat);
  struct matrix kfy = matmul(&kf_full, &y); // using formatted sensor data
  
  struct matrix error = matadd(&akfcx, &kfy);
  error = matadd(&error, &bu);
  error = matscale(dt, &error);

  if(millis() % 1 == 0) {
    /*Serial.println("akfcx ----- ");
    matshow(&akfcx);
    Serial.println("kfy ----- ");
    matshow(&kfy);
    Serial.println("bu ----- ");
    matshow(&bu);*/
    //Serial.println("error ----- ");
    //matshow(&error);
    /*Serial.println("y ----- ");
    matshow(&y);
    Serial.println("actual ----- ");
    matshow(&matadd(&error, &y));*/
  }

  // xhat_new (xhat)
  // (full-state estimate)
  xhat = matadd(&error, &xhat);

  // just gonna use this cause it's not doing what it should be.
  senserr = error;
}

// Requires state vector x (x, x_dot, th, th_dot, dt) in meters and seconds
void lqr_controller(float curPos, float posVel, float curTheta, float angVel, float dt) {

  // xhat - ref
  // xhat is full-state estimate
  struct matrix err = matsub(&xhat, &ref);

  // (A - B*K) from MATLAB
  // --------------------
  // (A - B*K) * (xhat - ref)
  struct matrix x_old = matmul(&abk, &err);
  x_old = matscale(dt, &x_old);
  x = matadd(&xhat, &x_old);
  struct matrix kx = matmul(&k, &x_old);
  struct matrix u = matscale(-1, &kx);
  double force = u.elements[0][0];

  //y = matmul(&c, &x_new);

  // IMPORTANT: If the system responds weakly, BUMP UP THE GAIN
  float turns = uToTurns(force, posVel, dt);
  
  //Serial << "turns: " << turns << '\n';
  
  odrive.SetVelocity(0, turns);

  // For testing slow it down
  //delay(3000);
}

// Requires state vector x (x, x_dot, th, th_dot, dt) in meters and seconds
void swingup_controller(float curPos, float posVel, float curTheta, float angVel, float dt) {

  float ke = 6600; // energy gain
  float kp = 0; // cart position gain
  float kd = 0; // cart velocity gain

  float th = theta();
  float delta_th = odrive.GetPosition(1) - curTheta;
  float thetadot = delta_th / dt;

  // Lyapunov Energy Function
  float energy = (1/2)*m*(L*L)*(thetadot*thetadot) - m*g*L*cos(th) - m*g*L;
  
  float xddot = ke*m*L*thetadot*cos(th)*energy - kp*(curPos-startPoint) - kd*posVel;
  float xdot = xddot*dt + posVel;
  float newPos = (xdot-posVel)*dt + curPos;

  // Convert to force of cart to exploit existing machinery
  double force = xddot * (m+M);

  // Kalman filter is always estimating for linearized region.
  //  When in area of operation for LQR, xhat will snap into place.
  //x.elements[0][0] = newPos;
  //x.elements[1][0] = xdot;
  //x.elements[2][0] = th;
  //x.elements[3][0] = thetadot;

  float turns = uToTurns(force, xdot, dt);

  odrive.SetVelocity(0, turns);
}

// Requires x distance in meters, velocity in m/s
// Returns time to move to location in milliseconds
float time_to_move(float distance, float v) {
  return distance / (abs(v) * 2*PI*radius) * 1000; // milliseconds
}

// Requires delta t in seconds
// Returns x position in meters after some delta t
float x_pos(float dt) {
  // Need to invert velocity because clockwise rotation is negative v
  //  this means that movement to the right is negative, when we have
  //  defined it to be the positive x direction.
  float displacement = (-odrive.GetVelocity(0) * 2*PI*radius) * dt;
  return y.elements[0][0] + displacement;
}

// Requires delta t in seconds, x0 in meters, x1 in meters
// Returns velocity of cart in m/s
float x_dot(float dt, float x_pos0, float x_pos1) {
  return (x_pos1 - x_pos0) / dt;
}

// Returns theta in radians
float theta() {
  float cur = odrive.GetPosition(1);
  float base = abs(pendDown - cur);
  int count = base / 2.5;
  base -= (count * 2.5);
  base /= 2.5;
  
  return base * 2*PI;
}

// Requires delta t in seconds, theta0 in rad, theta1 in rad
// Returns angular velocity in rad/s
float theta_dot(float dt, float theta0, float theta1) {
  return (theta1 - theta0) / dt;
}

// Requires force u in newtons, v0 in m/s, dt in seconds
// Returns turns per second (odrive velocity) in turns
float uToTurns(float u, float v_init, float dt) {
  float v_final = ((u / (M + m)) * dt) + v_init; // desired velocity in meters per second
  return v_final / (2*PI*radius); // turns per second to achieve required linear force
}
