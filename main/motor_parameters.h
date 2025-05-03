#ifndef MOTOR_PARAMETERS_H
#define MOTOR_PARAMETERS_H

// Motor 1 (Base) parameters
#define MOTOR1_KP 0.1f
#define MOTOR1_KI 0.01f
#define MOTOR1_KD 0.05f
#define MOTOR1_MAX_VELOCITY 265  // Maximum velocity for base rotation
#define MOTOR1_CENTER_POSITION 2048

// Motor 2 (Shoulder) parameters
#define MOTOR2_KP 0.1f
#define MOTOR2_KI 0.01f
#define MOTOR2_KD 0.05f
#define MOTOR2_MAX_VELOCITY 265  // Maximum velocity for shoulder
#define MOTOR2_INITIAL_POSITION 1800
#define MOTOR2_MIN_POSITION 1000
#define MOTOR2_MAX_POSITION 3000

// Motor 3 (Elbow) parameters
#define MOTOR3_KP 0.1f
#define MOTOR3_KI 0.01f
#define MOTOR3_KD 0.05f
#define MOTOR3_MAX_VELOCITY 265  // Maximum velocity for elbow
#define MOTOR3_INITIAL_POSITION 1900

// Motor 4 (Wrist) parameters
#define MOTOR4_KP 0.1f
#define MOTOR4_KI 0.01f
#define MOTOR4_KD 0.05f
#define MOTOR4_MAX_VELOCITY 265  // Maximum velocity for wrist
#define MOTOR4_INITIAL_POSITION 3000

#endif // MOTOR_PARAMETERS_H 