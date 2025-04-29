#ifndef MOTOR_PARAMETERS_H
#define MOTOR_PARAMETERS_H

// Motor 1 (Base rotation) parameters
#define MOTOR1_KP 0.2f
#define MOTOR1_KI 0.005f
#define MOTOR1_KD 0.05f
#define MOTOR1_MOVING_PWM 150
#define MOTOR1_HOLDING_PWM 30
#define MOTOR1_CENTER_POSITION 2048

// Motor 2 (Shoulder) parameters
#define MOTOR2_KP 0.4f
#define MOTOR2_KI 0.01f
#define MOTOR2_KD 0.15f
#define MOTOR2_MOVING_PWM 400
#define MOTOR2_HOLDING_PWM 500
#define MOTOR2_INITIAL_POSITION 1600
#define MOTOR2_MIN_POSITION 1000
#define MOTOR2_MAX_POSITION 3000

// Motor 3 (Elbow) parameters
#define MOTOR3_KP 0.4f
#define MOTOR3_KI 0.01f
#define MOTOR3_KD 0.1f
#define MOTOR3_MOVING_PWM 400
#define MOTOR3_HOLDING_PWM 300
#define MOTOR3_INITIAL_POSITION 2000

// Motor 4 (Wrist) parameters
#define MOTOR4_KP 0.1f
#define MOTOR4_KI 0.001f
#define MOTOR4_KD 0.3f
#define MOTOR4_MOVING_PWM 200
#define MOTOR4_HOLDING_PWM 100
#define MOTOR4_INITIAL_POSITION 2500

#endif // MOTOR_PARAMETERS_H 