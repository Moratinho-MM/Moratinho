#pragma once 

#define RIGHT_ENGINE_FRONT_PIN  27
#define RIGHT_ENGINE_BACK_PIN   16
#define RIGHT_ENGINE_PWM_PIN    25
#define RIGHT_ENGINE_PWM_CHANNEL 1

#define LEFT_ENGINE_FRONT_PIN    4
#define LEFT_ENGINE_BACK_PIN    32
#define LEFT_ENGINE_PWM_PIN     26
#define LEFT_ENGINE_PWM_CHANNEL  0

#define LEFT_ENCODER_A_PIN      35
#define LEFT_ENCODER_B_PIN      39

#define RIGHT_ENCODER_A_PIN     34
#define RIGHT_ENCODER_B_PIN     33

// Aqui sãa definidos os pinos de IO de cada sensor IR
#define XSHUT_F 19
#define XSHUT_R 18
#define XSHUT_L 23

#define WHEEL_DIAM 3.27

#define MAZE_SIZE  16
#define MAZE_GOAL_X 8
#define MAZE_GOAL_Y 8
#define TICKS_PER_CELL   203   // mudar para ticks

#define WALL_DETECTION_TRESHOLD 9.3 // Ajustar esse parâmetro com testes no labirinto
#define BLUETOOTH_NAME "Moratinho_BLE"
#define BLUETOOTH_NOTIFY_TIMER 1000

#define SEND_TELEMETRY

#define BLUETOOTH_ENABLE
//#define SIMULATION

#define TUNE_WALL_DISTANCE
