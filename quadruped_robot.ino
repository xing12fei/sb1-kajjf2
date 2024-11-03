#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_MPU6050.h>
#include <math.h>

#define MPU6050_ADDRESS 0x68
#define PCA9685_ADDRESS 0x40

#ifndef PI
#define PI 3.14159265358979323846
#endif

Adafruit_MPU6050 mpu;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDRESS);

// Constants for gait planning
const int NUM_LEGS = 4;          
const float STEP_LENGTH = 20.0;  // mm
const float STEP_HEIGHT = 25.0;  // mm
const int CYCLE_LENGTH = 1200;   // ms (optimized for smoother movement)

// 步态方向枚举
enum Direction {
    FORWARD = 1,
    BACKWARD = -1
};

// 更新步态参数结构
struct GaitParams {
    float dutyFactor;
    float phaseOffsets[NUM_LEGS];
    float stepHeight;
    float stepLength;
    Direction direction;
};

// 修改全局步态参数
GaitParams currentGait = {
    0.75,                          // 支撑相占比75%
    {0.75, 0.0, 0.5, 0.25},       // BL, FL, FR, BR 的相位偏移
    STEP_HEIGHT,
    STEP_LENGTH,
    FORWARD                        // 默认前进方向
};

// 修改CPG参数
void updateGaitParameters(float velocity, float roughness, Direction dir) {
    currentGait.stepLength = STEP_LENGTH * velocity;
    currentGait.stepHeight = STEP_HEIGHT * (1.0 + roughness);
    currentGait.dutyFactor = 0.75;
    currentGait.direction = dir;
}

// 优化的轨迹生成函数，支持方向控制
void generateTrajectory(int leg, float phase, float &x, float &y, float &z) {
    if (phase < currentGait.dutyFactor) {
        // 支撑相 - 使用余弦函数使运动更平滑
        float stancePhase = phase / currentGait.dutyFactor;
        x = currentGait.direction * currentGait.stepLength * (0.5 - stancePhase);
        z = 0;
    } else {
        // 摆动相 - 使用正弦函数实现平滑抬腿
        float swingPhase = (phase - currentGait.dutyFactor) / (1 - currentGait.dutyFactor);
        x = currentGait.direction * currentGait.stepLength * (-0.5 + swingPhase);
        z = currentGait.stepHeight * sin(PI * swingPhase);
    }
    y = 0;
}

// 修改walk函数以支持方向控制
void walk(int cycles, float velocity, Direction direction = FORWARD) {
    updateGaitParameters(velocity, 0.0, direction);
    
    for (int cycle = 0; cycle < cycles; cycle++) {
        for (int t = 0; t < CYCLE_LENGTH; t += 15) {  // 优化更新率
            float phase = (float)t / CYCLE_LENGTH;
            
            for (int leg = 0; leg < NUM_LEGS; leg++) {
                float legPhase = fmod(phase + currentGait.phaseOffsets[leg], 1.0);
                float x, y, z;
                
                generateTrajectory(leg, legPhase, x, y, z);
                
                // 计算腿部角度并设置舵机
                double hamAngle, shankAngle;
                calculateLegAngles(leg, x, y, z, hamAngle, shankAngle);
                
                // 映射到PWM值并设置舵机
                int hamPWM = map(hamAngle, 0, 180, 102, 500);
                int shankPWM = map(shankAngle, 0, 180, 102, 500);
                
                pwm.setPWM(leg * 2, 0, hamPWM);
                pwm.setPWM(leg * 2 + 1, 0, shankPWM);
            }
            delay(15);
        }
    }
}

void setup() {
    // ... (setup code remains exactly the same)
}

// 示例loop函数展示前进和后退
void loop() {
    // 前进示例
    float velocity = 0.25;
    walk(2, velocity, FORWARD);
    delay(1000);  // 短暂停顿
    
    // 后退示例
    velocity = 0.2;  // 后退时速度稍慢以保持稳定
    walk(2, velocity, BACKWARD);
    delay(1000);
    
    // 加速前进
    velocity = 0.35;
    walk(3, velocity, FORWARD);
    delay(1000);
}

// ... (其余函数保持不变)