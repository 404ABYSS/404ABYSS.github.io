//
// Created by wzh65 on 26-1-17.
//

#include <stdint.h>
#include <stdlib.h>
#include <sys/types.h>
#define  INMAX  1815*0.5//积分最大不超过50%
#define  INMIN   0
//全暗状态下电压最值
#define  OUTMAX  100
#define  OUTMIN 0

# define dt 0.02f // PID采样时间20ms


// #define COEFF_A (8.367510977436154e-7)
// #define COEFF_B (-0.004618271172148525)
// #define COEFF_C (8.555041160434484)
// #define COEFF_D (-5310.459167131552)
//
// #define A (3.767406957487145e-8)
// #define B (-0.00007285472145403954)
// #define C (0.07387763931907898)
// #define D (-19.89481871565293)
//
// #define AAA (14.65f)
// #define BBB (743.5f)
//
// #define T_AAA (0.06759328681008132)
// #define T_BBB  (-49.91406655516466)
//
// /*输入参数：PWM（%）
//  *输出参数：电压（mv）
//  * 手段：三次多项式拟合
//  * 范围 20-50%
//  */
//
// double PWM_to_voltage_20_50(double pwm)
// {
//     return   AAA*pwm + BBB;
// }
//
// /*输入参数：电压（mv）
//  *输出参数：PWM（%）
//  * 手段：三次多项式拟合
//  * 范围 20-50%
//  */
// double voltage_to_PWM_20_50(double voltage_mv)
// {
//     return  T_AAA * voltage_mv +
//               T_BBB;
// }
//
// /*输入参数：电压（mv）
//  *输出参数：PWM（%）
//  * 手段：三次多项式拟合
//  */
// double voltage_to_PWM(double voltage_mv)
// {
//     return    A * voltage_mv * voltage_mv * voltage_mv +
//               B * voltage_mv * voltage_mv +
//               C * voltage_mv +
//               D;
// }
//
//
// /*输入参数：PWM（%）
//  *输出参数：电压（mv）
//  * 手段：三次多项式拟合
//  */
//
// double PWM_to_voltage(double pwm)
// {
//     return    COEFF_A * pwm * pwm * pwm +
//               COEFF_B * pwm * pwm +
//               COEFF_C * pwm +
//               COEFF_D;
// }

/*输入参数：电压（mv）
 *输出参数：PWM（%）
 * 手段：线性插值
 */
double voltage_to_PWM(double voltage_mv)
{
    const uint16_t reference_voltage[] = {380, 775, 1020, 1200, 1345, 1460, 1560, 1645, 1690, 1760, 1815};

    // 边界检查
    if (voltage_mv <= reference_voltage[0]) {
        return 0.0;
    }
    if (voltage_mv >= reference_voltage[10]) {
        return 100.0;
    }

    // 查找区间
    for (int index = 0; index < 10; index++) {
        if (voltage_mv >= reference_voltage[index] &&
            voltage_mv <= reference_voltage[index + 1]) {

            // 防止除零
            if (reference_voltage[index + 1] == reference_voltage[index]) {
                return (double)(index * 10);
            }

            // 线性插值
            double ratio = (voltage_mv - reference_voltage[index]) /
                          (reference_voltage[index + 1] - reference_voltage[index]);
            return (double)(index * 10) + ratio * 10.0;
            }
    }

    // 理论上不会到达这里
    return 50.0;
}

// /*输入参数：目标PWM（%），目前的电压（mv）,负责存储结果的数组的指针
//  *输出参数：目标电压（mv），即将输出的PWM（%）,即将输出的电压（mv）
//  * 手段：PID算法算即将输出的PWM，
//  *      用全暗状态下PWM和电压的对应值算目标电压，之后非10倍的PWM，考虑用线性拟合
//  */
//
// void PID_Light(int16_t Target_PWM, int16_t current_voltage_mv,uint16_t output_array[])
// {
//     const uint16_t reference_target_voltage[] ={380, 775, 1020, 1200, 1345, 1460, 1560, 1645, 1690, 1760, 1815};
//     uint16_t Target_voltage=0,index=0;
//     //目标PWM算目标电压
//     // if (abs(Target_PWM%10-0)<=0.01)
//     //      Target_voltage=reference_target_voltage[Target_PWM/10];
//     // else
//     //      Target_voltage=(uint16_t)PWM_to_voltage_20_50((double)Target_PWM);
//     if (Target_PWM/10>10)
//         index=10;
//     else if (Target_PWM/10<0)
//         index=0;
//     else
//         index=Target_PWM/10;
//
//     Target_voltage=reference_target_voltage[index];
//
//     uint16_t next_PWM=0;
//
//     static float kp_light = 1.5f;
//     static float ki_light = 0.f;
//     static float kd_light = 0.0f;
//
//     static float last_error = 0.0f;
//     static float integral = 0.0f;
//
//     float error = (float)(Target_voltage - current_voltage_mv);
//     integral += error;
//
//     // 积分限幅
//     if (integral > INMAX) integral = INMAX;
//     else if (integral < INMIN) integral = INMIN;
//
//     float derivative = error - last_error;
//     last_error = error;
//
//     float next_voltage_mv = kp_light * error + ki_light * integral + kd_light * derivative;
//
//
//
//
//  next_PWM=voltage_to_PWM(next_voltage_mv+500);
//
//
//      // 输出限幅
//     if (next_PWM > OUTMAX) next_PWM = OUTMAX;
//     else if (next_PWM < OUTMIN) next_PWM = OUTMIN;
//
//     output_array[0]=Target_voltage;
//     output_array[1]=next_PWM;
//     output_array[2]=next_voltage_mv;
// }

/**
 * 纯 PID 控制器（直接输出 PWM 值）
 * @param Target_PWM: 目标 PWM 百分比 (0-100)
 * @param current_voltage_mv: 当前 ADC 电压 (mV)
 * @param output_array: 输出数组 [目标电压, 输出PWM, 当前电压]
 */
void PID_Light(int16_t Target_PWM, int16_t current_voltage_mv, uint16_t output_array[]) {
    // 校准数据：PWM → 电压
    const uint16_t reference_voltage[] = {380, 775, 1020, 1200, 1345, 1460, 1560, 1645, 1690, 1760, 1815};

    // 获取目标电压（用于显示）
    uint16_t target_voltage = 0;
    if (Target_PWM <= 0) {
        target_voltage = reference_voltage[0];
    } else if (Target_PWM >= 100) {
        target_voltage = reference_voltage[10];
    } else {
        // 线性插值得到目标电压
        uint16_t index = Target_PWM / 10;
        uint16_t remainder = Target_PWM % 10;
        if (index >= 10) {
            target_voltage = reference_voltage[10];
        } else {
            uint16_t v_low = reference_voltage[index];
            uint16_t v_high = reference_voltage[index + 1];
            target_voltage = v_low + (v_high - v_low) * remainder / 10;
        }
    }

    // PID 参数
    static float kp =0.05f;     //0.05f;
    static float ki =0.5f;     //0.5f;   // 先关闭积分
    static float kd =0.0015f;     //0.0001f;   // 先关闭微分

    static float last_error = 0.0f;
    static float integral = 0.0f;

    // 计算误差（基于电压）
    float error = (float)(target_voltage - current_voltage_mv);

    // 积分项
    integral += error * dt; // dt = 20ms

    // 积分限幅（基于 PWM 范围）
    if (integral > 50.0f) integral = 50.0f;
    else if (integral < -50.0f) integral = -50.0f;

    // 微分项
    float derivative = (error - last_error) / dt;

    last_error = error;

    // PID 输出（直接是 PWM 值）
    float output_pwm = kp * error + ki * integral + kd * derivative;

    // 转换为实际 PWM 值
    int16_t final_pwm = Target_PWM + (int16_t)output_pwm;

    // PWM 限幅
    if (final_pwm > 100) final_pwm = 100;
    else if (final_pwm < 0) final_pwm = 0;

    // 填充输出数组（用于观察）
    output_array[0] = target_voltage;      // 目标电压 (mV)
    output_array[1] = (uint16_t)final_pwm; // 输出 PWM (%)
    output_array[2] = (uint16_t)current_voltage_mv; // 当前电压 (mV)
}
