#ifndef __FUZZYPID_H
#define __FUZZYPID_H

/*尽量使用C99及之后版本的C语言标准*/

/*使C++编译器识别C语言内容*/
#ifdef __cplusplus
extern "C"{
#endif

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

/*兼容C11之前的C标准*/
#ifndef bool
#define bool int
#endif

#ifndef false
#define false (int)0
#endif

#ifndef true 
#define true (int)1
#endif

/*定义模糊语言变量的个数*/
enum quantityFields {
    qfSmall = 5,
    qfMiddle = 7,
    qfLarge = 8
};
#define qfDefault (enum quantityFields)qfMiddle // 默认模糊语言个数为7个

/*定义模糊语言对应的量化等级*/
#define NB -3
#define NM -2
#define NS -1
#define ZO 0
#define PS 1
#define PM 2
#define PB 3

/*定义PID控制器相关参数*/
struct PIDController
{
    /*PID控制器参数*/
    float Kp;
    float Ki;
    float Kd;

    float currentError;
    float lastError;
    float lastLastError;  
    
    float Output; //PID控制器输出控制量
    float outputMax;// 输出限幅
};


/*定义模糊PID控制器的相关参数*/
struct fuzzyPIDControlParameters {
    unsigned int inputNum; // 定义模糊控制器输入端子个数
    unsigned int outputNum; // 定义模糊控制器输出端子个数
    unsigned int *membershipFunctionType;
    unsigned int *membershipFunctionParameters;
    int *ruleBase; // 模糊规则库
    float *Output;
};

struct fuzzyPIDControlParameters* fuzzyPIDInit(const unsigned int inputNum,
                                                const unsigned int outputNum,
                                                int ruleBase[][qfDefault],
                                                unsigned int* membershipFunctionType,
                                                unsigned int* membershipFunctionParameters);
void deleteFuzzyPIDControlParameters(struct fuzzyPIDControlParameters *fuzzyPID);
float trapMF(float x, float a, float b, float c, float d);
float triMF(float x, float a, float b, float c);
float membershipFunction(float x, unsigned int membershipFunctionType, int *membershipFunctionParameters);
float Or(float a, float b, unsigned int type);
float And(float a, float b, unsigned int type);
float Equilibrium(float a, float b, float params);
void fakeFuzzyControl(float E, float EC, struct fuzzyPIDControlParameters* fuzzyPID);

float* getRuleMembership(struct fuzzyPIDControlParameters* fuzzyPID);
void realFuzzyControl(float E, float EC, struct fuzzyPIDControlParameters* fuzzyPID, float* ruleMembership);

#ifdef __cplusplus
}
#endif

#endif