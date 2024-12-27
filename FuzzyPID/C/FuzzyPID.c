#include"FuzzyPID.h"


struct fuzzyPIDControlParameters* fuzzyPIDInit(const unsigned int inputNum,
                                                const unsigned int outputNum,
                                                int ruleBase[][qfDefault],
                                                unsigned int* membershipFunctionType,
                                                unsigned int* membershipFunctionParameters)
{
    struct fuzzyPIDControlParameters* fuzzyPID = (struct fuzzyPIDControlParameters*)malloc(sizeof(struct fuzzyPIDControlParameters));
    fuzzyPID->inputNum = inputNum;
    fuzzyPID->outputNum = outputNum;

    /*拷贝隶属度函数类型*/
    fuzzyPID->membershipFunctionType = (int *)malloc(3 * sizeof(int));
    for(int i = 0; i < 3; i++){
        fuzzyPID->membershipFunctionType[i] = membershipFunctionType[i];
    }
    fuzzyPID->membershipFunctionParameters = membershipFunctionParameters;

    /*拷贝规则库*/
    fuzzyPID->ruleBase = (int *)malloc(outputNum * qfDefault * qfDefault * sizeof(int));
    for (int i = 0; i < outputNum * qfDefault; i++) {
         for (int j = 0; j < qfDefault; j++) {
             fuzzyPID->ruleBase[i * 7 + j] = ruleBase[i][j];
         }
    }

    fuzzyPID->Output = (float *)malloc(outputNum * sizeof(float));
    return fuzzyPID;
}

void deleteFuzzyPIDControlParameters(struct fuzzyPIDControlParameters *fuzzyPID) {
    free(fuzzyPID->ruleBase);
    free(fuzzyPID->membershipFunctionType);
    free(fuzzyPID->Output);
    free(fuzzyPID);
}

/********定义隶属度函数(membership function)********/
// 梯形
float trapMF(float x, float a, float b, float c, float d) {
    if (x >= a && x < b)
        return (x - a) / (b - a);
    else if (x >= b && x < c)
        return 1.0f;
    else if (x >= c && x <= d)
        return (d - x) / (d - c);
    else return 0.0f;
}
// 三角型
float triMF(float x, float a, float b, float c) {
    if (a == b){
        if (x >= b && x <= c) return (c - x) / (c - b);
        else return 0.0f;
    }

    if (b == c){
        if (x >= a && x <= b) return (x - a) / (b - a);
        else return 0.0f;
    }

    if (x >= a && x < b) return (x - a) / (b - a);
    else if (x >= b && x <= c) return (c - x) / (c - b);
    else return 0.0f;
}
// 选择隶属度函数
float membershipFunction(float x, unsigned int membershipFunctionType, int *membershipFunctionParameters) {
    switch (membershipFunctionType) {
        case 0:
            return trapMF(x, membershipFunctionParameters[0], membershipFunctionParameters[1], membershipFunctionParameters[2], membershipFunctionParameters[4]);
        default: return triMF(x, membershipFunctionParameters[0], membershipFunctionParameters[1], membershipFunctionParameters[2]);
    }
}

/********定义模糊算子********/
// 并算子
float Or(float a, float b, unsigned int type) {
    if (type == 1) {
        return a + b - a * b;
    } else if (type == 2) {
        return fminf(1, a + b);
    } else {
        return fmaxf(a, b);
    }
}
// 交算子
float And(float a, float b, unsigned int type) {
    if (type == 1) {
        return a * b;
    } else if (type == 2) {
        return fmaxf(0, a + b - 1);
    } else {
        return fminf(a, b);
    }
}
// 平衡算子
float Equilibrium(float a, float b, float params) {
    return powf(a * b, 1 - params) * powf(1 - (1 - a) * (1 - b), params);
}
// 模糊算子
float fuzzyOperation(float a, float b, unsigned int type) {
    if (type < 3) {
        return And(a, b, type);
    } else if (type < 6) {
        return Or(a, b, type - 3);
    } else {
        return Equilibrium(a, b, 0.5f);
    }
}

float* getRuleMembership(struct fuzzyPIDControlParameters* fuzzyPID){
    float* ruleMembership = (float *)malloc(101 * qfDefault * sizeof(float));
    float delta = ((float)PB - (float)NB) / 100;

    for (int i = 0; i < qfDefault; i++){
        for (int j = 0; j < 101; j++){
            ruleMembership[i * 101 + j] = membershipFunction(NB + j * delta, fuzzyPID->membershipFunctionType[2], fuzzyPID->membershipFunctionParameters + 4 * i);
        }
    }

    // printf("\nruleMembership:\n");
    // for (int i = 0; i < qfDefault; i++){
    //     printf("\n");
    //     for (int j = 0; j < 101; j++){
    //         printf("%.2f ", ruleMembership[i * 101 + j]);
    //     }
    // }
    return ruleMembership;
}

/********模糊推理和解模糊********/

void realFuzzyControl(float E, float EC, struct fuzzyPIDControlParameters* fuzzyPID, float* ruleMembership){

    // 此处的模糊控制输出与MATLAB的fuzzy工具箱相同，但为了追求模糊理论的准确性，大大提高了程序在内存和运行时间上的开销，更加适用于算力较高的控制器上
    // 模糊算子解析： 交算子：min   模糊蕴含(implication)：min   模糊聚合算子(aggregation)：max
    // 温馨提示1：请至少预留65KB的运行内存用以支持该函数实时运行
    // 温馨提示2：该函数必须与getRuleMembership函数搭配使用，具体方法见main.c
    // 温馨提示3：不要忘记释放ruleMembership的内存空间

    float Membership[qfDefault * fuzzyPID->inputNum]; // 存储隶属度
    float ruleOut[101 * qfDefault * qfDefault * fuzzyPID->outputNum];
    float aggregatedOut[101 * fuzzyPID->outputNum];

    /*求取隶属度*/
    int j = 0;
    for (int i = 0; i < qfDefault; i++){
        Membership[j++] = membershipFunction(E, fuzzyPID->membershipFunctionType[0], fuzzyPID->membershipFunctionParameters + 4 * i);
    }

    for (int i = 0; i < qfDefault; i++){
        Membership[j++] = membershipFunction(EC, fuzzyPID->membershipFunctionType[1], fuzzyPID->membershipFunctionParameters + 4 * i);
    }

    // printf("\nMembership:\n");
    // for (int i = 0; i < j; ++i) {
    //     printf("%f ", Membership[i]);
    // }

    /*求交运算得到模糊矩阵*/
    float membershipMat[qfDefault * qfDefault];
    for (int i = 0; i < qfDefault; ++i) {
        for (int j = 0; j < qfDefault; ++j) {
            membershipMat[i * qfDefault + j] = fuzzyOperation(Membership[i], Membership[qfDefault + j], 0); // And运算
        }
    }

    // printf("\n\nmembershipMat:\n");
    // for (int i = 0; i < qfDefault; ++i) {
    //     printf("\n");
    //     for (int j = 0; j < qfDefault; ++j) {
    //         printf("%f ", membershipMat[i * qfDefault + j]);
    //     }
    // }

    /*跳过计算输出量的模糊集合，直接利用重心法进行解模糊*/
    for (int k = 0; k < fuzzyPID->outputNum; k++){
        for (int i = 0; i < qfDefault * qfDefault; i++){
            for (int j = 0; j < 101; j++){
                ruleOut[k * qfDefault * qfDefault * 101 + i * 101 + j] = fuzzyOperation(membershipMat[i], 
                                                                    ruleMembership[101 * ((int)(fuzzyPID->ruleBase[k * qfDefault * qfDefault + i]) + (int)PB) + j], 0);
            }       
        }
    }

    for (int i = 0; i < fuzzyPID->outputNum; i++){
        for (int j = 0; j < 101; j++){
            float Max = ruleOut[101 * qfDefault * qfDefault * i + j];
            for (int k = 1; k < qfDefault * qfDefault; k++){
                if (ruleOut[101 * qfDefault * qfDefault * i + k * 101 + j] > Max) {
                    Max = ruleOut[101 * qfDefault * qfDefault * i + k * 101 + j];
                }
            }
            aggregatedOut[101 * i + j] = Max;
        }
    }
    
    // printf("\n\naggregatedOut:\n");
    // for (int i = 0; i < fuzzyPID->outputNum; i++) {
    //     printf("\n");
    //     for (int j = 0; j < 101; ++j) {
    //         printf("%f ", aggregatedOut[101 * i + j]);
    //     }
    // }

    float delta = ((float)PB - (float)NB) / 100;
    for (int i = 0; i < fuzzyPID->outputNum; i++){
        float output = 0.0f;
        float membershipSum = 0.0f;
        for (int j = 0; j < 101; j++){
            output += aggregatedOut[101 * i + j] * ((float)NB + j * delta);
            membershipSum += aggregatedOut[101 * i + j];
        }
        fuzzyPID->Output[i] = output / membershipSum;
    }
}

void fakeFuzzyControl(float E, float EC, struct fuzzyPIDControlParameters* fuzzyPID){

    // 此处的模糊控制输出与MATLAB的fuzzy工具箱不同，不考虑输出部分的隶属度分布，但相比于真正的模糊控制大大减少了程序在内存和运行时间上的开销，更加适用于移植到单片机等算力较低的控制器上

    int outputNum = fuzzyPID->outputNum;
    float Membership[qfDefault * outputNum]; // 存储隶属度
    int Belong[qfDefault * outputNum]; // 存储隶属度对应的模糊语言变量类型
    int Count[2] = {0, 0};

    float Temp = 0.0f; // 中间变量

    /*求取隶属度*/
    int j = 0;
    for (int i = 0; i < qfDefault; i++){
        Temp = membershipFunction(E, fuzzyPID->membershipFunctionType[0], fuzzyPID->membershipFunctionParameters + 4 * i);
        if (Temp > 1e-4){
            Membership[j] = Temp;
            Belong[j++] = i; 
        }
    }
    Count[0] = j;

    for (int i = 0; i < qfDefault; i++){
        Temp = membershipFunction(EC, fuzzyPID->membershipFunctionType[1], fuzzyPID->membershipFunctionParameters + 4 * i);
        if (Temp > 1e-4){
            Membership[j] = Temp;
            Belong[j++] = i;
        } 
    }
    Count[1] = j - Count[0];

    // printf("\nMembership:\n");
    // for (int i = 0; i < j; ++i) {
    //     printf("%f ", Membership[i]);
    // }

    /*求交运算得到模糊矩阵*/
    float membershipMat[Count[0] * Count[1]];
    for (int i = 0; i < Count[0]; ++i) {
        for (int j = 0; j < Count[1]; ++j) {
            membershipMat[i * Count[1] + j] = fuzzyOperation(Membership[i], Membership[Count[0] + j], 0);
        }
    }

    // printf("\n\nmembershipMat:\n");
    // for (int i = 0; i < Count[0]; ++i) {
    //     printf("\n");
    //     for (int j = 0; j < Count[1]; ++j) {
    //         printf("%f ", membershipMat[i * Count[1] + j]);
    //     }
    // }

    /*跳过计算输出量的模糊集合，直接利用重心法进行解模糊*/
    float Output[fuzzyPID->outputNum];
    for(int i = 0; i < fuzzyPID->outputNum; i++){
        Output[i] = 0.0f;
    }

    float membershipSum = 0.0f;
    for (int i = 0; i < Count[0]; i++) {
        for (int j = 0; j < Count[1]; j++) {
            membershipSum += membershipMat[i * Count[1] + j];
        }
    }

    // printf("\n\nmembershipSum:%f\n", membershipSum);

    for (int k = 0; k < fuzzyPID->outputNum; k++) {
        for (int i = 0; i < Count[0]; i++) {
            for (int j = 0; j < Count[1]; j++) {
                Output[k] += membershipMat[i * Count[1] + j] * 
                                        fuzzyPID->ruleBase[k * qfDefault * qfDefault + 
                                        Belong[i] * qfDefault + Belong[Count[0] + j]];
            }
        }
    }
    
    for (int i = 0; i < fuzzyPID->outputNum; i++){
        fuzzyPID->Output[i] = Output[i] / membershipSum;// 重心法解模糊
    }
}


