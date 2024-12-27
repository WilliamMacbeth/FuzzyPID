#include "FuzzyPID.h"

int main(void){
    int ruleBase[][qfDefault] = {
            //ΔKp模糊规则
            {PB, PB, PM, PM, PS, ZO, ZO},
            {PB, PB, PM, PS, PS, ZO, NS},
            {PM, PM, PM, PS, ZO, NS, NS},
            {PM, PM, PS, ZO, NS, NM, NM},
            {PS, PS, ZO, NS, NS, NM, NM},
            {PS, ZO, NS, NM, NM, NM, NB},
            {ZO, ZO, NM, NM, NM, NB, NB},
            //ΔKi模糊规则
            {NB, NB, NM, NM, NS, ZO, ZO},
            {NB, NM, NS, NS, ZO, PS, PS},
            {NM, NM, NS, ZO, PS, PM, PM},
            {NM, NS, ZO, PS, PS, PM, PB},
            {ZO, ZO, PS, PS, PM, PB, PB},
            {ZO, ZO, PS, PM, PM, PB, PB},
            {NB, NB, NM, NM, NS, ZO, ZO},
            //ΔKd模糊规则
            {PS, NS, NB, NB, NB, NM, PS},
            {ZO, NS, NM, NM, NS, NS, ZO},
            {ZO, NS, NS, NS, NS, NS, ZO},
            {ZO, ZO, ZO, ZO, ZO, ZO, ZO},
            {PB, NS, PS, PS, PS, PS, PB},
            {PB, PM, PM, PM, PS, PS, PB},
            {PS, NS, NB, NB, NB, NM, PS}};

    unsigned int inputNum = 2;
    unsigned int outputNum = 3;
    
    // 隶属度函数参数(默认三角形隶属度函数)
    int membershipFunctionType[3] = {1, 1, 1};
    int mfParams[4 * qfDefault] = {-3, -3, -1, 0,
                                   -3, -2,  0, 0,
                                   -3, -1,  1, 0,
                                   -2,  0,  2, 0,
                                   -1,  1,  3, 0,
                                    0,  2,  3, 0,
                                    1,  3,  3, 0};
    

    struct fuzzyPIDControlParameters* fuzzyPID = fuzzyPIDInit(inputNum, outputNum, ruleBase, membershipFunctionType, mfParams);

    float E = 0;
    float EC = 2;

    /***fakeFuzzyControl***/
    fakeFuzzyControl(E, EC, fuzzyPID);

    printf("\nfakeFuzzyControl_Output:\n");
    printf("deltaKp: %.4f\n", fuzzyPID->Output[0]);
    printf("deltaKi: %.4f\n", fuzzyPID->Output[1]);
    printf("deltaKd: %.4f\n", fuzzyPID->Output[2]);

    /***realFuzzyControl***/
    float* ruleMembership = getRuleMembership(fuzzyPID);
    realFuzzyControl(E, EC, fuzzyPID, ruleMembership);
    free(ruleMembership);

    printf("\nrealFuzzyControl_Output:\n");
    printf("deltaKp: %.4f\n", fuzzyPID->Output[0]);
    printf("deltaKi: %.4f\n", fuzzyPID->Output[1]);
    printf("deltaKd: %.4f\n", fuzzyPID->Output[2]);


    deleteFuzzyPIDControlParameters(fuzzyPID);
}