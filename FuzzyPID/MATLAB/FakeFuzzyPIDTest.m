% 二维模糊PID仿真文件

% Input = [E EC]
Input = [0 2]; 

% 定义模糊语言对应的量化等级
NB = -3;
NM = -2;
NS = -1;
ZO = 0;
PS = 1;
PM = 2;
PB = 3;

% 定义模糊规则表
ruleBase_Kp = [[PB, PB, PM, PM, PS, ZO, ZO];
               [PB, PB, PM, PS, PS, ZO, NS];
               [PM, PM, PM, PS, ZO, NS, NS];
               [PM, PM, PS, ZO, NS, NM, NM];
               [PS, PS, ZO, NS, NS, NM, NM];
               [PS, ZO, NS, NM, NM, NM, NB];
               [ZO, ZO, NM, NM, NM, NB, NB]];
ruleBase_Ki = [[NB, NB, NM, NM, NS, ZO, ZO];
               [NB, NM, NS, NS, ZO, PS, PS];
               [NM, NM, NS, ZO, PS, PM, PM];
               [NM, NS, ZO, PS, PS, PM, PB];
               [ZO, ZO, PS, PS, PM, PB, PB];
               [ZO, ZO, PS, PM, PM, PB, PB];
               [NB, NB, NM, NM, NS, ZO, ZO]];
ruleBase_Kd = [[PS, NS, NB, NB, NB, NM, PS];
               [ZO, NS, NM, NM, NS, NS, ZO];
               [ZO, NS, NS, NS, NS, NS, ZO];
               [ZO, ZO, ZO, ZO, ZO, ZO, ZO];
               [PB, NS, PS, PS, PS, PS, PB];
               [PB, PM, PM, PM, PS, PS, PB];
               [PS, NS, NB, NB, NB, NM, PS]];

% 隶属度函数参数
mfParams = [[-3, -3, -1];
            [-3, -2,  0];
            [-3, -1,  1];
            [-2,  0,  2];
            [-1,  1,  3];
            [ 0,  2,  3];
            [ 1,  3,  3]];

membershipOfE = [0 0 0 0 0 0 0];
membershipOfEC = [0 0 0 0 0 0 0];
for i = 1: 7
   membershipOfE(i) = trimf(Input(1), mfParams(i, :));
   membershipOfEC(i) = trimf(Input(2), mfParams(i, :));
end

membershipMat = zeros(7, 7);
for i = 1: 7
    for j = 1: 7
        membershipMat(i, j) = min(membershipOfE(i), membershipOfEC(j));
    end
end
membershipSum = sum(sum(membershipMat));

deltaKp = sum(sum(membershipMat .* ruleBase_Kp)) ./ membershipSum;
deltaKi = sum(sum(membershipMat .* ruleBase_Ki)) ./ membershipSum;
deltaKd = sum(sum(membershipMat .* ruleBase_Kd)) ./ membershipSum;

fis = readfis('FuzzyControl_demo01');
output = evalfis(fis, Input);
