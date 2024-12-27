% 二维模糊PID仿真文件
clc; clear;

% Input = [E EC]
Input = [0 3]; 

% fuzzy工具箱的输出
fis = readfis('FuzzyControl_demo01');
[Output1, fuzzifiedIn1, ruleOut1, aggregatedOut1, ruleFiring1] = evalfis(fis, Input);



%%%%%%%%%%%%%%% fuzzy工具箱---底层程序复现 %%%%%%%%%%%%%%%

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

X0 = linspace(-3, 3, 101)';
ruleMembership = zeros(101, 7);
for j = 1: 7
    for i = 1: 101
        ruleMembership(i, j) = trimf(X0(i), mfParams(j, :));
    end
end

membershipMat = zeros(7, 7);
for i = 1: 7
    for j = 1: 7
        membershipMat(i, j) = min(membershipOfE(i), membershipOfEC(j));
    end
end
ruleFiring = reshape(membershipMat', 49, 1);
boolRuleFiring = any(ruleFiring == ruleFiring1);

ruleOut_Kp = zeros(101, 49);
ruleOut_Ki = zeros(101, 49);
ruleOut_Kd = zeros(101, 49);

ruleBase_Kp = reshape(ruleBase_Kp', 49, 1);
ruleBase_Ki = reshape(ruleBase_Ki', 49, 1);
ruleBase_Kd = reshape(ruleBase_Kd', 49, 1);

for j = 1: 49
    for i = 1: 101
        ruleOut_Kp(i, j) = min(ruleFiring(j), ruleMembership(i, ruleBase_Kp(j) + PB + 1));
    end
end
for j = 1: 49
    for i = 1: 101
        ruleOut_Ki(i, j) = min(ruleFiring(j), ruleMembership(i, ruleBase_Ki(j) + PB + 1));
    end
end
for j = 1: 49
    for i = 1: 101
        ruleOut_Kd(i, j) = min(ruleFiring(j), ruleMembership(i, ruleBase_Kd(j) + PB + 1));
    end
end
ruleOut = horzcat(ruleOut_Kp, ruleOut_Ki, ruleOut_Kd);
boolRuleOut = any(any(ruleOut == ruleOut1));

aggregatedOut_Kp = zeros(101, 1);
aggregatedOut_Ki = zeros(101, 1);
aggregatedOut_Kd = zeros(101, 1);
for i = 1: 101
    aggregatedOut_Kp(i) = max(ruleOut_Kp(i, :));
    aggregatedOut_Ki(i) = max(ruleOut_Ki(i, :));
    aggregatedOut_Kd(i) = max(ruleOut_Kd(i, :));
end
aggregatedOut = horzcat(aggregatedOut_Kp, aggregatedOut_Ki, aggregatedOut_Kd);
boolAggregatedOut = any(any(aggregatedOut == aggregatedOut1));

deltaKp = sum(X0 .* aggregatedOut1(:, 1)) ./ sum(aggregatedOut1(:, 1));
deltaKi = sum(X0 .* aggregatedOut1(:, 2)) ./ sum(aggregatedOut1(:, 2));
deltaKd = sum(X0 .* aggregatedOut1(:, 3)) ./ sum(aggregatedOut1(:, 3));

Output = [deltaKp deltaKi deltaKd];
boolOutput = any(Output == Output1);