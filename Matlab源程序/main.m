clear;
% 默认解答问题1。选项有1,2,3
questionNum = 1;
% 默认使用sheet 1 数据。选项有1，2;
dataSheet = 1;  
%  在10000条可行航迹中找出最优值
iterNum = 10000;

load('数学建模.mat');
if dataSheet ==1 
   alpha1 = 25; alpha2 = 15; beta1 = 20; beta2 = 25; theta = 30; delta = 0.001;  % 第一组数据参数
   V=[rawdata1.x(:),rawdata1.y(:),rawdata1.z(:),rawdata1.type(:),rawdata1.bool(:)];  % 数据表1点集V
else
   alpha1 = 20; alpha2 = 10; beta1 = 15; beta2 = 20; theta = 20; delta = 0.001; % 第二组数据
   V=[rawdata2.x(:),rawdata2.y(:),rawdata2.z(:),rawdata2.type(:),rawdata2.bool(:)];  % 数据表2点集V
end

Pretreatment;   % 图预处理文件

if questionNum == 1
    question;   % 问题一求解文件
elseif questionNum == 2
    question2;  % 问题二求解文件
else           
    question3;  % 问题三求解文件
end

% Result给出一个patato解集
Result{1} = [routeInfo(1,2:3),route{routeInfo(1),1}];  % (最大成功概率)最短航迹优先解
Result{1}(3:end)
if Result{1}(1)~= shortestRouteLength
    for i = 1:iterNum
        if routeInfo(i,2) == shortestRouteLength
            Result{2} = [routeInfo(i,2:3),route{routeInfo(i),1}];   % (最大成功概率)最少节点优先解
            Result{2} (3:end)
            break;
        end
    end
end

%#ok<*NOPTS>
%#ok<*NASGU>
clear stackA stackB stackC stackD temp errorH errorV fesibleB fesibleP iterNum i p x ;
clear i j ans v1 v2 h1 h2 Number count s t w k temp VDist HDist;
clear alpha1 alpha2 beta1 beta2 theta Cos Degree delta Dist distA_B distHMax distVMax Edge lastRadius minTimes;
clear radiusNextV radiusNextH randTemp routeDist routeLength routeNum V rawdata1 rawdata2 dataSheet questionNum distMax;
clear shortestDistToB successPossbility;