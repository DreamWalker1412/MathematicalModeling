clear;
% Ĭ�Ͻ������1��ѡ����1,2,3
questionNum = 1;
% Ĭ��ʹ��sheet 1 ���ݡ�ѡ����1��2;
dataSheet = 1;  
%  ��10000�����к������ҳ�����ֵ
iterNum = 10000;

load('��ѧ��ģ.mat');
if dataSheet ==1 
   alpha1 = 25; alpha2 = 15; beta1 = 20; beta2 = 25; theta = 30; delta = 0.001;  % ��һ�����ݲ���
   V=[rawdata1.x(:),rawdata1.y(:),rawdata1.z(:),rawdata1.type(:),rawdata1.bool(:)];  % ���ݱ�1�㼯V
else
   alpha1 = 20; alpha2 = 10; beta1 = 15; beta2 = 20; theta = 20; delta = 0.001; % �ڶ�������
   V=[rawdata2.x(:),rawdata2.y(:),rawdata2.z(:),rawdata2.type(:),rawdata2.bool(:)];  % ���ݱ�2�㼯V
end

Pretreatment;   % ͼԤ�����ļ�

if questionNum == 1
    question;   % ����һ����ļ�
elseif questionNum == 2
    question2;  % ���������ļ�
else           
    question3;  % ����������ļ�
end

% Result����һ��patato�⼯
Result{1} = [routeInfo(1,2:3),route{routeInfo(1),1}];  % (���ɹ�����)��̺������Ƚ�
Result{1}(3:end)
if Result{1}(1)~= shortestRouteLength
    for i = 1:iterNum
        if routeInfo(i,2) == shortestRouteLength
            Result{2} = [routeInfo(i,2:3),route{routeInfo(i),1}];   % (���ɹ�����)���ٽڵ����Ƚ�
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