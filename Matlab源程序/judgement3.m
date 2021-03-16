function [distSum,tag,successPossbilitySum,numOfProblem] = judgement3(route,Edge,distMax,lastRadius,V,alpha1,alpha2,beta1,beta2,delta,theta)
% 对于问题三，对部分校正点增加校正成功的概率，尽可能增加成功率。
% 为简化问题，我们在第一问给出的解集中筛选出第三问的最优解集。
% 当飞行器经过标记校正点时，有80%校正失败，剩余误差为min(error,5);
% 成功概率计算：
[distSum,tag] = judgement(route,Edge,distMax,lastRadius);
if tag==1
    numOfProblem = 0;
    problem = [];

    for i = 2:length(route)
        if V(route(i),5)==1
            numOfProblem = numOfProblem + 1 ;
            problem(numOfProblem) = i;  %#ok<AGROW>
        end
    end
    if numOfProblem == 0
         successPossbilitySum = 1;
    else

        possbility = zeros(3,1);
        % 没有遇到校正失败则成功
        possbility(1) = power(0.8,numOfProblem);

        % 仅考虑任务中遇到一次校正失败的情况
        possbility(2) = numOfProblem * power(0.8,numOfProblem - 1)*0.2;
        success = 0;
        for k = 1:numOfProblem

            errorV = zeros(length(route),1);
            errorH = zeros(length(route),1);

            errorV(1) = 0;
            errorH(1) = 0;
            tag = 1;  %成功则输出1，失败为-1
            for i = 2:length(route)
                errorV(i) = errorV(i-1)+Edge(route(i-1),route(i))*delta; % 先更新到达路径点i校正前的误差
                errorH(i) = errorH(i-1)+Edge(route(i-1),route(i))*delta;
                if V(route(i),4)== 1                                % 判断是否满足各类型点要求，若不满足，判定任务大概率失败
                    if errorV(i)>alpha1 || errorH(i)>alpha2
                        tag = -1;
                    end
                elseif V(route(i),4)== 0
                    if errorV(i)>beta1 || errorH(i)>beta2
                        tag = -1;
                    end
                elseif V(route(i),4)== -1
                    if errorV(i)>theta || errorH(i)>theta
                        tag = -1;
                    end
                end

                if V(route(i),5)==0 ||  problem(k) ~= i           % 该校正点正常校正
                    if V(route(i),4)== 1  
                        errorV(i)=0;          
                    elseif V(route(i),4)== 0
                        errorH(i)=0;           
                    end
                elseif V(route(i),5)==1  && problem(k) == i       % 该校正点为问题校正点且出现问题，只能取得部分校正
                    if V(route(i),4)== 1  
                        errorV(i)= min(errorV(i),5);
                    elseif V(route(i),4)== 0
                        errorH(i)= min(errorH(i),5);      
                    end
                end   
            end

            if tag ==1
                success = success+1;
            end
        end
        % 假设每次任务遇到一次以上校正失败则失败
        possbility(3) = 1 - possbility(1) - possbility(2);

        successPossbilitySum = possbility(1) + possbility(2) * success/numOfProblem;
    end
else
    successPossbilitySum = 0;  % tag ==0时不予判断，一定失败
end