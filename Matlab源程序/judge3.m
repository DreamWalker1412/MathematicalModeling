
[distSum,tag] = judgement(routeTemp,Edge,distMax,lastRadius);
if tag==1
    % alpha1 = 25; alpha2 = 15; beta1 = 20; beta2 = 25; theta = 30; delta = 0.001; % 第一组数据
    alpha1 = 20; alpha2 = 10; beta1 = 15; beta2 = 20; theta = 20; delta = 0.001; % 第二组数据
    numOfProblem = 0;
    problem = [];

    for i = 2:length(routeTemp)
        if V(routeTemp(i),5)==1
            numOfProblem = numOfProblem + 1 ;
            problem(numOfProblem) = i;   %#ok<SAGROW>
        end
    end
    if numOfProblem == 0
         successPossbilitySum = 1;
    else
        stack = cell(numOfProblem,4);  %stack为状态栈，存储结构为[errorV,errorH,problePoint,type,]
        
        
        possbility = zeros(3,1);
        % 没有遇到校正失败则成功
        possbility(1) = power(0.8,numOfProblem);

        % 仅考虑任务中遇到一次校正失败的情况
        possbility(2) = numOfProblem * power(0.8,numOfProblem - 1)*0.2;
        success = 0;
        for k = 1:numOfProblem
            
            errorV = zeros(length(routeTemp),1);
            errorH = zeros(length(routeTemp),1);

            errorV(1) = 0;
            errorH(1) = 0;
            tag = 1;  %成功则输出1，失败为-1
            for i = 2:length(routeTemp)
                errorV(i) = errorV(i-1)+Edge(routeTemp(i-1),routeTemp(i))*delta; % 先更新到达路径点i校正前的误差
                stack{k,1}(i) = errorV(i);
                errorH(i) = errorH(i-1)+Edge(routeTemp(i-1),routeTemp(i))*delta;
                stack{k,2}(i) = errorH(i);
                if V(routeTemp(i),4)== 1                                % 判断是否满足各类型点要求，若不满足，判定任务大概率失败
                    if errorV(i)>alpha1 || errorH(i)>alpha2
                        tag = -1;
                    end
                elseif V(routeTemp(i),4)== 0
                    if errorV(i)>beta1 || errorH(i)>beta2
                        tag = -1;
                    end
                elseif V(routeTemp(i),4)== -1
                    if errorV(i)>theta || errorH(i)>theta
                        tag = -1;
                    end
                end

                if V(routeTemp(i),5)==0 ||  problem(k) ~= i           % 该校正点正常校正
                    if V(routeTemp(i),4)== 1  
                        errorV(i)=0;          
                    elseif V(routeTemp(i),4)== 0
                        errorH(i)=0;           
                    end
                elseif V(routeTemp(i),5)==1  && problem(k) == i       % 该校正点为问题校正点且出现问题，只能取得部分校正
                    stack{k,3} = routeTemp(i);
                    if V(routeTemp(i),4)== 1
                        stack{k,4} = 1;
                        errorV(i)= min(errorV(i),5);
                    elseif V(routeTemp(i),4)== 0
                        
                        stack{k,4} = 0;
                        errorH(i)= min(errorH(i),5); 
                    else
                        stack{k,4} = -1;
                    end
                end   
            end

            if tag ==1
                success = success+1;
            end
       
            stack{k,5} = tag;
        end
        % 假设每次任务遇到一次以上校正失败则失败
        possbility(3) = 1 - possbility(1) - possbility(2);

        successPossbilitySum = possbility(1) + possbility(2) * success/numOfProblem;
    end
else
    successPossbilitySum = 0;  % tag ==0时不予判断，一定失败
end