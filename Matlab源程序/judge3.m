
[distSum,tag] = judgement(routeTemp,Edge,distMax,lastRadius);
if tag==1
    % alpha1 = 25; alpha2 = 15; beta1 = 20; beta2 = 25; theta = 30; delta = 0.001; % ��һ������
    alpha1 = 20; alpha2 = 10; beta1 = 15; beta2 = 20; theta = 20; delta = 0.001; % �ڶ�������
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
        stack = cell(numOfProblem,4);  %stackΪ״̬ջ���洢�ṹΪ[errorV,errorH,problePoint,type,]
        
        
        possbility = zeros(3,1);
        % û������У��ʧ����ɹ�
        possbility(1) = power(0.8,numOfProblem);

        % ����������������һ��У��ʧ�ܵ����
        possbility(2) = numOfProblem * power(0.8,numOfProblem - 1)*0.2;
        success = 0;
        for k = 1:numOfProblem
            
            errorV = zeros(length(routeTemp),1);
            errorH = zeros(length(routeTemp),1);

            errorV(1) = 0;
            errorH(1) = 0;
            tag = 1;  %�ɹ������1��ʧ��Ϊ-1
            for i = 2:length(routeTemp)
                errorV(i) = errorV(i-1)+Edge(routeTemp(i-1),routeTemp(i))*delta; % �ȸ��µ���·����iУ��ǰ�����
                stack{k,1}(i) = errorV(i);
                errorH(i) = errorH(i-1)+Edge(routeTemp(i-1),routeTemp(i))*delta;
                stack{k,2}(i) = errorH(i);
                if V(routeTemp(i),4)== 1                                % �ж��Ƿ���������͵�Ҫ���������㣬�ж���������ʧ��
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

                if V(routeTemp(i),5)==0 ||  problem(k) ~= i           % ��У��������У��
                    if V(routeTemp(i),4)== 1  
                        errorV(i)=0;          
                    elseif V(routeTemp(i),4)== 0
                        errorH(i)=0;           
                    end
                elseif V(routeTemp(i),5)==1  && problem(k) == i       % ��У����Ϊ����У�����ҳ������⣬ֻ��ȡ�ò���У��
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
        % ����ÿ����������һ������У��ʧ����ʧ��
        possbility(3) = 1 - possbility(1) - possbility(2);

        successPossbilitySum = possbility(1) + possbility(2) * success/numOfProblem;
    end
else
    successPossbilitySum = 0;  % tag ==0ʱ�����жϣ�һ��ʧ��
end