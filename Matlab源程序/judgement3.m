function [distSum,tag,successPossbilitySum,numOfProblem] = judgement3(route,Edge,distMax,lastRadius,V,alpha1,alpha2,beta1,beta2,delta,theta)
% �������������Բ���У��������У���ɹ��ĸ��ʣ����������ӳɹ��ʡ�
% Ϊ�����⣬�����ڵ�һ�ʸ����Ľ⼯��ɸѡ�������ʵ����Ž⼯��
% ���������������У����ʱ����80%У��ʧ�ܣ�ʣ�����Ϊmin(error,5);
% �ɹ����ʼ��㣺
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
        % û������У��ʧ����ɹ�
        possbility(1) = power(0.8,numOfProblem);

        % ����������������һ��У��ʧ�ܵ����
        possbility(2) = numOfProblem * power(0.8,numOfProblem - 1)*0.2;
        success = 0;
        for k = 1:numOfProblem

            errorV = zeros(length(route),1);
            errorH = zeros(length(route),1);

            errorV(1) = 0;
            errorH(1) = 0;
            tag = 1;  %�ɹ������1��ʧ��Ϊ-1
            for i = 2:length(route)
                errorV(i) = errorV(i-1)+Edge(route(i-1),route(i))*delta; % �ȸ��µ���·����iУ��ǰ�����
                errorH(i) = errorH(i-1)+Edge(route(i-1),route(i))*delta;
                if V(route(i),4)== 1                                % �ж��Ƿ���������͵�Ҫ���������㣬�ж���������ʧ��
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

                if V(route(i),5)==0 ||  problem(k) ~= i           % ��У��������У��
                    if V(route(i),4)== 1  
                        errorV(i)=0;          
                    elseif V(route(i),4)== 0
                        errorH(i)=0;           
                    end
                elseif V(route(i),5)==1  && problem(k) == i       % ��У����Ϊ����У�����ҳ������⣬ֻ��ȡ�ò���У��
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
        % ����ÿ����������һ������У��ʧ����ʧ��
        possbility(3) = 1 - possbility(1) - possbility(2);

        successPossbilitySum = possbility(1) + possbility(2) * success/numOfProblem;
    end
else
    successPossbilitySum = 0;  % tag ==0ʱ�����жϣ�һ��ʧ��
end