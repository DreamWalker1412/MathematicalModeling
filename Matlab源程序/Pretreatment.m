
errorV = 0; % errorV��С�ڵ���alpha1����errorV��(beta1,alpha1)֮��ʱ��ֻ����ȥ��ֱУ���㣬ͼG��ͨ����ı�
errorH = 0; % errorH��С�ڵ���beta2����errorH��(alpha2,beta2)֮��ʱ��ֻ����ȥˮƽУ���㣬ͼG��ͨ����ı�
% ��ȥ���һ�κ�����Զ���Է���theta/delta������������Զ���о���ΪMIN(alpha1,beta2,max(beta1,alpha2))/delta
lastRadius = theta/delta;
distVMax= beta2/delta;  % ��ֱ�������������ֱУ�������󺽳�
distHMax= alpha1/delta; % ˮƽ�����������ˮƽУ�������󺽳�
distMax = distVMax;
% ����һ��Ŀ���Ϊ��ֱУ����,radiusNextV = min(alpha2,alpha1)/delta;
radiusNextV = min(alpha2,alpha1)/delta;
% ����һ��Ŀ���ΪˮƽУ����,radiusNextH = min(beta1,beta2)/delta;
radiusNextH = min(beta1,beta2)/delta;


Dist = zeros(length(V(:,1)));                   % ����V���ڽӾ���
for i = 1 : length(V(:,1))-1                      % iΪ������ı��
    for j = 2 : length(V(:,1))                  % jΪ�ִ��ı��
        Dist(i,j) = norm((V(i,1:3)-V(j,1:3)),2);
    end
end
distA_B = Dist(1,length(V(:,1)));                   % ������У���㶼��A��B����������λ��ʱ����̺��̣���A����B��ֱ�߾���
minTimes = (distA_B-lastRadius)/distMax * 2 + 1 ;   % �ù�ʽ��������һ��Ч����������У���㶼��A��B����������λ��ʱ������У��������

Edge = zeros(length(V(:,1)));                   % EdgeΪ�����
for i = 1 : length(V(:,1))-1                      % iΪ������ı��
    for j = 2 : length(V(:,1))                  % jΪ�ִ��ı��
        if V(j,4)==1 && Dist(i,j)>= radiusNextV % ����һ��Ŀ���Ϊ��ֱУ����,���볬���ɴﺽ�̣����ñ�E(i,j)��Ϊ���ɴ�;
            Edge(i,j) = 0;
        elseif V(j,4)==0 && Dist(i,j)>= radiusNextH % ����һ��Ŀ���ΪˮƽУ����,���볬���ɴﺽ�̣����ñ�E(i,j)��Ϊ���ɴ�;
            Edge(i,j) = 0;
        elseif V(j,4)==-1 && Dist(i,j)>= lastRadius % ����һ��Ŀ���ΪB,���볬���ɴﺽ�̣����ñ�E(i,j)��Ϊ���ɴ�;
            Edge(i,j) = 0;
        else
            Edge(i,j) = Dist(i,j);
        end
    end
end

% ÿ�κ����ܷ������ȡ���ں��̳����Ƿ���ڴ�ֱ��ˮƽ���������̵ĳ�����
% dist = Dist(i,j);
% (1)����ǰĿ���Ϊ��ֱУ����i���¸�Ŀ���ΪˮƽУ����j
    % dist��ҪС�ڵ���min��beta1/delta,(beta2-errorH(i))/delta)
    % Ŀ���jУ���� errorV(j) = dist/delta��errorH(j) = 0;
% (2)����ǰĿ���Ϊ��ֱУ����i���¸�Ŀ���Ϊ��ֱУ����j
    % dist��ҪС�ڵ���min��alpha1/delta,(alpha2-errorH(i))/delta)
    % Ŀ���jУ���� errorV(j) = 0��errorH(j)=errorH(i)+dist/delta;
% (3)����ǰĿ���ΪˮƽУ����i���¸�Ŀ���ΪˮƽУ����j
    % dist��ҪС�ڵ���min��beta1/delta,(beta2-errorH(i))/delta)
    % Ŀ���jУ���� errorV(j)= errorV(i) + dist/delta��errorH(j) = 0;
% (4)����ǰĿ���ΪˮƽУ����i���¸�Ŀ���Ϊ��ֱУ����j
    % dist��ҪС�ڵ���min��beta1/delta,(beta2-errorH(i))/delta)
    % Ŀ���jУ���� errorV(j)= 0��errorH(j) =dist/delta;

% Ŀ�꺯��:min=   sum(route(i,j)))/distA_B  +  (length(route)-1)/minTimes

% ����vector(i,j)��vector(i,B)�нǵ�����ֵ��Degree(i,j)ֵΪvector(i,j)��vector(i,B)�н�
Cos = zeros(length(V(:,1)));                    % Cos(i,j)ֵΪvector(i,j)��vector(i,B)�нǵ�����ֵ
Degree = zeros(length(V(:,1)));                 % Degree(i,j)ֵΪvector(i,j)��vector(i,B)�н�
for i = 1 : length(V(:,1))-1                    % iΪ������ı��
    for j = 2 : length(V(:,1))                  % jΪ�ִ��ı��
        if Edge(i,j) ~= 0
            v1 = V(j,1:3)-V(i,1:3);
            v2 = V(length(V(:,1)),1:3)-V(i,1:3);
            if j == length(V(:,1))
                Cos(i,j)=1;
            else
                Cos(i,j) = dot(v1,v2)/norm(v1,2)/norm(v2,2);
            end
            Degree(i,j) = real(rad2deg(acos(Cos(i,j))));
        end
    end
end

for i = 1 : length(V(:,1))-1                    % iΪ������ı��
    for j = 2 : length(V(:,1))                  % jΪ�ִ��ı��
        if Edge(i,j) ~= 0
          if V(i,4)==V(j,4)                     % ����Լ���������Ŀ��㲻��Ϊͬһ����
              Edge(i,j) = 0;
          end
        end
    end
end

for i = 1 : length(V(:,1))-1                    % iΪ������ı��
    for j = 2 : length(V(:,1))                  % jΪ�ִ��ı��
        if Edge(i,j) ~= 0
          if Cos(i,j) <= 0.5               % Ҫ�������н�����ֵ����0.5
              Edge(i,j) = 0;
          end
        end
    end
end

Number = find(Edge);            %������������ͼG
count = length(Number);
w = zeros(count,1);
s = mod(Number,length(V(:,1)));
t = (Number-s)/length(V(:,1))+1;
for i = 1:count
    w(i) = Edge(Number(i));
end
G = graph(s,t,w);

shortestDistToB = zeros(length(V(:,1)),1); 
for i = 1:length(V(:,1))
    [~,shortestDistToB(i)] =  shortestpath(G,i,length(V(:,1)));
end
numOfEdge = length(find(Edge));  % ͼGϡ�軯��ʣ��ı���


