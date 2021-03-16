
errorV = 0; % errorV需小于等于alpha1，当errorV在(beta1,alpha1)之间时，只能先去垂直校正点，图G连通情况改变
errorH = 0; % errorH需小于等于beta2，当errorH在(alpha2,beta2)之间时，只能先去水平校正点，图G连通情况改变
% 除去最后一段航程最远可以飞行theta/delta，其他航段最远飞行距离为MIN(alpha1,beta2,max(beta1,alpha2))/delta
lastRadius = theta/delta;
distVMax= beta2/delta;  % 垂直误差容忍两个垂直校正点间最大航程
distHMax= alpha1/delta; % 水平误差容忍两个水平校正点间最大航程
distMax = distVMax;
% 若下一个目标点为垂直校正点,radiusNextV = min(alpha2,alpha1)/delta;
radiusNextV = min(alpha2,alpha1)/delta;
% 若下一个目标点为水平校正点,radiusNextH = min(beta1,beta2)/delta;
radiusNextH = min(beta1,beta2)/delta;


Dist = zeros(length(V(:,1)));                   % 计算V的邻接矩阵
for i = 1 : length(V(:,1))-1                      % i为出发点的编号
    for j = 2 : length(V(:,1))                  % j为抵达点的编号
        Dist(i,j) = norm((V(i,1:3)-V(j,1:3)),2);
    end
end
distA_B = Dist(1,length(V(:,1)));                   % 当所有校正点都在A至B连线上理想位置时的最短航程，点A到点B的直线距离
minTimes = (distA_B-lastRadius)/distMax * 2 + 1 ;   % 该公式仅对数据一有效！！当所有校正点都在A至B连线上理想位置时的最少校正次数。

Edge = zeros(length(V(:,1)));                   % Edge为有向边
for i = 1 : length(V(:,1))-1                      % i为出发点的编号
    for j = 2 : length(V(:,1))                  % j为抵达点的编号
        if V(j,4)==1 && Dist(i,j)>= radiusNextV % 若下一个目标点为垂直校正点,距离超过可达航程，将该边E(i,j)设为不可达;
            Edge(i,j) = 0;
        elseif V(j,4)==0 && Dist(i,j)>= radiusNextH % 若下一个目标点为水平校正点,距离超过可达航程，将该边E(i,j)设为不可达;
            Edge(i,j) = 0;
        elseif V(j,4)==-1 && Dist(i,j)>= lastRadius % 若下一个目标点为B,距离超过可达航程，将该边E(i,j)设为不可达;
            Edge(i,j) = 0;
        else
            Edge(i,j) = Dist(i,j);
        end
    end
end

% 每次航程能否成立，取决于航程长度是否短于垂直和水平误差可以容忍的长度内
% dist = Dist(i,j);
% (1)若当前目标点为垂直校正点i，下个目标点为水平校正点j
    % dist需要小于等于min（beta1/delta,(beta2-errorH(i))/delta)
    % 目标点j校正后 errorV(j) = dist/delta，errorH(j) = 0;
% (2)若当前目标点为垂直校正点i，下个目标点为垂直校正点j
    % dist需要小于等于min（alpha1/delta,(alpha2-errorH(i))/delta)
    % 目标点j校正后 errorV(j) = 0，errorH(j)=errorH(i)+dist/delta;
% (3)若当前目标点为水平校正点i，下个目标点为水平校正点j
    % dist需要小于等于min（beta1/delta,(beta2-errorH(i))/delta)
    % 目标点j校正后 errorV(j)= errorV(i) + dist/delta，errorH(j) = 0;
% (4)若当前目标点为水平校正点i，下个目标点为垂直校正点j
    % dist需要小于等于min（beta1/delta,(beta2-errorH(i))/delta)
    % 目标点j校正后 errorV(j)= 0，errorH(j) =dist/delta;

% 目标函数:min=   sum(route(i,j)))/distA_B  +  (length(route)-1)/minTimes

% 计算vector(i,j)与vector(i,B)夹角的余弦值，Degree(i,j)值为vector(i,j)与vector(i,B)夹角
Cos = zeros(length(V(:,1)));                    % Cos(i,j)值为vector(i,j)与vector(i,B)夹角的余弦值
Degree = zeros(length(V(:,1)));                 % Degree(i,j)值为vector(i,j)与vector(i,B)夹角
for i = 1 : length(V(:,1))-1                    % i为出发点的编号
    for j = 2 : length(V(:,1))                  % j为抵达点的编号
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

for i = 1 : length(V(:,1))-1                    % i为出发点的编号
    for j = 2 : length(V(:,1))                  % j为抵达点的编号
        if Edge(i,j) ~= 0
          if V(i,4)==V(j,4)                     % 增加约束出发点和目标点不能为同一类型
              Edge(i,j) = 0;
          end
        end
    end
end

for i = 1 : length(V(:,1))-1                    % i为出发点的编号
    for j = 2 : length(V(:,1))                  % j为抵达点的编号
        if Edge(i,j) ~= 0
          if Cos(i,j) <= 0.5               % 要求向量夹角余弦值大于0.5
              Edge(i,j) = 0;
          end
        end
    end
end

Number = find(Edge);            %构建拓扑有向图G
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
numOfEdge = length(find(Edge));  % 图G稀疏化后还剩余的边数


