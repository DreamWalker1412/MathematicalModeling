function [distSum,tag] = judgement2(route,Edge,distMax,lastRadius,V)
% 第二问增加了对转弯时的假设
% 简单的考虑：在最后判断这条路是否可行时，需要增加因为转弯而增加航程所导致的误差。
% 我们假设在校正后对飞行角度进行调整，所增加的转弯航迹长度为vector(r_{i},r_{i+1})与vector(r_{i+1},r_{i+2})夹角sigma的函数。
% 改变约束条件，在第一问通过粒子群算法求得的解集上筛选出问题二的可行解
% 假设从A点出发时不用调整方向，径直向第一目标点飞去。此后每经过一个校正点，都在校正完成后进行转弯，转弯航迹长度由\sigma*radius来计算。
% 对每段航迹长度函数进行调整distNew(i) = dist(i)+2*radius*(\sigma(i)-sin(\sigma(i)))，其中i为途径的第i个校正点对应的向量角
% 

% 第二问的判断及距离计算
tag = 1;  % 成功则输出1，失败为-1
sigma = zeros(length(route)-1,1);
dist = zeros(length(route)-1,1);
sigma(1)=0;   % 对于点A所属的出发航段而言，航迹长度没有发生变化。
dist(1)= Edge(route(1),route(2));
for i = 2:length(route)-1
    v1 = V(route(i),1:3)-V(route(i-1),1:3);
    v2 = V(route(i+1),1:3)-V(route(i),1:3);
    cos = dot(v1,v2)/norm(v1,2)/norm(v2,2);
    sigma(i) = real(rad2deg(acos(cos)))/180.0*pi;
    dist(i) = Edge(route(i),route(i+1))+ bonusLength(sigma(i));
end

for i = 1:length(route)-2
 if dist(i)+dist(i+1) > distMax
     tag = -1;
 end
end
if dist(end-1)+dist(end) > lastRadius
     tag = -1;
end

distSum = 0;
for i = 1:length(route)-1
    distSum = distSum + dist(i);
end
end