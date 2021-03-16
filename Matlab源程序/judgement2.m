function [distSum,tag] = judgement2(route,Edge,distMax,lastRadius,V)
% �ڶ��������˶�ת��ʱ�ļ���
% �򵥵Ŀ��ǣ�������ж�����·�Ƿ����ʱ����Ҫ������Ϊת������Ӻ��������µ���
% ���Ǽ�����У����Է��нǶȽ��е����������ӵ�ת�亽������Ϊvector(r_{i},r_{i+1})��vector(r_{i+1},r_{i+2})�н�sigma�ĺ�����
% �ı�Լ���������ڵ�һ��ͨ������Ⱥ�㷨��õĽ⼯��ɸѡ��������Ŀ��н�
% �����A�����ʱ���õ������򣬾�ֱ���һĿ����ȥ���˺�ÿ����һ��У���㣬����У����ɺ����ת�䣬ת�亽��������\sigma*radius�����㡣
% ��ÿ�κ������Ⱥ������е���distNew(i) = dist(i)+2*radius*(\sigma(i)-sin(\sigma(i)))������iΪ;���ĵ�i��У�����Ӧ��������
% 

% �ڶ��ʵ��жϼ��������
tag = 1;  % �ɹ������1��ʧ��Ϊ-1
sigma = zeros(length(route)-1,1);
dist = zeros(length(route)-1,1);
sigma(1)=0;   % ���ڵ�A�����ĳ������ζ��ԣ���������û�з����仯��
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