function [distSum,tag] = judgement(route,Edge,distMax,lastRadius)
tag = 1;  %成功则输出1，失败为-1
for i = 1:length(route)-3
 if (Edge(route(i),route(i+1))+Edge(route(i+1),route(i+2)))> distMax
     tag = -1;
 end
end
if (Edge(route(end-2),route(end-1))+Edge(route(end-1),route(end)))> lastRadius
     tag = -1;
end

distSum = 0;
for i = 1:length(route)-1
    
    distSum = distSum + Edge(route(i),route(i+1));
end
end