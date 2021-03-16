% 通过对问题一建模G=(V,E),给出假设每条边前进方向;
% 给出状态栈stackA存储已搜索的状态
% 给出状态栈stackB存储待搜索状态集
% 给出状态栈stackC存储下一步可行状态集
% 给出状态栈stackD存储废弃状态集
% 给出转态指针p=[i,j,Edge(j,i),errorV,errorH,func]存储当前状态,记当前状态的可达点编号集合为fesibleP;
% 每一状态包括[当前点编号，上一个编号，前一段航迹长度，校正前errorV,校正前errorH,评估函数]
% 初始化p=[1,1,0,0,0,0]
% 初始化stackA = [];
% 初始化stackB(i) = [fesibleP(i),i,Edge(i,fesibleP(i)),errorV,errorH,func];
% alpha1 = 25; alpha2 = 15; beta1 = 20; beta2 = 25; theta = 30; delta = 0.001; % 第一组数据
  alpha1 = 20; alpha2 = 10; beta1 = 15; beta2 = 20; theta = 20; delta = 0.001; % 第二组数据

stackA = [];
stackB = [];
stackC = [];

p = [1,1,0,0,0,0,0];
count = 1;
iterNum = 1;
route = cell(iterNum,3);

while count<=iterNum
    fesibleB = intersect(find( Edge( p(1),length(V(:,1)) )),find(  Edge( p(1),length(V(:,1)) )<= theta/delta -p(3) ));
    if ~isempty(fesibleB)
        stackA = [stackA;p];
        tempP = [length(V(:,1)),p(1),Edge( p(1),length(V(:,1))),p(4)+Edge( p(1),length(V(:,1)))*delta,p(5)+Edge( p(1),length(V(:,1)))*delta,func(p(1),length(V(:,1)),V,shortestDistToB),p(7)+Edge( p(1),length(V(:,1)))];        stackA = [stackA;tempP]; %#ok<*AGROW>
        
     
            route{count,1}=[length(V(:,1)),tempP(2)];
            i=1;
            while tempP(2)~=1
                route{count,1}(i)=tempP(2);
                temp = find(stackA(:,1)==tempP(2));
                tempP(2)=stackA(temp(end),2);
                i=i+1;
            end
            route{count,1} = fliplr( [length(V(:,1)),route{count,1},1]);   % 将路径正向显示
            [route{count,2},route{count,3}] = judgement(route{count,1},Edge,distMax,theta/delta);
            if route{count,2} < shortestRouteLength   % 路径长度没有优化，还原栈A状态
                count = count+1;
                continue
            else
                stackD = [stackD;p];
                p = [1,1,0,0,0,0,0];
                stackA = [];
                stackB = [];
                stackC = [];
                
                continue;
               
            end
        
    else     % 如果没有终止，搜寻p的可行点域
        fesibleP =intersect( find(Edge( p(1),: )) , find(Edge( p(1),: )<=distMax-p(3)));
        if ~isempty(fesibleP)                                %  若可行点域不为空
            stackC = [];
            if V(p(1),4)==1 % 判断当前点类型
                for i = 1:length(fesibleP)
                    stackC =[stackC; [fesibleP(i),p(1),Edge(p(1),fesibleP(i)),Edge(p(1),fesibleP(i))*delta,Edge(p(1),fesibleP(i))*delta+p(5),func(p(1),fesibleP(i),V,shortestDistToB),p(7)+Edge(p(1),fesibleP(i))]];
                end
            else
                for i = 1:length(fesibleP)
                    stackC =[stackC; [fesibleP(i),p(1),Edge(p(1),fesibleP(i)),Edge(p(1),fesibleP(i))*delta+p(4),Edge(p(1),fesibleP(i))*delta,func(p(1),fesibleP(i),V,shortestDistToB),p(7)+Edge(p(1),fesibleP(i))]];
                end
            end
            if  ~isempty(stackC) && ~isempty(stackD)
                stackC = setdiff(stackC,stackD,'row');
            end
            if  ~isempty(stackC)                  % 从栈C中更新当前状态
                stackA = [stackA;p];
                stackC = sortrows(stackC,6);
                randTemp = randN(stackC);
                p = stackC(randTemp,:);     
                stackC(randTemp,:) = [];
                stackB = [stackB;stackC];         % 将没用过的状态栈C并入状态栈B
            else
                if  ~isempty(stackB) && ~isempty(stackD)
                     stackB = setdiff(stackB,stackD,'row');
                end
                if  ~isempty(stackB) 
                    stackB = sortrows(stackB,6);  % 栈若栈C与栈D差集为空，从栈B中取出更新当前状态
                    randTemp = randN(stackB);
                    p = stackB(randTemp,:);
                    stackB(randTemp,:)=[];
                else
                    false
                    break
                end
            end
        else                                  % 如果fesibleP为空，那么从上一个stackC中挑出一个候选状态作为新的p，并将p从stackB和stackC中清除
            stackD = [stackD;p];
            if  ~isempty(stackC)                    
                p = stackC(randN(stackC),:);
                stackC(1,:) = [];
                for i = length(stackB(:,1)):-1:1   
                    if stackB(i,:)== p
                        stackB(i,:)=[];
                        break;
                    end
                end
            else                               % 如果stackC也为空，从栈B中取出状态
                if  ~isempty(stackB) 
                    stackB = sortrows(stackB,6);  
                    randTemp = randN(stackB);
                    p = stackB(randTemp,:);
                    stackB(randTemp,:)=[];
                else
                    false                      % 标记算法出错或问题无解
                    break
                end
            end
        end
         
    end
end

routeLength = zeros(iterNum,1);
routeDist =  zeros(iterNum,1);
routeNum =  zeros(iterNum,1);
for i = 1:iterNum
    routeNum(i) = i;
    routeLength(i)=length(route{i,1});
    routeDist(i)= route{i,2};
end
routeInfo = [routeNum,routeLength,routeDist];
routeInfo = sortrows(routeInfo,3);
shortestRouteLength = min(min(routeDist),shortestRouteLength);
clear  temp errorH errorV fesibleB fesibleP iterNum i p x ;