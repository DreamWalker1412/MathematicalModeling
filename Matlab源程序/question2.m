% ͨ��������һ��ģG=(V,E),��������ÿ����ǰ������;
% ����״̬ջstackA�洢��������״̬
% ����״̬ջstackB�洢������״̬��
% ����״̬ջstackC�洢��һ������״̬��
% ����״̬ջstackD�洢����״̬��
% ����תָ̬��p=[i,j,Edge(j,i),errorV,errorH,func,�ܺ�������]�洢��ǰ״̬,�ǵ�ǰ״̬�Ŀɴ���ż���ΪfesibleP;
% ÿһ״̬����[��ǰ���ţ���һ����ţ�ǰһ�κ������ȣ�У��ǰerrorV,У��ǰerrorH,��������,�ܺ�������]
% ��ʼ��p=[1,1,0,0,0,0,0]
% ��ʼ��stackA = [];
% ��ʼ��stackB(i) = [fesibleP(i),i,Edge(i,fesibleP(i)),errorV,errorH,func,distSum];

stackA = [];
stackB = [];
stackC = [];
stackD = [];
p = [1,1,0,0,0,0,0];
count = 1;
route = cell(iterNum,3);

while count<=iterNum
    fesibleB = intersect(find( Edge( p(1),length(V(:,1)) )),find(  Edge( p(1),length(V(:,1)) )<= theta/delta -p(3) ));
    if ~isempty(fesibleB)
        stackA = [stackA;p];
        p = [length(V(:,1)),p(1),Edge( p(1),length(V(:,1))),p(4)+Edge( p(1),length(V(:,1)))*delta,p(5)+Edge( p(1),length(V(:,1)))*delta,func(p(1),length(V(:,1)),V,shortestDistToB),p(7)+Edge( p(1),length(V(:,1)))];      %���״̬
        stackA = [stackA;p]; %#ok<*AGROW>
        route{count,1}=[length(V(:,1)),p(2)];
        
        i=1;
        while p(2)~=1
            route{count,1}(i)=p(2);
            temp = find(stackA(:,1)==p(2));
            p(2)=stackA(temp(end),2);
            i=i+1;
        end
        route{count,1} = fliplr( [length(V(:,1)),route{count,1},1]);   % ��·��������ʾ
        [route{count,2},route{count,3}] = judgement2(route{count,1},Edge,distMax,theta/delta,V);
      
        p = [1,1,0,0,0,0,0];
        stackA = [];
        stackB = [];
        stackC = [];
        stackD = [];
        if route{count,3}<0
             continue;
        else
             count = count+1;
             continue;
        end
    else     % ���û����ֹ����Ѱp�Ŀ��е���
        fesibleP =intersect( find(Edge( p(1),: )) , find(Edge( p(1),: )<=distMax-p(3)));
        if ~isempty(fesibleP)                                %  �����е���Ϊ��
            
            stackC = [];
            if V(p(1),4)==1 % �жϵ�ǰ������
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
            if  ~isempty(stackC)                  % ��ջC�и��µ�ǰ״̬
                stackA = [stackA;p];
                stackC = sortrows(stackC,6);
                randTemp = randN(stackC);
                p = stackC(randTemp,:);
                stackC(randTemp,:) = [];
                stackB = [stackB;stackC];         % ��û�ù���״̬ջC����״̬ջB
            else
                if  ~isempty(stackB)
                    stackB = sortrows(stackB,6);  % ջ��ջC��ջD�Ϊ�գ���ջB��ȡ�����µ�ǰ״̬
                    randTemp = randN(stackB);
                    p = stackB(randTemp,:);
                    stackB(randTemp,:)=[];
                else
                    false
                    break
                end
            end
        else                                  % ���fesiblePΪ�գ���ô����һ��stackC������һ����ѡ״̬��Ϊ�µ�p������p��stackB��stackC�����
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
            else                               % ���stackCҲΪ�գ���ջB��ȡ��״̬
                if  ~isempty(stackB)
                    stackB = sortrows(stackB,6);
                    randTemp = randN(stackB);
                    p = stackB(randTemp,:);
                    stackB(randTemp,:)=[];
                else
                    false                      % ����㷨����������޽�
                    break
                end
            end
        end
        
    end
end

routeLength = zeros(iterNum,1);
routeDist =  zeros(iterNum,1);
routeNum =  zeros(iterNum,1);
routeTag = zeros(iterNum,1);
for i = 1:iterNum
    routeNum(i) = i;
    routeLength(i)=length(route{i,1});
    routeDist(i)= route{i,2};
    routeTag(i) = route{i,3};
end
routeInfo = [routeNum,routeLength,routeDist];
routeInfo = sortrows(routeInfo,3);
shortestRouteLength = min(routeLength);  % ���ٽڵ���
shortestRouteDist = min(routeDist); % ��̺���

