function [randNum] = randN(stack)
    p = zeros(1,length(stack(:,1)));
    p(1)=1;
    for i =1:length(stack(:,1))-1
        p(i+1) = p(i)/exp(i);
    end
    alphabet = [];
    sumP = sum(p);
    for i =1:length(stack(:,1))
        alphabet = [alphabet,i]; %#ok<AGROW>
        p(i)= p(i)/sumP;
    end
    
    randNum = randsrc(1,1,[alphabet; p]);
end
