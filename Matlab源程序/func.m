function [value] = func(i,j,V,shortestDistToB)
    value = norm(V(i,1:3)-V(j,1:3))+ shortestDistToB(j);
end