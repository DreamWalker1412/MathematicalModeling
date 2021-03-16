function [value] = func2(i,j,V,G)
    value = norm(V(i,1:3)-V(j,1:3))+ shortestpath(G,j,length(V(:,1)));
end