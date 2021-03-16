function [bonusLenth] = bonusLength(phi)
    r=200.0;
    a=1+tan(phi)^2;
    b=-2*(1+1/cos(phi))*tan(phi);  
    c=(1+1/cos(phi))^2-4;
    x=r*(-b+sqrt(b^2-4*a*c))/(2*a);
    if phi<=2*atan(1/2)
       bonusLenth=(-phi+2*asin(x/(2*r)))*r - x/cos(phi)+r*tan(phi);
    else
       bonusLenth=(2*pi-phi-2*asin(x/(2*r)))*r - x/cos(phi)+r*tan(phi);
    end
end