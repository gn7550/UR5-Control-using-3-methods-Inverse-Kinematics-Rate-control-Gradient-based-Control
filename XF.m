%function accepts a 6*1 vector and returns a 4*4 homogeneous transformation
%v=[x;y;z;theta1;theta2;theta3]
function g = XF(v)

if (size(v,1)~=6||size(v,2) ~= 1)
        error('The input vector is 6*1');        
end
%create R-3*3, p, a 3*1 0 matrix
R=EULERXYZ(v(4:6)); 
p=v(1:3);
Mzero=[0;0;0];
g=vertcat([R p],[Mzero' 1]);