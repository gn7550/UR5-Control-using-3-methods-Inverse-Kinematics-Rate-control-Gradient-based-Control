function xi = getXi(g)
% This function calculates the twist given a transformation matrix

% Get R and p from the transformation g
R = g(1:3,1:3);
p = g(1:3,4);

% Find theta
theta = acos((trace(R)-1)/2);

% Find w vector
w = (1/(2*sin(theta)))*[R(3,2) - R(2,3); R(1,3) - R(3,1); R(2,1) - R(1,2)];

% Find the v vector
w_hat = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];
M = (eye(3) - R)*w_hat + w*w'*theta;
v = M\p;

% Get twist from w and v
xi = [v; w]*theta;
end