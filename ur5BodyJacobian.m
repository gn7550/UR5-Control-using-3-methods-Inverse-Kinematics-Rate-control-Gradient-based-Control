function J = ur5BodyJacobian(joints)
% input: joints is 6*1 vector where joints (i) correspond to joint i in
% gazebo setting.
% output: J is 6*6 body based Jacobian matrix

% Define the lengths of the UR5 Robot
L0 = 0.0892;
L1 = 0.425;
L2 = 0.392;
L3 = 0.1093;
L4 = 0.09475;
L5 = 0.0825;

% Define joint angles from input

offset=[pi/2;pi/2;0;pi/2;0;0];
joints=joints+offset;

theta1 = joints(1);
theta2 = joints(2);
theta3 = joints(3);
theta4 = joints(4);
theta5 = joints(5);
theta6 = joints(6);

% Define w's for the twist
w1 = [0 0 1]';
w2 = [1 0 0]';
w3 = [1 0 0]';
w4 = [1 0 0]';
w5 = [0 0 1]';
w6 = [1 0 0]';


w1_hat = [0 -1 0; 1 0 0; 0 0 0];
%w5_hat = w1_hat;
w2_hat = [0 0 0; 0 0 -1; 0 1 0];
%w2_hat = w1_hat;
w3_hat = w2_hat;
w4_hat = w2_hat;
w5_hat = w1_hat;
w6_hat = w1_hat;

% Define q vectors for each twist
q1 = [0 0 0]';
q2 = [0 0 L0]';
q3 = [0 0 L0+L1]';
q4 = [0 0 L0+L1+L2]';
q5 = [L3 0 0]';
q6 = [0 0 L0+L1+L2+L4]';

% Define the twist skew matrices
twist1 = [w1_hat cross(-w1,q1); [0 0 0] 0];
twist2 = [w2_hat cross(-w2,q2); [0 0 0] 0];
twist3 = [w3_hat cross(-w3,q3); [0 0 0] 0];
twist4 = [w4_hat cross(-w4,q4); [0 0 0] 0];
twist5 = [w5_hat cross(-w5,q5); [0 0 0] 0];
twist6 = [w6_hat cross(-w6,q6); [0 0 0] 0];

% Define the twist not as a skew
tw1 = [cross(-w1,q1); w1];
tw2 = [cross(-w2,q2); w2];
tw3 = [cross(-w3,q3); w3];
tw4 = [cross(-w4,q4); w4];
tw5 = [cross(-w5,q5); w5];
tw6 = [cross(-w6,q6); w6];


% Define exp(twist*theta)
screw1 = eye(4) + theta1*twist1 + (1 - cos(theta1))*twist1^2 + (theta1 - sin(theta1))*twist1^3;
screw2 = eye(4) + theta2*twist2 + (1 - cos(theta2))*twist2^2 + (theta2 - sin(theta2))*twist2^3;
screw3 = eye(4) + theta3*twist3 + (1 - cos(theta3))*twist3^2 + (theta3 - sin(theta3))*twist3^3;
screw4 = eye(4) + theta4*twist4 + (1 - cos(theta4))*twist4^2 + (theta4 - sin(theta4))*twist4^3;
screw5 = eye(4) + theta5*twist5 + (1 - cos(theta5))*twist5^2 + (theta5 - sin(theta5))*twist5^3;
screw6 = eye(4) + theta6*twist6 + (1 - cos(theta6))*twist6^2 + (theta6 - sin(theta6))*twist6^3;

% Define gst(0)
gst_0 = [0 0 1 L5+L3; -1 0 0 0; 0 -1 0 L1+L2+L4+L0; 0 0 0 1];

% Generate gst given the screws and gst(0)
g1 = screw1*screw2*screw3*screw4*screw5*screw6*gst_0;
g2 = screw2*screw3*screw4*screw5*screw6*gst_0;
g3 = screw3*screw4*screw5*screw6*gst_0;
g4 = screw4*screw5*screw6*gst_0;
g5 = screw5*screw6*gst_0;
g6 = screw6*gst_0;

% Define the Adjoint to Calc Jacobian
R1 = [g1(1,1) g1(1,2) g1(1,3); g1(2,1) g1(2,2) g1(2,3); g1(3,1) g1(3,2) g1(3,3)];
p_hat1 = [0 -g1(3,4) g1(2,4); g1(3,4) 0 -g1(1,4); -g1(2,4) g1(1,4) 0];
Ad_inv1 = [R1' -R1'*p_hat1; zeros(3) R1'];
R2 = [g2(1,1) g2(1,2) g2(1,3); g2(2,1) g2(2,2) g2(2,3); g2(3,1) g2(3,2) g2(3,3)];
p_hat2 = [0 -g2(3,4) g2(2,4); g2(3,4) 0 -g2(1,4); -g2(2,4) g2(1,4) 0];
Ad_inv2 = [R2' -R2'*p_hat2; zeros(3) R2'];
R3 = [g3(1,1) g3(1,2) g3(1,3); g3(2,1) g3(2,2) g3(2,3); g3(3,1) g3(3,2) g3(3,3)];
p_hat3 = [0 -g3(3,4) g3(2,4); g3(3,4) 0 -g3(1,4); -g3(2,4) g3(1,4) 0];
Ad_inv3 = [R3' -R3'*p_hat3; zeros(3) R3'];
R4 = [g4(1,1) g4(1,2) g4(1,3); g4(2,1) g4(2,2) g4(2,3); g4(3,1) g4(3,2) g4(3,3)];
p_hat4 = [0 -g4(3,4) g4(2,4); g4(3,4) 0 -g4(1,4); -g4(2,4) g4(1,4) 0];
Ad_inv4 = [R4' -R4'*p_hat4; zeros(3) R4'];
R5 = [g5(1,1) g5(1,2) g5(1,3); g5(2,1) g5(2,2) g5(2,3); g5(3,1) g5(3,2) g5(3,3)];
p_hat5 = [0 -g5(3,4) g5(2,4); g5(3,4) 0 -g5(1,4); -g5(2,4) g5(1,4) 0];
Ad_inv5 = [R5' -R5'*p_hat5; zeros(3) R5'];
R6 = [g6(1,1) g6(1,2) g6(1,3); g6(2,1) g6(2,2) g6(2,3); g6(3,1) g6(3,2) g6(3,3)];
p_hat6 = [0 -g6(3,4) g6(2,4); g6(3,4) 0 -g6(1,4); -g6(2,4) g6(1,4) 0];
Ad_inv6 = [R6' -R6'*p_hat6; zeros(3) R6'];


% Define twist dagger
twisted1 = Ad_inv1*tw1;
twisted2 = Ad_inv2*tw2;
twisted3 = Ad_inv3*tw3;
twisted4 = Ad_inv4*tw4;
twisted5 = Ad_inv5*tw5;
twisted6 = Ad_inv6*tw6;

% Define body Jacobian
J = [twisted1 twisted2 twisted3 twisted4 twisted5 twisted6];

end

