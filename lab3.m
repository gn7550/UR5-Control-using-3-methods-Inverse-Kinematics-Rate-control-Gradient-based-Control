% %Lab3 executive file
% 
% 
%% Test UR5 Forward Kinematics
ur5=ur5_interface;
ur5.move_joints(ur5.home,10);
pause(2);

%Joint vector q1
q1=[pi/2;-pi/3;-pi/6;pi/8;pi/3;pi/8];

gst=ur5FwdKin(q1);

fwdKinToolFrame = tf_frame('base_link','fwdKinToolFrame',eye(4));
pause(6);

fwdKinToolFrame.move_frame('world',gst);
pause(6);
ur5.move_joints(q1,10);
pause(10);

ur5.get_current_transformation('base_link','ee_link');
ur5FwdKin(ur5.get_current_joints());

%% Test UR5 Body Jacobian 
%Joint vector q2
q2=[-pi/2;pi/8;pi/6;pi/6;pi/15;pi/8];   

J = ur5BodyJacobian(q2);
I=eye(6);
e1=I(1,:);
e2=I(2,:);
e3=I(3,:);
e4=I(4,:);
e5=I(5,:);
e6=I(6,:);

gst= ur5FwdKin(q2);
p=0.01;
gst1_plus=ur5FwdKin(q2+p*e1');
gst1_minus=ur5FwdKin(q2-p*e1');

gst2_plus=ur5FwdKin(q2+p*e2');
gst2_minus=ur5FwdKin(q2-p*e2');

gst3_plus=ur5FwdKin(q2+p*e3');
gst3_minus=ur5FwdKin(q2-p*e3');

gst4_plus=ur5FwdKin(q2+p*e4');
gst4_minus=ur5FwdKin(q2-p*e4');

gst5_plus=ur5FwdKin(q2+p*e5');
gst5_minus=ur5FwdKin(q2-p*e5');

gst6_plus=ur5FwdKin(q2+p*e6');
gst6_minus=ur5FwdKin(q2-p*e6');

dg1=(1/(2*p))*(gst1_plus-gst1_minus);
dg2=(1/(2*p))*(gst2_plus-gst2_minus);
dg3=(1/(2*p))*(gst3_plus-gst3_minus);
dg4=(1/(2*p))*(gst4_plus-gst4_minus);
dg5=(1/(2*p))*(gst5_plus-gst5_minus);
dg6=(1/(2*p))*(gst6_plus-gst6_minus);

xi1_prime_hat=(gst\dg1);
xi1_prime_w1= xi1_prime_hat(3,2);
xi1_prime_w2=xi1_prime_hat(1,3);
xi1_prime_w3=xi1_prime_hat(2,1);
xi1_prime_v=xi1_prime_hat(1:3,4);
xi1_prime=[xi1_prime_v;xi1_prime_w1;xi1_prime_w2;xi1_prime_w3];

xi2_prime_hat=(gst\dg2);
xi2_prime_w1= xi2_prime_hat(3,2);
xi2_prime_w2=xi2_prime_hat(1,3);
xi2_prime_w3=xi2_prime_hat(2,1);
xi2_prime_v=xi2_prime_hat(1:3,4);
xi2_prime=[xi2_prime_v;xi2_prime_w1;xi2_prime_w2;xi2_prime_w3];

xi3_prime_hat=(gst\dg3);
xi3_prime_w1= xi3_prime_hat(3,2);
xi3_prime_w2=xi3_prime_hat(1,3);
xi3_prime_w3=xi3_prime_hat(2,1);
xi3_prime_v=xi3_prime_hat(1:3,4);
xi3_prime=[xi3_prime_v;xi3_prime_w1;xi3_prime_w2;xi3_prime_w3];

xi4_prime_hat=(gst\dg4);
xi4_prime_w1= xi4_prime_hat(3,2);
xi4_prime_w2=xi4_prime_hat(1,3);
xi4_prime_w3=xi4_prime_hat(2,1);
xi4_prime_v=xi4_prime_hat(1:3,4);
xi4_prime=[xi4_prime_v;xi4_prime_w1;xi4_prime_w2;xi4_prime_w3];

xi5_prime_hat=(gst\dg5);
xi5_prime_w1= xi5_prime_hat(3,2);
xi5_prime_w2=xi5_prime_hat(1,3);
xi5_prime_w3=xi5_prime_hat(2,1);
xi5_prime_v=xi5_prime_hat(1:3,4);
xi5_prime=[xi5_prime_v;xi5_prime_w1;xi5_prime_w2;xi5_prime_w3];

xi6_prime_hat=(gst\dg6);
xi6_prime_w1= xi6_prime_hat(3,2);
xi6_prime_w2=xi6_prime_hat(1,3);
xi6_prime_w3=xi6_prime_hat(2,1);
xi6_prime_v=xi6_prime_hat(1:3,4);
xi6_prime=[xi6_prime_v;xi6_prime_w1;xi6_prime_w2;xi6_prime_w3];

Japprox=[xi1_prime,xi2_prime,xi3_prime,xi4_prime,xi5_prime,xi6_prime];

norm(Japprox-J)


%% Test Manipulability
% Joint vector q3
for t=-50:1:50
    theta3=t*pi/200;
    q3=[-pi/6;-pi/3;theta3;0;-pi/2;pi/8];
    g_mani=ur5FwdKin(q3);
    J_mani=ur5BodyJacobian(q3);
    
    %legend sigmamin with a dot
    m_theta3=manipulability(J_mani,'sigmamin');
    plot(theta3,m_theta3,'.')
    hold on;
    
    %legend detjac with a star
    m_det=manipulability(J_mani,'detjac');
    plot(theta3,m_det,'o')
    hold on;
    
    %legend invcond with a cross
    m_inv=manipulability(J_mani,'invcond');
    plot(theta3,m_inv,'+')
    hold on;
end
hold off;
xlabel('theta 3');
ylabel('Manipulability Measure');

xlabel('theta 3');
ylabel('Manipulability Measure');

%% Test function getXi
% Getting arbitrary joint vector
q4=[0.65;0.23;-0.45;-0.99;0.83;-0.657];

g_getXi=ur5FwdKin(q4)

% Getting Xi value 
Xi=getXi(g_getXi);   
v_getXi=Xi(1:3);
w_getXi=Xi(4:6);
w_hat_getXi=[0,-w_getXi(3),w_getXi(2);w_getXi(3),0,-w_getXi(1);-w_getXi(2),w_getXi(1),0];

% Result comparision
finXi_test=[w_hat_getXi,v_getXi;zeros(1,4)];
expm(finXi_test)


%% Test Resolved Rate Controller  
ur5 = ur5_interface();
q = [pi/3; -pi/2; 0; pi/7; pi/6; pi/2];
% we start from a non-singularity position! Comment out the home position in
% the beginning of the function to run this part!!!
joints=[pi/3;-1.5708;pi/2;-1.5708;pi/3;pi/3];
g_home = ur5FwdKin(joints);
new_home = tf_frame('base_link', 'new_home', g_home);
ur5.move_joints(joints,2);
pause(5);

new_joints = [pi/3;-1.5708;pi/2;pi/6;pi/3;pi/3];
g_des = ur5FwdKin(new_joints);
destination = tf_frame('base_link', 'destination', g_des);
K = 0.5;
finalerr = ur5RRcontrol(g_des, K, ur5)
pause(15);

% move back to the "new" home position
ur5.move_joints(joints,2);
pause(5);

% now move to a singular point
%new_joints2 = [pi/3;pi/6;pi/2;pi/2;0;0];
new_joints2 = [0;-pi/2;0;0;0;0];
g_des2 = ur5FwdKin(new_joints2);
destination_singular = tf_frame('base_link', 'destination_singular', g_des2);
K = 0.5;
finalerr = ur5RRcontrol(g_des2, K, ur5)

