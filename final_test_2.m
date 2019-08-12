

%% This file tests whether RR control is correct!
q0=[-pi/3;-pi/4;pi/5;pi/8;pi/4;0];
%q0=[-pi/2;-pi/2;0;-pi/2;0;0];
gst=ur5FwdKin(q0); %with new function,

%gst_2=ur5FwdKin_2(q1); %with original function

ur5=ur5_interface;

q = [-pi/3; pi/2; 0; pi/7; pi/6; pi/2];
% we start from a non-singularity position! Comment out the home position in
% the beginning of the function to run this part!!!
joints=[-pi/3;-pi/4;pi/5;pi/8;pi/4;0];
g_home = ur5FwdKin(joints);

ur5.move_joints(ur5.home,5);
pause(5);

new_home = tf_frame('base_link', 'new_home', g_home);
ur5.move_joints(joints,5);
pause(5);

new_joints = [-pi/3;-pi/4;pi/3;pi/8;pi/4;0];
g_des = ur5FwdKin(new_joints);
destination = tf_frame('base_link', 'destination', g_des);
K = 0.5;
finalerr = ur5RRcontrol(g_des, K, ur5)
pause(15);
