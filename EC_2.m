
ur5=ur5_interface;
K = 0.3;
offset=[0;2/180*pi;0;0;0;0];

input('Please move robot to final hitting position.');
pause(1);
theta_3 = ur5.get_current_joints();
theta_3_up = theta_3-offset;
g_theta_3 = ur5FwdKin(theta_3);
g_theta_3_up = ur5FwdKin(theta_3_up);

input('Please move robot to second hitting position.');
pause(1);
theta_2 = ur5.get_current_joints();
theta_2_up = theta_2-offset;
g_theta_2 = ur5FwdKin(theta_2);
g_theta_2_up = ur5FwdKin(theta_2_up);

input('Please move robot to start position.');
pause(1);
theta_1 = ur5.get_current_joints();
g_theta_1 = ur5FwdKin(theta_1);



%% the robot is at start postion now 
%move to second postion
ur5RRcontrol(g_theta_2_up, K, ur5);
ur5RRcontrol(g_theta_2, K, ur5);

s_offset = [0;0;0;0;60/180*pi;0];
%do scooping here
disp('hitting first target!');
rot_2 = ur5.get_current_joints();
rot_2_2 = rot_2 + s_offset;
ur5.move_joints(rot_2_2, 1);
pause(1);
ur5.move_joints(rot_2,1);
pause(1);
ur5RRcontrol(g_theta_2_up, K, ur5);

% move to destination
ur5RRcontrol(g_theta_3_up, K, ur5);
ur5RRcontrol(g_theta_3, K, ur5);

%do dumping here
disp('hitting second target!');
rot_3 = ur5.get_current_joints();
rot_3_2 = rot_3 + s_offset;
ur5.move_joints(rot_3_2, 1);
pause(1);
ur5.move_joints(rot_3,1);
pause(1);
ur5RRcontrol(g_theta_3_up, K, ur5);

disp('Done!');

 