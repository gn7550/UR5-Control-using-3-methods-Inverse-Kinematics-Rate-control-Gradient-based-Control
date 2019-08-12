
ur5=ur5_interface;
q0=[-pi/3;-pi/4;pi/5;pi/8;pi/4;0];
%q0=[-pi/2;-pi/2;0;-pi/2;0;0];
gst=ur5FwdKin(q0); %with new function,

K=0.3;

ur5.move_joints(ur5.home,5);
pause(5);
g_start = [0, -1, 0, 0.47; 0, 0, 1, 0.55; -1, 0, 0, 0.12; 0, 0, 0, 1];
g_end = [0, -1, 0, -.3; 0, 0, 1, 0.39; -1, 0, 0, 0.12; 0, 0, 0, 1];

end_pos = tf_frame('base_link', 'end_pos', g_end);
end_theta = ur5InvKin(g_end); %angle from inv kin.m
ur5.move_joints(end_theta(:,1),5);
pause(5);

start_pos = tf_frame('base_link', 'start_pos', g_start);
start_theta = ur5InvKin(g_start); %angle from inv kin.m
ur5.move_joints(start_theta(:,1),5);
pause(5);


%ur5InvKinControl(g_end, K, ur5);
ur5RRcontrol(g_end, K, ur5);
pause(15);

