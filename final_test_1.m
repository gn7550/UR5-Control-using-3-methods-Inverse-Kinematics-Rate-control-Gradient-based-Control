q0=[-pi/3;-pi/4;pi/5;pi/8;pi/4;0];
%q0=[-pi/2;-pi/2;0;-pi/2;0;0];
gst=ur5FwdKin(q0); %with new function,

%gst_2=ur5FwdKin_2(q1); %with original function

ur5=ur5_interface;
%home_old = tf_frame('base_link', 'home_old', gst_2);
desired = tf_frame('base_link', 'desired', gst);
theta=ur5InvKin(gst)
theta1=theta(1:6,1);
ur5.move_joints(ur5.home,5);
pause(5);
ur5.move_joints(q0,5);
pause(5);
ur5.move_joints(ur5.home,5);
pause(5);
ur5.move_joints(theta1,5);
pause(5);
%ur5.move_joints(theta1,5);
