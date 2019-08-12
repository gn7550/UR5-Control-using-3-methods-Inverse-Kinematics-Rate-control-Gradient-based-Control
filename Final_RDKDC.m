%% This Matlab file teaches the robot start and end postions. Then ask the 
% user for input to choose among three methods. Then run the corresponding method to 
% get the robot to the destination

% initialize and declare everything here
ur5=ur5_interface;
K = 0.3;

demonstration = input('Enter 0 for simulation, 1 for physical robot: ');
if demonstration == 0
    
    ur5.move_joints(ur5.home,5);
    pause(5);
    
    %g_start = [0, -1, 0, 0.47; 0, 0, 1, 0.55; -1, 0, 0, 0.12; 0, 0, 0, 1];
    %g_end = [0, -1, 0, -.3; 0, 0, 1, 0.39; -1, 0, 0, 0.12; 0, 0, 0, 1];
    joints=[-pi/3;-pi/4;pi/5;pi/8;pi/4;0];
    g_start= ur5FwdKin(joints);
    new_joints = [-pi/2;-pi/4;pi/7;pi/5;pi/3;0];
    g_end = ur5FwdKin(new_joints);
    
    end_pos = tf_frame('base_link', 'end_pos', g_end);
    end_theta_vec = ur5InvKin(g_end); %angle from inv kin.m
    end_theta = end_theta_vec(:,1);
    ur5.move_joints(end_theta,5);
    pause(5);
    
    start_pos = tf_frame('base_link', 'start_pos', g_start);
    start_theta = ur5InvKin(g_start); %angle from inv kin.m
    ur5.move_joints(start_theta(:,1),5);
    pause(5);
    
elseif demonstration == 1

    % move the robot to the end position here
    input('Please move robot to end position.');
    pause(1);
    end_theta = ur5.get_current_joints();
    g_end = ur5FwdKin(end_theta);

    % move the robot to the start configuration
    input('Please move robot to start position.');
    pause(1);
    start_theta = ur5.get_current_joints();
end

% get the intermediate frame
offset = [0;3/180*pi;0;0;0;0];
if(end_theta(2) < -pi/2)
    intermediate_theta = end_theta + offset;
    intermediate_start = start_theta + offset;
else 
    intermediate_theta = end_theta-offset;
    intermediate_start = start_theta - offset;
    
end

    g_intermediate= ur5FwdKin(intermediate_theta);
    g_intermediate_start = ur5FwdKin(intermediate_start);


    %ur5.move_joints(ur5.home,6);
    %ur5InvControl(g_intermediate,K,ur5);
    
% ask the user for command
robot_at_start = 1;
goal_frame = tf_frame('base_link','goalFrame',g_end);
disp("input which method to use");
prompt = 'Input 1 for Inverse Kin, 2 for RR Control, 3 for Gradient control, or any other number to exit: ';
user_selection = input(prompt);
while isnumeric(user_selection)
    if user_selection == 1
        disp("user selected inverse kinematics");
        
        ur5InvKinControl(g_intermediate_start, K, ur5);
        ur5InvKinControl(g_intermediate, K, ur5);
        %pause(2);
        ur5InvKinControl(g_end, K, ur5);
        robot_at_start = 0;
        
    elseif user_selection == 2
        disp("user selected resolved rate control");
        ur5RRcontrol(g_intermediate_start, K, ur5);
        ur5RRcontrol(g_intermediate, K, ur5);
        pause(2);
        ur5RRcontrol(g_end, K, ur5);
        robot_at_start = 0;
    elseif user_selection == 3
        disp("user selected gradident control");
        K_g = 1;
        ur5Gradident(g_intermediate_start, K_g, ur5);
        ur5Gradident(g_intermediate, K_g, ur5);
        ur5Gradident(g_end,K_g, ur5);
        robot_at_start = 0;
    else
        disp('Input not in range 1-3. Exiting.');
        break
    end
    
  
    disp('would you like to try a different algorithm?');
    user_selection = input(prompt);

    
    if all(user_selection >= 1 & user_selection <=3) && ~robot_at_start
        disp("stand back! moving robot back to start");
        pause(3);
        ur5.move_joints(intermediate_theta, 5);
        pause(5);
        ur5.move_joints(intermediate_start, 5);
        pause(5);
        ur5.move_joints(start_theta, 5);
        pause(6);
    end
    

end