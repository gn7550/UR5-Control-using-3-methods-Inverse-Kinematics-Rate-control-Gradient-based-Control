
% function finalerr = ur5InvKinControl(g_des, K, ur5)
function finalerr = ur5InvKinControl(g_des, K, ur5)
q= ur5.get_current_joints();
sign_q3 = sign(q(3));

q = ur5InvKin(g_des);
norm_max = 2*pi*6;
q_desired = zeros(6,1);
for i = 1:6
    if q(2,i) <= 0 && isequal(sign(q(3,i)),sign_q3) && norm(q(:,i)) <= norm_max
        q_desired = q(:,i);
        norm_max = norm(q(:,i));
    end
end
if isequal(q_desired, zeros(6,1))
    disp('could not find inverse kinematics solution given constraints')
end

ur5.move_joints(q_desired,8);
pause(8.5);

qa=ur5.get_current_joints();
xi = getXi(g_des\(ur5FwdKin(qa)) );
v=xi(1:3);
w=xi(4:6);
norm(v)

end














% function finalerr = ur5InvKinControl(g_des, K, ur5)
function finalerr = ur5InvKinControl(g_des, K, ur5)
q_start = ur5.get_current_joints();  % 6x1
sign_q3 = sign(q_start(3));

q = ur5InvKin(g_des);  % q is a 6x8
norm_max = 2*pi*6;
q_desired = zeros(6,1);
for i = 1:6
    if q(2,i) <= 0 && isequal(sign(q(3,i)),sign_q3) && norm(q(:,i)) <= norm_max
        q_desired = q(:,i);
        norm_max = norm(q(:,i));
    end
end
if isequal(q_desired, zeros(6,1))
    disp('could not find inverse kinematics solution given constraints')
    return
end

ur5.move_joints(q_desired,8);
pause(8.5);

qa=ur5.get_current_joints();
xi = getXi(g_des\(ur5FwdKin(qa)) );
v=xi(1:3);
w=xi(4:6);
norm(v)

end

