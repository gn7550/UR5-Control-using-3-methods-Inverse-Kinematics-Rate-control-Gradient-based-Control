% function finalerr = ur5RRcontrol(g_des, K, ur5)
function finalerr = ur5RRcontrol(g_des, K, ur5)
q= ur5.get_current_joints();
t_step = 1;
loop_time=0;

while(1)
    
% Checking norms
xi = getXi(g_des\(ur5FwdKin(q)) );
v=xi(1:3);
w=xi(4:6);
if (norm(w)<0.002) && (norm(v)<0.002)   % threshold
    finalerr=norm(v)
    break
end

% Checking singularities
if abs(det(ur5BodyJacobian(q))) < 0.001
    finalerr=-1;
    error('ABORT finalerr=-1');
    break
end

% RR control
q = q-K*t_step*(ur5BodyJacobian(q)\xi);
loop_time = loop_time+1; %update how many times we looped
if(loop_time<4)
    ur5.move_joints(q,2);
    pause(2.02);
else
    ur5.move_joints(q,1);
    pause(1.02);
end

end

