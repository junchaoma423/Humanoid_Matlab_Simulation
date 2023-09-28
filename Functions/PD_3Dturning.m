function tau=PD_3Dturning(uin)

%test=0; % used for debug
%test=test+1 % check this output to locate break point

global t_start dt_MPC optimization FootPosition gait i_gait acc_t i_MPC_var dt_MPC_vec stand_position current_height
t=uin(1);
xdes=uin(2:14);
x=uin(15:27);
q=uin(28:37);
qd=uin(38:47); 
fpfv=uin(48:59);



hip=uin(60:65);

Q=x(1:4);
x_act=x(5:7);
w_act=x(8:10);
v_act=x(11:13);


% Feedback term coefficient 
% (rough approximation, actual COM is lower than xdes(7))
K_step=1*(0.1*xdes(7))^0.5;

dt = dt_MPC;
gaitcycle = dt_MPC*10;
vx_des = xdes(11);
vy_des = xdes(12);
wz_des=xdes(10);
vx_act = v_act(1);
vy_act = v_act(2);
vz_act = v_act(3);
wz_act=w_act(3);
%lift height of each foot:
lift_height = 0.12;
%current foot states
fpR = fpfv(1:3);
fpL = fpfv(7:9);
fvR = fpfv(4:6);
fvL = fpfv(10:12);


if t > t_start %gait start
    %desired trajectory:
%     t_cycle = rem(t-acc_t(i_MPC_var), gaitcycle/2);
%     i_cycle = floor((t-t_start)/(gaitcycle/2))+1;
%     i_gait = rem(i_cycle,2);
    stage = floor((i_MPC_var-1)/5);
    ii = (stage)*5+1;
    t_cycle = t-acc_t(ii);
    swing_schedule = zeros(2,1);
    ki = rem(i_MPC_var,20);
if ki == 0; ki = 20; end
if gait == 1
    if 1 <= ki && ki <= 5
        swing_schedule(1) = 1;
        swing_schedule(2) = 0;
    elseif 6 <= ki && ki <= 10
        swing_schedule(1) = 0;
        swing_schedule(2) = 1;
    elseif 11 <= ki && ki <= 15
        swing_schedule(1) = 1;
        swing_schedule(2) = 0;
    elseif 16 <= ki && ki <= 20
        swing_schedule(1) = 0;
        swing_schedule(2) = 1;
    end
elseif gait == 4
    if 6 <= ki && ki <= 10
        swing_schedule(1) = 0;
        swing_schedule(2) = 1;
    elseif 16 <= ki && ki <= 20
        swing_schedule(1) = 1;
        swing_schedule(2) = 0;
    else
        swing_schedule(1) = 0;
        swing_schedule(2) = 0;
    end
elseif gait == 0
    swing_schedule(1) = 0;
    swing_schedule(2) = 0;
end

   
    %transition between two gaits:
    delta_t = gaitcycle/2;
    if i_MPC_var>5 %delta_t2 is needed to have even transition between different gaits
        delta_t2 = dt_MPC_vec(i_MPC_var-5)*5;
        trans_quot = dt_MPC_vec(i_MPC_var-5)^1/dt_MPC_vec(i_MPC_var)^1;
        dt2 = dt_MPC_vec(i_MPC_var-5);
    else
        delta_t2 =delta_t;
        trans_quot = 1;
        dt2 = dt;
    end
    %dt_MPC
    trans_quot = 1;
    %delta_t2
    %delta_t
    %desired foot height:
    %quadratic :
    tolerance=0.000;% avoid impact, seems not so necessary
    fzdes = tolerance+0.04-(lift_height/(gaitcycle/4)^2)*t_cycle*(t_cycle-gaitcycle/2);
    dfzdes = -2*(lift_height/(gaitcycle/4)^2)*t_cycle+(lift_height/(gaitcycle/4)^2)*gaitcycle/2; %0.3 

    fzend=tolerance+0.04;
    dfzend=0;
    %     %linear:
%     t_up = 0.03;
%     t_down = 0.05;
%     if 0<=t_cycle && t_cycle<t_up
%         fzdes = (lift_height)/t_up*t_cycle;
%         dfzdes = (lift_height)/t_up;
%     elseif gaitcycle/2-t_up<=t_cycle && t_cycle<gaitcycle/2
%         fzdes = -(lift_height)/t_down*(t_cycle-gaitcycle/2);
%         dfzdes = -(lift_height)/t_down;
%     else
%         fzdes = lift_height;
%         dfzdes = 0;
%     end
%     t_cycle
%     fzdes

%     R = eul2rotm(x_act(1:3)','XYZ');
    %foot placement policy:
    Rotm = quat2rotm(Q');
    eul = quat2eul(Q')';
%     Rotm = eul2rotm([eul(3)/2 eul(2) eul(1)]);
    eul=[0,0,1;0,1,0;1,0,0]*eul;
    width=0.001; % add up to stand width, wider stance helps with stability
    x_com=-0.01; % actual COM is a little behind x_act(5)
    p_hip_R_w = (hip(1:3) + Rotm*[x_com;-width;0]);
    p_hip_L_w = (hip(4:6) + Rotm*[x_com;width;0]);
    r=0.047+width;
    %p_shoulder+p_symmetry ([mini cheetah paper])
%    fx_des_L = p_hip_L_w(1)+trans_quot*(delta_t+delta_t2)/2*(vx_act-wz_act*r*cos(eul(3)))/2+K_step*((vx_act-wz_act*r*cos(eul(3)))-(vx_des-wz_des*r*cos(eul(3))));
%    fy_des_L = p_hip_L_w(2)+trans_quot*(delta_t+delta_t2)/2*(vy_act-wz_act*r*sin(eul(3)))/2+K_step*((vy_act-wz_act*r*sin(eul(3)))-(vy_des-wz_des*r*sin(eul(3))));
%    fx_des_R = p_hip_R_w(1)+trans_quot*(delta_t+delta_t2)/2*(vx_act+wz_act*r*cos(eul(3)))/2+K_step*((vx_act+wz_act*r*cos(eul(3)))-(vx_des+wz_des*r*cos(eul(3))));
%    fy_des_R = p_hip_R_w(2)+trans_quot*(delta_t+delta_t2)/2*(vy_act+wz_act*r*sin(eul(3)))/2+K_step*((vy_act+wz_act*r*sin(eul(3)))-(vy_des+wz_des*r*sin(eul(3))));
    if t_cycle<=delta_t
        t_halfcycle=t_cycle;
    else
        t_halfcycle=t_cycle-delta_t;
    end

    if (t_halfcycle-delta_t<1e-4&&t_halfcycle-delta_t>-1e-4)||(t_halfcycle<1e-4&&t_halfcycle>-1e-4)
        swing_schedule = zeros(2,1);
    end



    if t<=0.002
     stand_position=[0;-0.1;0;0;0.1;0];
     current_height=x_act(3);
    end

    if fpR(3)<=0.002&&swing_schedule(1)==0
     stand_position(1:3,1)=fpR;
    end
    if fpL(3)<=0.002&&swing_schedule(2)==0
     stand_position(4:6,1)=fpL;
    end

    if abs(t_halfcycle-delta_t)<2e-3
    current_height=x_act(3);
    end


    fx_end_R = p_hip_R_w(1)+trans_quot*(delta_t+delta_t2)/2*(vx_act+wz_act*r*cos(eul(3)))/2+K_step*(vx_act-vx_des);
    fy_end_R = p_hip_R_w(2)+trans_quot*(delta_t+delta_t2)/2*(vy_act+wz_act*r*sin(eul(3)))/2+K_step*(vy_act-vy_des);
    fx_end_L = p_hip_L_w(1)+trans_quot*(delta_t+delta_t2)/2*(vx_act-wz_act*r*cos(eul(3)))/2+K_step*(vx_act-vx_des);
    fy_end_L = p_hip_L_w(2)+trans_quot*(delta_t+delta_t2)/2*(vy_act-wz_act*r*sin(eul(3)))/2+K_step*(vy_act-vy_des);

    


    fx_des_R=(t_halfcycle/delta_t)*(fx_end_R-stand_position(1,1))+stand_position(1,1);
    fy_des_R=(t_halfcycle/delta_t)*(fy_end_R-stand_position(2,1))+stand_position(2,1);
    fx_des_L=(t_halfcycle/delta_t)*(fx_end_L-stand_position(4,1))+stand_position(4,1);
    fy_des_L=(t_halfcycle/delta_t)*(fy_end_L-stand_position(5,1))+stand_position(5,1);

%fx_des_R = p_hip_R_w(1);
%    fy_des_R = p_hip_R_w(2);
 %   fx_des_L = p_hip_L_w(1);
 %   fy_des_L = p_hip_L_w(2);

    if swing_schedule(1)==0
        fx_des_R=fpR(1,1);
        fy_des_R=fpR(2,1);
    end
    if swing_schedule(2)==0
        fx_des_L=fpL(1,1);
        fy_des_L=fpL(2,1);
    end
    
    %PD law:


    
%     Kp = diag([300,300,200]); Kd = diag([5,5,12]);
%    Kp = diag([170,170,150]); Kd = diag([3,3,10]);
%     Kp = diag([300,0,200]); Kd = diag([5,0,12]);
%     Kp = diag([100,100,60]); Kd = diag([5,5,7]); %stable
%     Kp = diag([150,150,100]); Kd = diag([6,6,10]);
    Kp = diag([100,100,60]); Kd = diag([5,5,7]);
    %foot location from Optimization:
    fpL_opt = []; fpR_opt = [];
    precentage_opt = diag([1 0 0]); % how much precentage optimal foot placement should be used
    k = floor(t/0.001)+1;
    s = size(FootPosition);
    if optimization == 1 && k <= s(2)
        fpL_opt = [1; 1 ; 1].*FootPosition(1:3,k);
        fpR_opt = [1; 1 ; 1].*FootPosition(4:6,k);
        fpL_des = (eye(3)-precentage_opt)*[fx_des_L;fy_des_L;fzdes] + precentage_opt*fpL_opt;
        fpR_des = (eye(3)-precentage_opt)*[fx_des_R;fy_des_R;fzdes] + precentage_opt*fpR_opt;
    else
        fpL_des = [fx_des_L;fy_des_L;fzdes];%-x_act;
        fpR_des = [fx_des_R;fy_des_R;fzdes];%-x_act;
    end
    
   % fx_end_L = p_hip_L_w(1)+(trans_quot*(delta_t+delta_t2)/2*(vx_act-wz_act*r*cos(eul(3)))/2+K_step*(vx_act-vx_des));
  %  fy_end_L = p_hip_L_w(2)+(trans_quot*(delta_t+delta_t2)/2*(vy_act-wz_act*r*sin(eul(3)))/2+K_step*(vy_act-vy_des));
 %   fx_end_R = p_hip_R_w(1)+(trans_quot*(delta_t+delta_t2)/2*(vx_act+wz_act*r*cos(eul(3)))/2+K_step*(vx_act-vx_des));
  %  fy_end_R = p_hip_R_w(2)+(trans_quot*(delta_t+delta_t2)/2*(vy_act+wz_act*r*sin(eul(3)))/2+K_step*(vy_act-vy_des));

    fpL_end = [fx_end_L;fy_end_L;fzend];
    fpR_end = [fx_end_R;fy_end_R;fzend];
    

    %% IK and Joint PD (a bit more complex but can manipulate each joint)
    %find desired joint angle by IK:
   %{
     qL_des = IK_bipedal_symbolic((fpL_des-x_act(1:3)),1);
     qR_des = IK_bipedal_symbolic((fpR_des-x_act(1:3)),-1);
     Kp = diag([30 30 30 30 30]); Kd = diag([1 1 1 1 1]);
     tau1_IK = (Kp*(qL_des-[0;0;eul(2);0;0] - q(1:5)) + Kd*(0 - qd(1:5))).*[1;swing_schedule(1);swing_schedule(1);swing_schedule(1);1];
     tau2_IK = (Kp*(qR_des-[0;0;eul(2);0;0] - q(6:10)) + Kd*(0 - qd(6:10))).*[1;swing_schedule(2);swing_schedule(2);swing_schedule(2);1];
     tau = [tau1_IK;tau2_IK];
%}

    %% catersian PD (intuitive but cannnot control foot rotation)
    %{
    F_L = Kp*( fpL_des - fpL)+ Kd*([0;0;dfzdes] - fvL);
    F_L = Rotm'*F_L*swing_schedule(1);
    F_R = Kp*( fpR_des - fpR)+ Kd*([0;0;dfzdes] - fvR);
    F_R = Rotm'*F_R*swing_schedule(2);
%     
%     F_L = Kp*( fpL_des - fpL)+ Kd*([0;0;0] - fvL);
%     F_L = Rotm'*F_L*swing_schedule(1);
%     F_R = Kp*( fpR_des - fpR)+ Kd*([0;0;0] - fvR);
%     F_R = Rotm'*F_R*swing_schedule(2);
    
    % leg 1;
    sidesign = 1;
    q1 = q(1);
    q2 = q(2);
    q3 = q(3);
    q4 = q(4);
    q5 = q(5);
    J1 = ComputeJacobian3d_bipedal(q1,q2,q3,q4,q5,sidesign);
    tau1 = J1'*F_L;
    % leg 2 ;
    sidesign = -1;
    q1 = q(6);
    q2 = q(7);
    q3 = q(8);
    q4 = q(9);
    q5 = q(10);
    J2 = ComputeJacobian3d_bipedal(q1,q2,q3,q4,q5,sidesign);
    tau2 = J2'*F_R;

    % control hip1 and ankle rotation:
    qL_des = IK_bipedal_symbolic((fpL_des-x_act(1:3)),1);
    qR_des = IK_bipedal_symbolic((fpR_des-x_act(1:3)),-1);
%     Kp_stance = diag([0 0 0 0 0]); Kd_stance = diag([2 0 0 0 2]);
%     Kp_swing = diag([10 0 0 0 10]); Kd_swing = diag([1 0 0 0 .1]);
%    Kp_stance = diag([10 0 0 0 0]); Kd_stance = diag([2 0 0 0 2]);
%    Kp_swing = diag([10 0 0 0 10]); Kd_swing = diag([1 0 0 0 .4]);
        Kp_stance = diag([10 0 0 0 0]); Kd_stance = diag([2 0 0 0 2]);
        Kp_swing = diag([10 0 0 0 10]); Kd_swing = diag([1 0 0 0 .4]);
        coefficient=mod(i_MPC_var,5);
        if coefficient==0
            Kp_swing=Kp_stance;
        end
    tau1_add = ( Kp_swing*swing_schedule(1)+Kp_stance*swing_schedule(2) )*(qL_des - q(1:5))...
        + ( Kd_swing*swing_schedule(1)+Kd_stance*swing_schedule(2) )*(0 - qd(1:5));
    tau2_add = ( Kp_swing*swing_schedule(2)+Kp_stance*swing_schedule(1) )*(qR_des - q(6:10))...
        + ( Kd_swing*swing_schedule(2)+Kd_stance*swing_schedule(1) )*(0 - qd(6:10));
%    one = 1;
%     tau = [0;tau1(2:5);0;tau2(2:5)];
%     tau = [tau1;tau2];
    tau =  [tau1;tau2]+[tau1_add;tau2_add];
    %}
%% Joint PD, updated June 2022 by Han Gong
 
 % function foot_to_joint() mapping the desired foot position to joint angle
 % hip_yaw fixed to 0
 % hip_roll, hip_pitch, knee calculated with simple geometry 
 % ankle calculated to keep the foot horizontal

 % sidesign 1 and swing_schedule(1) stand for right leg
 % sidesign -1 and swing_schedule(2) stand for left leg
 feedback_ratio=t_halfcycle/delta_t;
 height_feedback=0;
 feedback_limit=0.03;
 %x_act(3)=current_height+feedback_ratio*min(height_feedback*(xdes(7)-current_height),feedback_limit);
x_act(3)=current_height+min(height_feedback*(xdes(7)-current_height),feedback_limit);
     qR_des = foot_to_joint(fpR_des-x_act(1:3),q(1:5,1),Rotm,eul,feedback_ratio,1);
     qL_des = foot_to_joint(fpL_des-x_act(1:3),q(6:10,1),Rotm,eul,feedback_ratio,-1);
     qR_end = foot_to_joint(fpR_end-x_act(1:3),q(1:5,1),Rotm,eul,feedback_ratio,1);
     qL_end = foot_to_joint(fpL_end-x_act(1:3),q(6:10,1),Rotm,eul,feedback_ratio,-1);
     %qR_des =[qR_des(1:4,1);qR_end(5,1)];
     %qL_des =[qL_des(1:4,1);qL_end(5,1)];
     Kp = diag([500 500 500 500 100]); Kd = diag([10 10 5 5 1]);
     Kp = diag([500 500 500 500 100]); Kd = diag([10 10 5 5 1]);
     if fpR(3)<=fzend
         If_offground_R=0;
     else
         If_offground_R=1;
     end
     if fpL(3)<=fzend
         If_offground_L=0;
     else
         If_offground_L=1;
     end

     %swing_schedule=zeros(2,1);
     %tau1 = (Kp*(qR_des - q(1:5)) + Kd*(0 - qd(1:5))).*[swing_schedule(1);1;swing_schedule(1);swing_schedule(1);swing_schedule(1)];
     %tau2 = (Kp*(qL_des - q(6:10)) + Kd*(0 - qd(6:10))).*[swing_schedule(2);1;swing_schedule(2);swing_schedule(2);swing_schedule(2)];
     tau1 = (Kp*(qR_des - q(1:5)) + Kd*(0 - qd(1:5))).*swing_schedule(1);
     tau2 = (Kp*(qL_des - q(6:10)) + Kd*(0 - qd(6:10))).*swing_schedule(2);
     %tau1 = (Kp*(qR_des - q(1:5)) + Kd*(0 - qd(1:5))).*[1;swing_schedule(1);swing_schedule(1);swing_schedule(1);swing_schedule(1)];
     %tau2 = (Kp*(qL_des - q(6:10)) + Kd*(0 - qd(6:10))).*[1;swing_schedule(2);swing_schedule(2);swing_schedule(2);swing_schedule(2)];
     
     tau = [tau1;tau2];
 
     test=[fpR_des;fpL_des];
     tau=[tau;test];

%tau = zeros(16,1);

%% no cartesian control    
else
    tau = zeros(16,1);
end
end
