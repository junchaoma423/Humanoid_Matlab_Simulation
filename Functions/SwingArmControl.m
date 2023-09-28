function q_arms_des = SwingArmControl(vx,q_legs,t)
global armslocked arms_IC pickup throw dropoff
    thigh_L_IC = pi/4;
    thigh_R_IC = pi/4;
    calf_L_IC = -pi/2;
    calf_R_IC = -pi/2;

    d_syl = pi/6+q_legs(8) - thigh_R_IC;
    d_syr = pi/6+q_legs(3) - thigh_L_IC;

   q_elbow_des = -(60+vx*40)*(pi/180);
   
    
   if armslocked == 0
       q_arms_des = [-q_legs(7)-pi/10;-q_legs(2)+pi/10; d_syl;d_syr; q_elbow_des;q_elbow_des];
   else
       %without arm swing:
       q_arms_des = arms_IC ;
       if pickup == 1
           q_arms_des = arms_IC + [-0.5; 0.5; zeros(4,1)];
           q_orig = arms_IC + [-0.34; 0.34; zeros(4,1)]; %orig
%            q_orig = arms_IC + [-0.65; 1.3;  zeros(4,1)]; %faster 90 turn
           q_extend = [arms_IC(1:2); -pi/8;-pi/8;-pi/3.5;-pi/3.5 ];
%            q_extend = [arms_IC(1:2)+[0;0.5]; -pi/6;pi/6;-pi/10;-pi/2.5 ];
%            t_extend = 1.8;
%            t_extend_end = 2.3;
%            t_retract = 2.8;
           t_extend = 2;
           t_extend_end = 2.5;
           t_retract = 3;
           if t_extend <= t && t < t_extend_end
               q_arms_des = q_orig + (t-t_extend)*2*(q_extend - q_orig);
           elseif t_extend_end <= t && t < t_retract
               q_arms_des = q_extend + (t -t_extend_end)*2*(arms_IC - q_extend);
           elseif t < t_extend
               q_arms_des = arms_IC + [-0.17; 0.17; 0.2;0.2; 0;0];
           else
               q_arms_des = arms_IC;
           end

       end

       if dropoff == 1
           q_arms_des = arms_IC;
           q_orig = arms_IC;
           q_extend = [arms_IC(1:2); -pi/4;-pi/4;-pi/4;-pi/4 ];
%            t_extend = 2;
%            t_extend_end = 2.5;
%            t_retract = 3;
%             t_extend = 10;
%             t_extend_end = 10.5;
%             t_retract = 11;
%             t_extend = 3.3;
%             t_extend_end = 3.8;
%             t_retract = 4.3;
            t_extend = 5;
            t_extend_end = 5.5;
            t_retract = 6;
           if t_extend <= t && t < t_extend_end
               q_arms_des = q_orig + (t-t_extend)*2*(q_extend - q_orig);
           elseif t_extend_end <= t && t < t_retract
               q_arms_des = q_extend+ [-0.5; 0.5; zeros(4,1)] + (t -t_extend_end)*2*(arms_IC+ [-0.5; 0.5; zeros(4,1)] - q_extend);
           elseif t < t_extend
               q_arms_des = arms_IC;
           else
               q_arms_des = arms_IC + [-0.5; 0.5; zeros(4,1)];
           end

       end

       if throw == 1
           q_orig = arms_IC;
           throw_init_t = 7.5;
           throw_end_t = 7.61;
           q_throw = [0.15; -0.15; -pi/2;-pi/2; -pi/20;-pi/20];
           if throw_init_t <= t && t < throw_end_t
               q_arms_des = q_orig + (t - throw_init_t)*(1/(throw_end_t-throw_init_t))*(q_throw - q_orig);
           elseif t >= throw_end_t
%                 q_arms_des = q_throw;
                q_arms_des = [0; -0; -pi/2;-pi/2; -pi/20;-pi/20];
           end
       end

   end
%     q_arms_des = [-q_legs(7)-pi/10;-q_legs(2)+pi/10; 1*q_legs(8)-pi/6; 1*q_legs(3)-pi/6; -q_legs(9)-pi; -q_legs(4)-pi;];
%     q_arms_des = [-q_legs(2)-pi/10;-q_legs(7)+pi/10; 1*q_legs(3)-pi/6; 1*q_legs(8)-pi/6; -q_legs(4)-pi; -q_legs(9)-pi;];
%     q_arms_des = [-q_legs(7)-pi/10;-q_legs(2)+pi/10; 1*q_legs(8)-pi/6; 1*q_legs(3)-pi/6; -pi/2; -pi/2;];
end