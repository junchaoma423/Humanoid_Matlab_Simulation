function tau=PD_arms(uin)
q=uin(1:6);
dq=uin(7:12);
q_d=[0;0;deg2rad(30);deg2rad(30);deg2rad(-90);deg2rad(-90)];
dq_d=zeros(6,1);
Kp=100*ones(6,1);
Kd=1*ones(6,1);
tau=-(Kp.*(q-q_d)+Kd.*(dq-dq_d));



end