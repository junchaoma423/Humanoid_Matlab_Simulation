% Originally Bipedal_MPC.m by Junheng
% Modified by Han on 20220718 for controller without WBC

% Updated by Han on 20220815 for humanoid robot

%% Main %%
function u = MPC_3Dturning_humanoid(uin)

%test=0; % used for debug
%test=test+1 % check this output to locate break point

%% Parameters
global dt_MPC i_MPC_var dt_MPC_vec gait x_traj_IC x_traj Contact_Jacobian Rotm_foot
%dt_MPC - vector for MPC time steps 
%i_MPC_var - current MPC timestep k
%t=uin(65);
k = i_MPC_var;
h = 10; % prediction horizons
g = 9.81; % gravity 

tic
%% Making definition consistent with MPC
% input definition
xdes=uin(1:13);
x=uin(14:26);
q=uin(27:36);
qd=uin(37:46);
foot=uin(47:58);
hip=uin(59:64);

R = quat2rotm(x(1:4)');
eulzyx=quat2eul(x(1:4)')';
eul=[eulzyx(3);eulzyx(2);eulzyx(1)];
eulzyx_des=quat2eul(xdes(1:4)')';
eul_des=[eulzyx_des(3);eulzyx_des(2);eulzyx_des(1)];

% enforce "2*pi=0" relation
yaw_correction=0;
while yaw_correction==0
    if eul_des(3,1)-eul(3,1)>pi
        eul(3,1)=eul(3,1)+2*pi;
    elseif eul_des(3,1)-eul(3,1)<-pi
        eul(3,1)=eul(3,1)-2*pi;
    else
        yaw_correction=1;
    end
end
xdes = [eul_des;xdes(5:13);g];
x = [eul;x(5:13);g];
R_z=[cos(eul(3,1)), -sin(eul(3,1)),0;
         sin(eul(3,1)), cos(eul(3,1)),0;
         0,0,1];

i_MPC_gait = rem(k,h);

%    R_z_foot_L=[cos(eul(3,1)+q(6,1)),-sin(eul(3,1)+q(6,1)),0;
%            sin(eul(3,1)+q(6,1)), cos(eul(3,1)+q(6,1)),0;
%            0,0,1];

%    R_z_foot_R=[cos(eul(3,1)+q(1,1)),-sin(eul(3,1)+q(1,1)),0;
%            sin(eul(3,1)+q(1,1)), cos(eul(3,1)+q(1,1)),0;
%            0,0,1];




%% Assigning desired trajectory for CoM and Foot locations
if k == 1
    x_traj = x_traj_IC;
end
x_traj = Calc_x_traj(xdes,x,h,k);
% CoM_traj_matrix = x_traj(4:6,:)'
foot_traj = zeros(6,1);
%foot_traj = Calc_foot_traj_3Dwalking(xdes,x_traj,foot,h,k,R)；
foot_traj =[foot(1:3);foot(7:9)];
%% Robot simplified dynamics physical properties/MPC setup
mu = 1.5; %friction coefficient
%m = 5.75 + 2*(0.13+0.65+1.35) + 2*(0.835 + 0.764+1.613+0.12+0.08); % mass
m = 5.75 + 2*(0.835+0.764+1.613+0.12+0.08) + 2*(1+0.835+1.35+0.53)+ 2*(0.05*3+0.01); % mass
Ib = [44350.390401426, -1.912013512, -141.501651425;
    -1.912013512, 53149.535979716, -4.127516029;
    -141.501651425, -4.127516029, 21390.854659820]*1e-6;
Ib = diag([0.064, 0.057, 0.016]);
g=9.81;
Fmax = 1000; Fmin = 0;

%% Forward Kinematics
RR=reshape(R,[9,1]);
Jc=Contact_Jacobian(q(1),q(2),q(3),q(4),q(5),q(6),q(7),q(8),q(9),q(10),RR(1),RR(2),RR(3),RR(4),RR(5),RR(6),RR(7),RR(8),RR(9));
contact_mapping=[blkdiag(Jc(1:3,:)',Jc(4:6,:)'),blkdiag(Jc(7:9,:)',Jc(10:12,:)')]; % torque=contact_mapping*u

%% A & B matrices

R_foot=Rotm_foot(q(1),q(2),q(3),q(4),q(5),q(6),q(7),q(8),q(9),q(10),RR(1),RR(2),RR(3),RR(4),RR(5),RR(6),RR(7),RR(8),RR(9));
R_foot_R=R_foot(1:3,:);
R_foot_L=R_foot(4:6,:);

I = R*Ib*R';
IM = diag([0 1 1]); % my mz selection matrices
IM_R=R_foot_R*IM*R_foot_R'; 
IM_L=R_foot_L*IM*R_foot_L'; % convert selection into "foot frame"




B=repmat({zeros(13,12)},h,1);
A_hat=repmat({zeros(13,13)},h,1);
for i = 1 : h
%Ac=[zeros(3,3), zeros(3,3), eul2rotm([0,0,x_traj(3,i)],'XYZ')', zeros(3,3), zeros(3,1);
Ac=[zeros(3,3), zeros(3,3), [cos(x_traj(3,i))*cos(x_traj(2,i)),-sin(x_traj(3,i)),0;sin(x_traj(3,i))*cos(x_traj(2,i)),cos(x_traj(3,i)),0;-sin(x_traj(2,i)),0,1]\eye(3), zeros(3,3), zeros(3,1);
    zeros(3,3), zeros(3,3), zeros(3,3), eye(3), zeros(3,1);
    zeros(3,3), zeros(3,3), zeros(3,3), zeros(3,3), zeros(3,1);
    zeros(3,3), zeros(3,3), zeros(3,3), zeros(3,3),[0;0;-1];
    zeros(1,13)];

Bc=[zeros(3,3),zeros(3,3),zeros(3,3),zeros(3,3);
    zeros(3,3),zeros(3,3),zeros(3,3),zeros(3,3);
    I\skew(-x_traj(4:6,i) + foot_traj(1:3,1)), I\skew(-x_traj(4:6,i) + foot_traj(4:6,1)), I\eye(3), I\eye(3);
%    I\skew(-x_traj(4:6,i) + foot_traj(1:3,1)), I\skew(-x_traj(4:6,i) + foot_traj(4:6,1)), I\IM_R, I\IM_L;
%    inv(Ib)*skew(R'*(-x_traj(4:6,i) + foot_traj(1:3,1)))*R', inv(Ib)*skew(R'*(-x_traj(4:6,i) + foot_traj(4:6,1)))*R', inv(Ib)*R'*IM_R, inv(Ib)*R'*IM_L;
%    inv(I)*skew(-x_traj(4:6,i) + foot(1:3,1)), inv(I)*skew(-x_traj(4:6,i) + foot(7:9,1)), inv(I)*IM, inv(I)*IM;
    eye(3)/m, eye(3)/m, zeros(3),zeros(3);
    zeros(1,12)];

B{i}=Bc*dt_MPC_vec(i+k-1);
A_hat{i}=eye(13)+Ac*dt_MPC_vec(i+k-1);

end



%% QP_MPC derivation:
y = reshape(x_traj,[13*h 1]);
Aqp=repmat({zeros(13,13)},h,1);
%Aqp
Aqp{1}=A_hat{1};
for i=2:h
    Aqp{i}=Aqp{i-1}*A_hat{i};
end
Aqp=cell2mat(Aqp);
%Bqp
Bqp=repmat({zeros(13,12)},h,h);
for i=1:h
    Bqp{i,i}=B{i};
    for j=1:h-1
        Bqp{i,j}=A_hat{i}^(i-j)*B{j};
    end
end
for i=1:h-1
    for j=i+1:h
        Bqp{i,j}=zeros(13,12);
    end
end
Bqp=cell2mat(Bqp);
% MPC Weights:
% alpha=[1 1 1, 1 1 1, 1 1 1, 1 1 1]*10^(-5);
 L1 = [1500 2000 1000 ,  1000 1000 1000 ,  1 3 10 ,  1 1 1,  0];%eul, p, omega, v, g
%alpha=[.1 .1 .1, .1 .1 .1, .5 .5 .5, .5 .5 .5]*10^(-3); % bipedal
alpha=[.1 .1 .1, .1 .1 .1, .5 .5 .5, .5 .5 .5]*10^(-3); % humanoid

%L1 = diag([50 50 1  100 100 1000  1 1 10  10 10 10  1]);%eul, p, omega, v, g
%alpha=[1 1 1, 1 1 1, .1 .1 .1, .1 .1 .1]*10^(-3);
%L1 = diag([50 50 1  100 100 1000  10 10 10  10 10 10  1]);%eul, p, omega, v, g

L10 = repmat(L1,1,h);
L=diag(L10);
alpha10 = repmat(alpha,1,h);
K = diag(alpha10);
H=2*(Bqp'*L*Bqp+K);


f=2*Bqp'*L*(Aqp*x-y);  


%% MPC constraints:


%max and min forces / friction cone
C1=[1,0,-mu;
    -1,0,-mu;
    0,1,-mu;
    0,-1,-mu];
Ca=[blkdiag(C1,C1),zeros(8,6)];% for friction cone
Ma=zeros(8,1);

Cb=[0,0,-1,zeros(1,9);
    zeros(1,3),0,0,-1,zeros(1,6);]; % to ensure F_z>=fmin;
Mb=-Fmin.*ones(2,1);
Cc=-Cb; % to ensure F_z<=fmax;
Mc=Fmax.*ones(2,1);
S=[Ca;Cb;Cc];
Sk=blkdiag(S,S,S,S,S,S,S,S,S,S); % depends on horizontal length
M=[Ma;Mb;Mc];
Mk=repmat(M,h,1);


%enforce my mz / friction cone of heel and toe:
lt = 0.09-0.01; ln = 0.06-0.02;

SF1=[-ln*[0,0,1]*R_foot_R',zeros(1,3),[0,1,0]*R_foot_R',zeros(1,3);
    -lt*[0,0,1]*R_foot_R',zeros(1,3),-[0,1,0]*R_foot_R',zeros(1,3);
    zeros(1,3),-ln*[0,0,1]*R_foot_L',zeros(1,3),[0,1,0]*R_foot_L';
    zeros(1,3),-lt*[0,0,1]*R_foot_L',zeros(1,3),-[0,1,0]*R_foot_L'];
MF1 = [0;0;0;0]; % ensure foot on ground


SF2 = [ [0,0,-0.5*mu*(ln+lt)],zeros(1,3),[0,-0.5*mu*(lt-ln)/(ln+lt),1]*R_foot_R',zeros(1,3);
    [0,0,-0.5*mu*(ln+lt)],zeros(1,3),[0,-0.5*mu*(lt-ln)/(ln+lt),-1]*R_foot_R',zeros(1,3);
    zeros(1,3),[0,0,-0.5*mu*(ln+lt)],zeros(1,3),[0,-0.5*mu*(lt-ln)/(ln+lt),1]*R_foot_L';
    zeros(1,3),[0,0,-0.5*mu*(ln+lt)],zeros(1,3),[0,-0.5*mu*(lt-ln)/(ln+lt),-1]*R_foot_L'];

MF2 = [0;0;0;0]; % rotation friction condition

SFs = [SF1;SF2]; 
MFs = [MF1;MF2];
SF = blkdiag(SFs,SFs,SFs,SFs,SFs,SFs,SFs,SFs,SFs,SFs);
MF=repmat(MFs,h,1);


% motor torque limit
ST1=contact_mapping;
MT1=[33.5;33.5;67;67;10;33.5;33.5;67;67;10];

STs=[ST1;-ST1];
MTs=[MT1;MT1];

ST=blkdiag(STs,STs,STs,STs,STs,STs,STs,STs,STs,STs);
MT=repmat(MTs,h,1);


% equality constraints
% foot moment Mx=0
Moment_selection=[1,0,0];
Sx1=[zeros(1,3),zeros(1,3),Moment_selection*R_foot_R',zeros(1,3);
    zeros(1,3),zeros(1,3),zeros(1,3),Moment_selection*R_foot_L'];
Mx1=[0;0];
Sx=blkdiag(Sx1,Sx1,Sx1,Sx1,Sx1,Sx1,Sx1,Sx1,Sx1,Sx1);
Mx=repmat(Mx1,h,1);

%gait constraints:
if gait == 0
    gaitm = [];
    beq = [];
elseif gait == 1 
    gaitm = gait_walking(k);
    beq = zeros(60,1);
elseif gait == 2
    gaitm = gait_hopping(k);
    beq = zeros(120,1);
elseif gait == 3
    gaitm = gait_running(k);
    beq = zeros(120,1);
end


%% Solve MPC
GRFM=quadprog(H,f,[Sk;SF;ST],[Mk;MF;MT],[gaitm;Sx],[beq;Mx]);% Walking
%GRFM=quadprog(H,f,[Sk;SF;ST],[Mk;MF;MT],gaitm,beq);

%GRFM=quadprog(H,f,[Sk;SF;ST],[Mk;MF;MT],[Sx],[Mx]); % Standing

%GRFM=quadprog(H,f,[Sk;SF;ST],[Mk;MF;MT]);

%GRFM(1:12);

u=-contact_mapping*GRFM(1:12);

%test1=Aqp*x-y;
%test1(1:12,1)
%test2=Bqp*GRFM;
%test2(1:12,1)
toc


u=[u;GRFM(1:12)];
end


%% functions %%



function foot_traj = Calc_foot_traj_3Dwalking(xdes,x_traj,foot,h,k,R)
global dt_MPC_vec 
i_MPC_gait = rem(k,h);

foot_traj = zeros(6,h);
%foot prediction: walking gait
if 1 <= i_MPC_gait && i_MPC_gait <= 5 % stance sequence: R-L-R
    current_R = foot(1:3); % current right foot - anchor foot
    next_L = current_R + xdes(10)*dt_MPC_vec(k)*h/2;
    next_R = next_L + xdes(10)*dt_MPC_vec(k+5)*h/2;
    %phase 1: right foot anchoring (6-i_MPC_gait)
    for i = 1:6-i_MPC_gait
        foot_traj(4:6,i) = R*current_R;
    end
    %phase 2: next left foot anchoring (5)
    for i = 7-i_MPC_gait:11-i_MPC_gait
        foot_traj(1:3,i) = R*next_L;
    end
    %phase 3: next right foot anchoring (i_MPC-gait-1)
    if i_MPC_gait>1
        for i = 12-i_MPC_gait:h
            foot_traj(4:6,i) = R*next_R;
        end
    end
else % stance sequence: L-R-L
    if i_MPC_gait == 0; i_MPC_gait = 10; end
    i_MPC_gait = i_MPC_gait - h/2;
    current_L = foot(1:3); % current left foot - anchor foot
    next_R = current_L + n*xdes(10)*dt_MPC_vec(k)*h/2;
    next_L = next_R + n*xdes(10)*dt_MPC_vec(k+5)*h/2;
    %phase 1: left foot anchoring (6-i_MPC_gait)
    for i = 1:6-i_MPC_gait
        foot_traj(1:3,i) = R*current_L;
    end
    %phase 2: next right foot anchoring (5)
    for i = 7-i_MPC_gait:11-i_MPC_gait
        foot_traj(4:6,i) = R*next_R;
    end
    %phase 3: next left foot anchoring (i_MPC-gait-1)
    if i_MPC_gait>1
        for i = 12-i_MPC_gait:h
            foot_traj(1:3,i) = R*next_L;
        end
    end
end

end


function x_traj = Calc_x_traj(xdes,x,h,k)
global dt_MPC_vec x_traj
 for i = 0:h-1

    %x_traj(1,i+1) = x(1) + xdes(7)*sum(dt_MPC_vec(k:k+i));
    %x_traj(2,i+1) = x(2) + xdes(8)*sum(dt_MPC_vec(k:k+i));
   % x_traj(3,i+1) = x(3) + xdes(9)*sum(dt_MPC_vec(k:k+i));
  %  x_traj(4,i+1) = x(4) + xdes(10)*sum(dt_MPC_vec(k:k+i));
    if xdes(7) == 0
        x_traj(1,i+1) = xdes(1) + xdes(7)*sum(dt_MPC_vec(k:k+i));
    else
        x_traj(1,i+1) = x(1) + xdes(7)*sum(dt_MPC_vec(k:k+i));
    end
    if xdes(8) == 0
        x_traj(2,i+1) = xdes(2) + xdes(8)*sum(dt_MPC_vec(k:k+i));
    else
        x_traj(2,i+1) = x(2) + xdes(8)*sum(dt_MPC_vec(k:k+i));
    end
    if xdes(9) == 0
        x_traj(3,i+1) = xdes(3) + xdes(9)*sum(dt_MPC_vec(k:k+i));
    else
        x_traj(3,i+1) = x(3) + xdes(9)*sum(dt_MPC_vec(k:k+i));
    end
    if xdes(10) == 0
        x_traj(4,i+1) = xdes(4) + xdes(10)*sum(dt_MPC_vec(k:k+i));
    else
        x_traj(4,i+1) = x(4) + xdes(10)*sum(dt_MPC_vec(k:k+i));
    end
    if xdes(11) == 0
        x_traj(5,i+1) = xdes(5) + xdes(11)*sum(dt_MPC_vec(k:k+i));
    else
        x_traj(5,i+1) = x(5) + xdes(11)*sum(dt_MPC_vec(k:k+i));
    end
    if xdes(12) == 0
        x_traj(6,i+1) = xdes(6) + xdes(12)*sum(dt_MPC_vec(k:k+i));
    else
        x_traj(6,i+1) = x(6) + xdes(12)*sum(dt_MPC_vec(k:k+i));
    end
    x_traj(7,i+1) = xdes(7);
    x_traj(8,i+1) = xdes(8);
    x_traj(9,i+1) = xdes(9);
    x_traj(10,i+1) = xdes(10);
    x_traj(11,i+1) = xdes(11);
    x_traj(12,i+1) = xdes(12);
    x_traj(13,i+1) = xdes(13);
 end
end



function gaitm = gait_walking(i)
global dt_MPC
tspan = 2;
dt = dt_MPC;
N = 10;
L = tspan/dt+N; %total number of horizons
n = L/N; %total number of gait matrices;
i = rem(i,10);
% walking 
Rm = [eye(3),zeros(3),zeros(3),zeros(3);
    zeros(3),zeros(3),eye(3),zeros(3)];
Lm = [zeros(3),eye(3),zeros(3),zeros(3);
    zeros(3),zeros(3),zeros(3),eye(3)];


flight = [eye(3),zeros(3),zeros(3),zeros(3);
    zeros(3),eye(3),zeros(3),zeros(3)];
if i == 1
    IOI{1} = Rm; IOI{2} = Rm; IOI{3} = Rm; IOI{4} = Rm; IOI{5} = Rm; 
    IOI{6} = Lm; IOI{7} = Lm; IOI{8} = Lm; IOI{9} = Lm; IOI{10} = Lm; 
elseif i == 2
    IOI{1} = Rm; IOI{2} = Rm; IOI{3} = Rm; IOI{4} = Rm; IOI{5} = Lm; 
    IOI{6} = Lm; IOI{7} = Lm; IOI{8} = Lm; IOI{9} = Lm; IOI{10} = Rm; 
elseif i == 3
    IOI{1} = Rm; IOI{2} = Rm; IOI{3} = Rm; IOI{4} = Lm; IOI{5} = Lm;
    IOI{6} = Lm; IOI{7} = Lm; IOI{8} = Lm; IOI{9} = Rm; IOI{10} = Rm;
elseif i == 4
    IOI{1} = Rm; IOI{2} = Rm; IOI{3} = Lm; IOI{4} = Lm; IOI{5} = Lm;
    IOI{6} = Lm; IOI{7} = Lm; IOI{8} = Rm; IOI{9} = Rm; IOI{10} = Rm;  
elseif i == 5
    IOI{1} = Rm; IOI{2} = Lm; IOI{3} = Lm; IOI{4} = Lm; IOI{5} = Lm;
    IOI{6} = Lm; IOI{7} = Rm; IOI{8} = Rm; IOI{9} = Rm; IOI{10} = Rm;
elseif i == 6
    IOI{1} = Lm; IOI{2} = Lm; IOI{3} = Lm; IOI{4} = Lm; IOI{5} = Lm;
    IOI{6} = Rm; IOI{7} = Rm; IOI{8} = Rm; IOI{9} = Rm; IOI{10} = Rm;
elseif i == 7
    IOI{1} = Lm; IOI{2} = Lm; IOI{3} = Lm; IOI{4} = Lm; IOI{5} = Rm;
    IOI{6} = Rm; IOI{7} = Rm; IOI{8} = Rm; IOI{9} = Rm; IOI{10} = Lm;
elseif i == 8
    IOI{1} = Lm; IOI{2}= Lm; IOI{3} = Lm; IOI{4} = Rm; IOI{5} = Rm;
    IOI{6} = Rm; IOI{7} = Rm; IOI{8} = Rm; IOI{9} = Lm; IOI{10} = Lm;
elseif i == 9
    IOI{1} = Lm; IOI{2} = Lm; IOI{3} = Rm; IOI{4} = Rm; IOI{5} = Rm;
    IOI{6} = Rm; IOI{7} = Rm; IOI{8} = Lm; IOI{9} = Lm; IOI{10} = Lm;
elseif i == 0
    IOI{1} = Lm; IOI{2} = Rm; IOI{3} = Rm; IOI{4} = Rm; IOI{5} = Rm;
    IOI{6} = Rm; IOI{7} = Lm; IOI{8} = Lm; IOI{9} = Lm; IOI{10} = Lm;
end
gaitm = ...%gait matrix for 10 horizons;
    [ cell2mat(IOI(1)),zeros(6,12*9);
    zeros(6,12*1), cell2mat(IOI(2)),zeros(6,12*8);
    zeros(6,12*2), cell2mat(IOI(3)),zeros(6,12*7);
    zeros(6,12*3), cell2mat(IOI(4)),zeros(6,12*6);
    zeros(6,12*4), cell2mat(IOI(5)),zeros(6,12*5);
    zeros(6,12*5), cell2mat(IOI(6)),zeros(6,12*4);
    zeros(6,12*6), cell2mat(IOI(7)),zeros(6,12*3);
    zeros(6,12*7), cell2mat(IOI(8)),zeros(6,12*2);
    zeros(6,12*8), cell2mat(IOI(9)),zeros(6,12*1);
    zeros(6,12*9), cell2mat(IOI(10)) ];
end

function gaitm = gait_hopping(i)
global dt_MPC 
tspan = 2;
dt = dt_MPC;
N = 10;
L = tspan/dt+N; %total number of horizons
n = L/N; %total number of gait matrices;
i = rem(i,10);
% walking 
Lm = [eye(3),zeros(3),zeros(3),zeros(3);
    zeros(3),eye(3),zeros(3),zeros(3);
    zeros(3),zeros(3),eye(3),zeros(3);
    zeros(3),zeros(3),zeros(3),eye(3)];
Rm = [zeros(3),zeros(3),zeros(3),zeros(3);
    zeros(3),zeros(3),zeros(3),zeros(3);
    zeros(3),zeros(3),zeros(3),zeros(3);
    zeros(3),zeros(3),zeros(3),zeros(3)];


if i == 1
    IOI{1} = Rm; IOI{2} = Rm; IOI{3} = Rm; IOI{4} = Rm; IOI{5} = Rm; 
    IOI{6} = Lm; IOI{7} = Lm; IOI{8} = Lm; IOI{9} = Lm; IOI{10} = Lm; 
elseif i == 2
    IOI{1} = Rm; IOI{2} = Rm; IOI{3} = Rm; IOI{4} = Rm; IOI{5} = Lm; 
    IOI{6} = Lm; IOI{7} = Lm; IOI{8} = Lm; IOI{9} = Lm; IOI{10} = Rm; 
elseif i == 3
    IOI{1} = Rm; IOI{2} = Rm; IOI{3} = Rm; IOI{4} = Lm; IOI{5} = Lm;
    IOI{6} = Lm; IOI{7} = Lm; IOI{8} = Lm; IOI{9} = Rm; IOI{10} = Rm;
elseif i == 4
    IOI{1} = Rm; IOI{2} = Rm; IOI{3} = Lm; IOI{4} = Lm; IOI{5} = Lm;
    IOI{6} = Lm; IOI{7} = Lm; IOI{8} = Rm; IOI{9} = Rm; IOI{10} = Rm;  
elseif i == 5
    IOI{1} = Rm; IOI{2} = Lm; IOI{3} = Lm; IOI{4} = Lm; IOI{5} = Lm;
    IOI{6} = Lm; IOI{7} = Rm; IOI{8} = Rm; IOI{9} = Rm; IOI{10} = Rm;
elseif i == 6
    IOI{1} = Lm; IOI{2} = Lm; IOI{3} = Lm; IOI{4} = Lm; IOI{5} = Lm;
    IOI{6} = Rm; IOI{7} = Rm; IOI{8} = Rm; IOI{9} = Rm; IOI{10} = Rm;
elseif i == 7
    IOI{1} = Lm; IOI{2} = Lm; IOI{3} = Lm; IOI{4} = Lm; IOI{5} = Rm;
    IOI{6} = Rm; IOI{7} = Rm; IOI{8} = Rm; IOI{9} = Rm; IOI{10} = Lm;
elseif i == 8
    IOI{1} = Lm; IOI{2}= Lm; IOI{3} = Lm; IOI{4} = Rm; IOI{5} = Rm;
    IOI{6} = Rm; IOI{7} = Rm; IOI{8} = Rm; IOI{9} = Lm; IOI{10} = Lm;
elseif i == 9
    IOI{1} = Lm; IOI{2} = Lm; IOI{3} = Rm; IOI{4} = Rm; IOI{5} = Rm;
    IOI{6} = Rm; IOI{7} = Rm; IOI{8} = Lm; IOI{9} = Lm; IOI{10} = Lm;
elseif i == 0
    IOI{1} = Lm; IOI{2} = Rm; IOI{3} = Rm; IOI{4} = Rm; IOI{5} = Rm;
    IOI{6} = Rm; IOI{7} = Lm; IOI{8} = Lm; IOI{9} = Lm; IOI{10} = Lm;
end
gaitm = ...%gait matrix for 10 horizons;
    [ cell2mat(IOI(1)),zeros(12,12*9);
    zeros(12,12*1), cell2mat(IOI(2)),zeros(12,12*8);
    zeros(12,12*2), cell2mat(IOI(3)),zeros(12,12*7);
    zeros(12,12*3), cell2mat(IOI(4)),zeros(12,12*6);
    zeros(12,12*4), cell2mat(IOI(5)),zeros(12,12*5);
    zeros(12,12*5), cell2mat(IOI(6)),zeros(12,12*4);
    zeros(12,12*6), cell2mat(IOI(7)),zeros(12,12*3);
    zeros(12,12*7), cell2mat(IOI(8)),zeros(12,12*2);
    zeros(12,12*8), cell2mat(IOI(9)),zeros(12,12*1);
    zeros(12,12*9), cell2mat(IOI(10)) ];
end

function gaitm = gait_running(i)
global dt_MPC
tspan = 2;
dt = dt_MPC;
N = 10;
L = tspan/dt+N; %total number of horizons
n = L/N; %total number of gait matrices;
i = rem(i,10);
% walking 
Rm = [eye(3),zeros(3),zeros(3),zeros(3);
    zeros(3),zeros(3),zeros(3),zeros(3);
    zeros(3),zeros(3),eye(3),zeros(3);
    zeros(3),zeros(3),zeros(3),zeros(3);];
Lm = [zeros(3),eye(3),zeros(3),zeros(3);
    zeros(3),zeros(3),zeros(3),zeros(3);
    zeros(3),zeros(3),zeros(3),eye(3);
    zeros(3),zeros(3),zeros(3),zeros(3);];
flight = [eye(3),zeros(3),zeros(3),zeros(3);
    zeros(3),eye(3),zeros(3),zeros(3);
    zeros(3),zeros(3),eye(3),zeros(3);
    zeros(3),zeros(3),zeros(3),eye(3)];
if i == 1
    IOI{1} = Rm; IOI{2} = Rm; IOI{3} = Rm; IOI{4} = Rm; IOI{5} = flight; 
    IOI{6} = Lm; IOI{7} = Lm; IOI{8} = Lm; IOI{9} = Lm; IOI{10} = flight; 
elseif i == 2
    IOI{1} = Rm; IOI{2} = Rm; IOI{3} = Rm; IOI{4} = flight; IOI{5} = Lm; 
    IOI{6} = Lm; IOI{7} = Lm; IOI{8} = Lm; IOI{9} = flight; IOI{10} = Rm; 
elseif i == 3
    IOI{1} = Rm; IOI{2} = Rm; IOI{3} = flight; IOI{4} = Lm; IOI{5} = Lm;
    IOI{6} = Lm; IOI{7} = Lm; IOI{8} = flight; IOI{9} = Rm; IOI{10} = Rm;
elseif i == 4
    IOI{1} = Rm; IOI{2} = flight; IOI{3} = Lm; IOI{4} = Lm; IOI{5} = Lm;
    IOI{6} = Lm; IOI{7} = flight; IOI{8} = Rm; IOI{9} = Rm; IOI{10} = Rm;  
elseif i == 5
    IOI{1} = flight; IOI{2} = Lm; IOI{3} = Lm; IOI{4} = Lm; IOI{5} = Lm;
    IOI{6} = flight; IOI{7} = Rm; IOI{8} = Rm; IOI{9} = Rm; IOI{10} = Rm;
elseif i == 6
    IOI{1} = Lm; IOI{2} = Lm; IOI{3} = Lm; IOI{4} = Lm; IOI{5} = flight;
    IOI{6} = Rm; IOI{7} = Rm; IOI{8} = Rm; IOI{9} = Rm; IOI{10} = flight;
elseif i == 7
    IOI{1} = Lm; IOI{2} = Lm; IOI{3} = Lm; IOI{4} = flight; IOI{5} = Rm;
    IOI{6} = Rm; IOI{7} = Rm; IOI{8} = Rm; IOI{9} = flight; IOI{10} = Lm;
elseif i == 8
    IOI{1} = Lm; IOI{2}= Lm; IOI{3} = flight; IOI{4} = Rm; IOI{5} = Rm;
    IOI{6} = Rm; IOI{7} = Rm; IOI{8} = flight; IOI{9} = Lm; IOI{10} = Lm;
elseif i == 9
    IOI{1} = Lm; IOI{2} = flight; IOI{3} = Rm; IOI{4} = Rm; IOI{5} = Rm;
    IOI{6} = Rm; IOI{7} = flight; IOI{8} = Lm; IOI{9} = Lm; IOI{10} = Lm;
elseif i == 0
    IOI{1} = flight; IOI{2} = Rm; IOI{3} = Rm; IOI{4} = Rm; IOI{5} = Rm;
    IOI{6} = flight; IOI{7} = Lm; IOI{8} = Lm; IOI{9} = Lm; IOI{10} = Lm;
end
gaitm = ...%gait matrix for 10 horizons;
    [ cell2mat(IOI(1)),zeros(12,12*9);
    zeros(12,12*1), cell2mat(IOI(2)),zeros(12,12*8);
    zeros(12,12*2), cell2mat(IOI(3)),zeros(12,12*7);
    zeros(12,12*3), cell2mat(IOI(4)),zeros(12,12*6);
    zeros(12,12*4), cell2mat(IOI(5)),zeros(12,12*5);
    zeros(12,12*5), cell2mat(IOI(6)),zeros(12,12*4);
    zeros(12,12*6), cell2mat(IOI(7)),zeros(12,12*3);
    zeros(12,12*7), cell2mat(IOI(8)),zeros(12,12*2);
    zeros(12,12*8), cell2mat(IOI(9)),zeros(12,12*1);
    zeros(12,12*9), cell2mat(IOI(10)) ];
end


function A= skew(v)
 A=[0 -v(3) v(2) ; v(3) 0 -v(1) ; -v(2) v(1) 0 ]; 
end