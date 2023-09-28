L = length(out.tau_MPC(1,1,:));
t = linspace(0,out.time(end),L);
tau1 = zeros(L,1);
tau2 = zeros(L,1);
tau3 = zeros(L,1);
tau4 = zeros(L,1);
tau5 = zeros(L,1);
tau6 = zeros(L,1);
tau7 = zeros(L,1);
tau8 = zeros(L,1);
tau9 = zeros(L,1);
tau10 = zeros(L,1);

for i = 1:L
%     t(i) = out.tout(i);
    tau1(i) = out.tau_MPC(1,1,i);
    tau2(i) = out.tau_MPC(2,1,i);
    tau3(i) = out.tau_MPC(3,1,i);
    tau4(i) = out.tau_MPC(4,1,i);
    tau5(i) = out.tau_MPC(5,1,i);
    tau6(i) = out.tau_MPC(6,1,i);
    tau7(i) = out.tau_MPC(7,1,i);
%      if tau7(i) > 33.5
%         tau7(i) = 33.5;
%     elseif tau7(i) < -33.5
%         tau7(i) = -33.5;
%     end
    tau8(i) = out.tau_MPC(8,1,i);
%      if tau8(i) > 33.5
%         tau8(i) = 33.5;
%     elseif tau8(i) < -33.5
%         tau8(i) = -33.5;
%     end
    tau9(i) = out.tau_MPC(9,1,i);
%     if tau9(i) > 33.5
%         tau9(i) = 33.5;
%     elseif tau9(i) < -33.5
%         tau9(i) = -33.5;
%     end
    tau10(i) = out.tau_MPC(10,1,i);
%     if tau10(i) > 33.5
%         tau10(i) = 33.5;
%     elseif tau10(i) < -33.5
%         tau10(i) = -33.5;
%     end
end

tmax = t(end);
figure;

subplot(5,1,1)
plot(t,tau1,t,tau6,'Linewidth',1.5);
xlabel('Time (s)');
ylabel('Ab Torque (Nm)');
legend('Left Leg','Right Leg');
title('Joint Torque with MPC');
xlim([0 tmax]);
% ylim([-33.5 33.5]);
subplot(5,1,2)
plot(t,tau2,t,tau7,'Linewidth',1.5);
xlabel('Time (s)');
ylabel('Hip Torque (Nm)');
xlim([0 tmax]);
% ylim([-33.5 33.5]);
subplot(5,1,3)
plot(t,tau3,t,tau8,'Linewidth',1.5);
xlabel('Time (s)');
ylabel('Thigh Torque (Nm)');
xlim([0 tmax]);
% ylim([-33.5 33.5]);
subplot(5,1,4)
plot(t,tau4,t,tau9,'Linewidth',1.5);
xlabel('Time (s)');
ylabel('Calf Torque (Nm)');
xlim([0 tmax]);
% ylim([-33.5 33.5]);
subplot(5,1,5)
plot(t,tau5,t,tau10,'Linewidth',1.5);
xlabel('Time (s)');
ylabel('Ankle Torque (Nm)');
xlim([0 tmax]);
% ylim([-33.5 33.5]);

figure;
plot(t,tau1,t,tau2,t,tau3,t,tau4,t,tau5,'Linewidth',1.5);
hold on;
plot([0 tmax],[33.5 33.5],'b--');
plot([0 tmax],[51.7 51.7],'r--')
xlabel('Time (s)');
ylabel('Torque (Nm)');

xlim([0 7]);
ylim([-10 68]);
xline(1,'k:');
xline(2,'k:');
xline(3,'k:');
xline(4,'k:');
xline(5,'k:');
xline(6,'k:');
text(0.3,-8, '1 kg');
text(1.3,-8, '5 kg');
text(2.3,-8, '9 kg');
text(3.3,-8, '13 kg');
text(4.3,-8, '15 kg');
text(5.3,-8, '16 kg');
text(6.3,-8, '17 kg');

legend({'Ab','Hip','Thigh','Knee','Ankle',...
    'Torque limit for Ab, Hip, Thigh, and Ankle',...
    'Torque limit for Knee'},'orientation','horizontal','NumColumns',3);
title('Left Leg Joint Torques');
