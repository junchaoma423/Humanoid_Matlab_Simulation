close all
figure; 
L = length(out.armTorque(:,1));
t = linspace(0, 9, L);
tmax = t(end);

subplot(3,1,1);
armTorque = out.tau_arms;
plot(t,armTorque(:,1))
hold on
plot(t,armTorque(:,2))
xlabel('Time (s)');
ylabel('Torque (Nm)');
title('Yaw Torque')
xline(1,'k:');
xline(2,'k:');
xline(3,'k:');
xline(4,'k:');
xline(5,'k:');
xline(6,'k:');
% text(0.3,-8, '1 kg');
% text(1.3,-8, '5 kg');
% text(2.3,-8, '9 kg');
% text(3.3,-8, '13 kg');
% text(4.3,-8, '15 kg');
% text(5.3,-8, '16 kg');
% text(6.3,-8, '17 kg');
legend('Left Leg','Right Leg');
xlim([0 tmax]);
ylim([-13 13]);

subplot(3,1,2)
plot(t,armTorque(:,3))
hold on
plot(t,armTorque(:,4))
xlabel('Time (s)');
ylabel('Torque (Nm)');
title('Upper-arm Torque')
xline(1,'k:');
xline(2,'k:');
% xline(3,'k:');
xline(4,'k:');
% xline(5,'k:');
xline(6,'k:');
% text(0.3,-8, '1 kg');
% text(1.3,-8, '3 kg');
% text(2.3,-8, '5 kg');
% text(3.3,-8, '7 kg');
% text(4.3,-8, '8 kg');
% text(5.3,-8, '9 kg');
% text(6.3,-8, '10 kg');
legend('Left Leg','Right Leg');
xlim([0 tmax]);
ylim([-13 13]);


subplot(3,1,3)
plot(t,armTorque(:,5))
hold on
plot(t,armTorque(:,6))
xlabel('Time (s)');
ylabel('Torque (Nm)');
title('Fore-arm Torque')
xline(1,'k:');
xline(2,'k:');
% xline(3,'k:');
xline(4,'k:');
% xline(5,'k:');
xline(6,'k:');
% text(0.3,8, '1 kg');
% text(1.3,8, '3 kg');
% text(2.3,8, '5 kg');
% text(3.3,8, '7 kg');
% text(4.3,8, '8 kg');
% text(5.3,8, '9 kg');
% text(6.3,8, '10 kg');
legend('Left Leg','Right Leg');
xlim([0 tmax]);
ylim([-13 13]);

