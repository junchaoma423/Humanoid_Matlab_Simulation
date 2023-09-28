close all
figure; 
L = length(out.arm_dq(:,1));
t = linspace(0, out.time(end), L);
tmax = t(end);

subplot(3,1,1);
armSpeed =squeeze(out.arm_dq);
plot(t,armSpeed(:,1))
hold on
plot(t,armSpeed(:,2))
% xlabel('Time (s)');
ylabel('Speed (rad/s)');
title('Yaw Speed')
% xline(1,'k:');
% xline(2,'k:');
% xline(3,'k:');
% xline(4,'k:');
% xline(5,'k:');
% xline(6,'k:');
% text(0.3,-8, '1 kg');
% text(1.3,-8, '5 kg');
% text(2.3,-8, '9 kg');
% text(3.3,-8, '13 kg');
% text(4.3,-8, '15 kg');
% text(5.3,-8, '16 kg');
% text(6.3,-8, '17 kg');
legend('Left Leg','Right Leg');
% xlim([0 tmax]);
% ylim([-13 13]);

subplot(3,1,2)
plot(t,armSpeed(:,3))
hold on
plot(t,armSpeed(:,4))
% xlabel('Time (s)');
ylabel('Speed (rad/s)');
title('Upper-arm Speed')
% xline(1,'k:');
% xline(2,'k:');
% % xline(3,'k:');
% xline(4,'k:');
% % xline(5,'k:');
% xline(6,'k:');
% % text(0.3,-8, '1 kg');
% % text(1.3,-8, '3 kg');
% % text(2.3,-8, '5 kg');
% % text(3.3,-8, '7 kg');
% % text(4.3,-8, '8 kg');
% % text(5.3,-8, '9 kg');
% % text(6.3,-8, '10 kg');
legend('Left Leg','Right Leg');
% xlim([0 tmax]);
% ylim([-13 13]);


subplot(3,1,3)
plot(t,armSpeed(:,5))
hold on
plot(t,armSpeed(:,6))
% xlabel('Time (s)');
ylabel('Speed (rad/s)');
title('Fore-arm Speed')
% xline(1,'k:');
% xline(2,'k:');
% % xline(3,'k:');
% xline(4,'k:');
% % xline(5,'k:');
% xline(6,'k:');
% % text(0.3,8, '1 kg');
% % text(1.3,8, '3 kg');
% % text(2.3,8, '5 kg');
% % text(3.3,8, '7 kg');
% % text(4.3,8, '8 kg');
% % text(5.3,8, '9 kg');
% % text(6.3,8, '10 kg');
legend('Left Leg','Right Leg');
% xlim([0 tmax]);
% ylim([-13 13]);

