tau = permute(out.tau_WBC,[1 2 3]);
tau = reshape(tau,10,[]);
size(tau);
figure;
t = [1:1:length(tau(1,:))]*0.001;
% out.time = t;
tmax = t(end);

subplot(5,1,1)
plot(t,tau(1,:),t,tau(6,:),'Linewidth',1.5);
xlabel('Time (s)');
ylabel('Ab Torque (Nm)');
legend('Left Leg','Right Leg');
title('Joint Torques');
xlim([0 tmax]);
ylim([-33.5 33.5]);
subplot(5,1,2)
plot(t,tau(2,:),t,tau(7,:),'Linewidth',1.5);
xlabel('Time (s)');
ylabel('Hip Torque (Nm)');
xlim([0 tmax]);
ylim([-33.5 33.5]);
subplot(5,1,3)
plot(t,tau(3,:),t,tau(8,:),'Linewidth',1.5);
xlabel('Time (s)');
ylabel('Thigh Torque (Nm)');
xlim([0 tmax]);
ylim([-33.5 33.5]);
subplot(5,1,4)
plot(t,tau(4,:),t,tau(9,:),'Linewidth',1.5);
xlabel('Time (s)');
ylabel('Calf Torque (Nm)');
xlim([0 tmax]);
ylim([-33.5 33.5]);
subplot(5,1,5)
plot(t,tau(5,:),t,tau(10,:),'Linewidth',1.5);
xlabel('Time (s)');
ylabel('Ankle Torque (Nm)');
xlim([0 tmax]);
ylim([-33.5 33.5]);