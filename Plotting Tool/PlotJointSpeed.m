%% MPC torque
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
    tau8(i) = out.tau_MPC(8,1,i);
    tau9(i) = out.tau_MPC(9,1,i);
    tau10(i) = out.tau_MPC(10,1,i);
end

%% Arm Speed
speed1 = out.qdout(:,1);
speed2 = out.qdout(:,2);
speed3 = out.qdout(:,3);
speed4 = out.qdout(:,4);
speed5 = out.qdout(:,5);
speed6 = out.qdout(:,6);
speed7 = out.qdout(:,7);
speed8 = out.qdout(:,8);
speed9 = out.qdout(:,9);
speed10 = out.qdout(:,10);

%% Joint Power Calculation
power1 = zeros(L,1);
power2 = zeros(L,1);
power3 = zeros(L,1);
power4 = zeros(L,1);
power5 = zeros(L,1);
power6 = zeros(L,1);
power7 = zeros(L,1);
power8 = zeros(L,1);
power9 = zeros(L,1);
power10 = zeros(L,1);

for i = 1:L
    power1(i) = tau1(i) * speed1(i);
    power2(i) = tau2(i) * speed2(i);
    power3(i) = tau3(i) * speed3(i);
    power4(i) = tau4(i) * speed4(i);
    power5(i) = tau5(i) * speed5(i);
    power6(i) = tau6(i) * speed6(i);
    power7(i) = tau7(i) * speed7(i);
    power8(i) = tau8(i) * speed8(i);
    power9(i) = tau9(i) * speed9(i);
    power10(i) = tau10(i) * speed10(i);
end

tmax = t(end);

figure;

subplot(5,1,1)
plot(t,power1,t,power6,'Linewidth',1.5);
xlabel('Time (s)');
ylabel('Ab Power (Watt)');
legend('Left Leg','Right Leg');
title('Joint Power with MPC');
xlim([0 tmax]);
% ylim([-33.5 33.5]);
subplot(5,1,2)
plot(t,power2,t,power7,'Linewidth',1.5);
xlabel('Time (s)');
ylabel('Hip Power (Watt)');
xlim([0 tmax]);
% ylim([-33.5 33.5]);
subplot(5,1,3)
plot(t,power3,t,power8,'Linewidth',1.5);
xlabel('Time (s)');
ylabel('Thigh Power (Watt)');
xlim([0 tmax]);
% ylim([-33.5 33.5]);
subplot(5,1,4)
plot(t,power4,t,power9,'Linewidth',1.5);
xlabel('Time (s)');
ylabel('Calf Power (Watt)');
xlim([0 tmax]);
% ylim([-33.5 33.5]);
subplot(5,1,5)
plot(t,power5,t,power10,'Linewidth',1.5);
xlabel('Time (s)');
ylabel('Ankle Power (Watt)');
xlim([0 tmax]);
% ylim([-33.5 33.5]);