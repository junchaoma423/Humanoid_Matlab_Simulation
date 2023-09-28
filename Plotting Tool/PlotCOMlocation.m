global optimization
tmin = 0;
a = size(out.xout);
t = linspace(0,a(1)*0.001,a(1));
tmax = t(end);

% figure(3)
% plot3(x,y,z);
% xlim([-0.025 0.025]);
% ylim([-0.025 0.025]);
% zlim([0.28 0.35]);
x_h = [];
y_h = [];
z_h = [];

for i = 1:a(1)
    x_h(i) = out.x_hybrid(1,5,i);
    y_h(i) = out.x_hybrid(1,6,i);
    z_h(i) = out.x_hybrid(1,7,i);
end

figure;

subplot(3,2,1);
plot(t,out.xout(:,5),'Linewidth',1.5);
xlabel('Time (s)');title('Body CoM Position');
ylabel('CoM position X (m)');
hold on;
plot(t,x_h,'Linewidth',1.5);
% legend('Actual','Commanded');
xlim([tmin tmax]);
if optimization == 1
    hold on;
    plot(linspace(0,length(f1x)*0.001,length(f1x)), com_x(1,:),'--r','LineWidth',1.5)
%     legend('Actual','Optimization');
end

subplot(3,2,3);
plot(t,out.xout(:,6),'Linewidth',1.5);
xlabel('Time (s)');
ylabel('CoM position Y (m)');
xlim([tmin tmax]);
 ylim([-0.1 0.1]);
 hold on;
plot(t,y_h,'Linewidth',1.5);
if optimization == 1
    hold on;
    plot(linspace(0,length(f1x)*0.001,length(f1x)), com_y(1,:),'--r','LineWidth',1.5)
%     legend('Actual','Optimization');
end

subplot(3,2,5);
plot(t,out.xout(:,7),'Linewidth',1.5);
xlabel('Time (s)');
ylabel('CoM position Z (m)');
xlim([tmin tmax]);
ylim([0.45 0.65]);
hold on;
plot(t,z_h,'Linewidth',1.5);
if optimization == 1
    hold on;
    plot(linspace(0,length(f1x)*0.001,length(f1x)), com_z(1,:),'--r','LineWidth',1.5)
%     legend('Actual','Optimization');
end

%velocity
subplot(3,2,2);
t,out.xout(1,11) = 0;
plot(t,out.xout(:,11),'Linewidth',1.5);
xlabel('Time (s)');title('Body CoM Velocity');hold on;
% plot([0 tmax],[1.5 1.5],'Linewidth',1.5);
ylabel('CoM velocity X (m/s)');
xlim([tmin tmax]);
ylim([0 2]);

if optimization == 1
    hold on;
    plot(linspace(0,length(f1x)*0.001,length(f1x)), com_vx(1,:)/1.1,'--r','LineWidth',1.5)
    legend('Actual','Optimization');
end

subplot(3,2,4);
plot(t,out.xout(:,12),'Linewidth',1.5);
xlabel('Time (s)');
ylabel('CoM velocity Y (m/s)');
xlim([tmin tmax]);
ylim([-0.6 0.6]);
% xlim([0 3.6]);
% legend('Actual','Desired');
if optimization == 1
    hold on;
    plot(linspace(0,length(f1x)*0.001,length(f1x)), com_vy(1,:),'--r','LineWidth',1.5)
%     legend('Actual','Optimization');
end

subplot(3,2,6);
plot(t,out.xout(:,13),'Linewidth',1.5);
xlabel('Time (s)');
ylabel('CoM velocity Z (m/s)');

xlim([tmin tmax]);ylim([-0.6 0.6]);
if optimization == 1
    hold on;
    plot(linspace(0,length(f1x)*0.001,length(f1x)), com_vz(1,:),'--r','LineWidth',1.5)
    legend('Actual','Optimization');
end

% figure;
% stem(acc_t(1:end-1),dt_MPC_vec);
% xlabel('Time,s');title('Gait periods');
% ylabel('Gait period, s');
% xlim([0 tmax]);