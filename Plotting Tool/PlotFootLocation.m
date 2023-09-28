global optimization
tmax = 1.5;
figure;
subplot(3,2,1);
plot(out.time, out.fpfv(:,1),'b','LineWidth',1.5); title('Foot 1 Position'); ylabel('x');
if optimization == 1
    hold on;
    plot(linspace(0,length(f1x)*0.001,length(f1x)), FootPosition(1,:),'--r','LineWidth',1.5);xlim([0 tmax]);
end
subplot(3,2,3);
plot(out.time, out.fpfv(:,2),'b','LineWidth',1.5);ylabel('y');
if optimization == 1
    hold on;
    plot(linspace(0,length(f1x)*0.001,length(f1x)), FootPosition(2,:),'--r','LineWidth',1.5);xlim([0 tmax]);
end
subplot(3,2,5);
plot(out.time, out.fpfv(:,3)-0.0246,'b','LineWidth',1.5);ylabel('z'); xlabel('t');
if optimization == 1
    hold on;
    plot(linspace(0,length(f1x)*0.001,length(f1x)), FootPosition(3,:),'--r','LineWidth',1.5);xlim([0 tmax]);
%     legend('Simulation','Optimization');
end

subplot(3,2,2);
plot(out.time, out.fpfv(:,7),'b','LineWidth',1.5); title('Foot 2 Position'); ylabel('x');
if optimization == 1
    hold on;
    plot(linspace(0,length(f1x)*0.001,length(f1x)), FootPosition(4,:),'--r','LineWidth',1.5);xlim([0 tmax]);
end
subplot(3,2,4);
plot(out.time, out.fpfv(:,8),'b','LineWidth',1.5);ylabel('y');
if optimization == 1
    hold on;
    plot(linspace(0,length(f1x)*0.001,length(f1x)), FootPosition(5,:),'--r','LineWidth',1.5);xlim([0 tmax]);
end
subplot(3,2,6);
plot(out.time, out.fpfv(:,9)-0.0246,'b','LineWidth',1.5);ylabel('z'); xlabel('t');
if optimization == 1
    hold on;
    plot(linspace(0,length(f1x)*0.001,length(f1x)), FootPosition(6,:),'--r','LineWidth',1.5);
    legend('Simulation','Optimization'); xlim([0 tmax]);
end

% figure;
% plot(out.fpfv(:,1),out.fpfv(:,3),'b','LineWidth',1.5);