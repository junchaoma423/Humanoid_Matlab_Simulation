tmin = 0;
t = [];
a = size(out.xout);
t = linspace(0,a(1)*0.001,a(1));
tmax = t(end);
roll = [];
yaw = [];
pitch = [];
roll_h = [];
yaw_h  = [];
pitch_h  = [];

for i = 1:a(1)
    eul = quat2eul(out.xout(i,1:4),'xyz');
%     t(i) = out.time(i);
    roll(i) = eul(1)*180/pi;
    yaw(i) = eul(3)*180/pi;
    pitch(i) = eul(2)*180/pi;
%     eul_h = quat2eul(out.xout(1,1:4,i),'xyz');
%     roll_h(i) = eul_h(1)*180/pi;
%     yaw_h(i) = eul_h(3)*180/pi;
%     pitch_h(i) = eul_h(2)*180/pi;
end

figure;

subplot(3,1,1);

plot(t,roll,'Linewidth',1.5,'Color','b');    
title('Body Orientation in Euler Angles');
ylim([-2.5 2.5]);
xlabel('time (s)');
ylabel('Roll (Deg.)');
% legend('Actual','Desired');

subplot(3,1,3);
plot(t,yaw,'Linewidth',1.5,'Color','b');    

ylim([-2.5 2.5]);
xlabel('time (s)');
ylabel('Yaw (Deg.)');
% legend('Actual','Desired');

subplot(3,1,2);
plot(t,pitch,'Linewidth',1.5,'Color','b');    

ylim([-2.5 2.5]);
xlabel('time (s)');
ylabel('Pitch (Deg.)');
% %% hybrid model
% figure;
% 
% subplot(3,1,1);
% 
% plot(t,roll_h,'Linewidth',1.5,'Color','b');    
% title('Body Orientation in Euler Angles');
% ylim([-2.5 2.5]);
% xlabel('time (s)');
% ylabel('Roll (Deg.)');
% % legend('Actual','Desired');
% 
% subplot(3,1,3);
% plot(t,yaw_h,'Linewidth',1.5,'Color','b');    
% 
% ylim([-2.5 2.5]);
% xlabel('time (s)');
% ylabel('Yaw (Deg.)');
% % legend('Actual','Desired');
% 
% subplot(3,1,2);
% plot(t,pitch_h,'Linewidth',1.5,'Color','b');    
% 
% ylim([-2.5 2.5]);
% xlabel('time (s)');
% ylabel('Pitch (Deg.)');
% 
% % legend('Actual','Desired');