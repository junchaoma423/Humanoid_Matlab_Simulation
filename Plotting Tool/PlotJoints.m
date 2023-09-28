figure;
subplot(2,1,1);
plot(out.time, out.qdout(:,1)); hold on;
plot(out.time, out.qdout(:,2)); 
plot(out.time, out.qdout(:,3)); 
plot(out.time, out.qdout(:,4)); 
plot(out.time, out.qdout(:,5)); 
plot([out.time(1) out.time(end)],[-21 -21],'r');
plot([out.time(1) out.time(end)],[21 21],'r'); title('Leg 1');xlabel('time, s');ylabel('Joint speed, rad/s')
% legend('hip1','hip2','thigh','calf','ankle');

subplot(2,1,2);
plot(out.time, out.qdout(:,6)); hold on;
plot(out.time, out.qdout(:,7)); 
plot(out.time, out.qdout(:,8)); 
plot(out.time, out.qdout(:,9)); 
plot(out.time, out.qdout(:,10)); 
plot([out.time(1) out.time(end)],[-21 -21],'r');
plot([out.time(1) out.time(end)],[21 21],'r'); title('Leg 2');xlabel('time, s');ylabel('Joint speed, rad/s')
legend('hip1','hip2','thigh','calf','ankle', 'lower bound', 'upper bound');

figure; 
plot(out.time,out.q_ankle_motor_L,out.time,out.qout(:,10))