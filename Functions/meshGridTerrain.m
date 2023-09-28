function [x_grid,y_grid,z_heights] = meshGridTerrain

depth = -20;
width = 0.1;
x_init = 0.4;
x_grid = [-1,x_init,x_init+0.01];
z = [0, 0, depth];

%stone 1:
c2c1 = 0.2;
s1 = [0.55, 0.6, 0.65, 0.651];
z1 = [0,0,0, depth];

%stone 2:
c2c2 = 0.25;
s2 = [0.8, 0.85, 0.9, 0.91];
z2 = [0,0,0, depth];

%stone 3:
c2c3 = 0.2;
s3 = [1.0, 1.05, 1.1, 1.11];
z3 = [0,0,0, depth];

%stone 4:
c2c4 = 0.3;
s4 = [1.3, 1.35, 1.4, 1.41];
z4 = [0,0,0, depth];

%stone 5:
c2c5 = 0.15;
s5 = [1.45, 1.5, 1.55, 1.551];
z5 = [0,0,0, depth];

%stone 6(ground):
c2c6 = 0.2;
s6 = [1.65, 1.7, 5];
z6 = [0,0,0];

x_grid = [x_grid, s1,s2,s3,s4,s5,s6];
z = [z,z1,z2,z3,z4,z5,z6];
y_grid = [-2 2];
z_heights = [z;z]';



% num = 5; %number of stepping stones
% length = 0.08*ones(1,30); %length vector
% % length = [0.03 0.075 0.045 0.09 0.06]
% c2c = [0.2 0.15 0.2 0.3 0.2 0.5 0.5*ones(1,30)]; %center to center distance vector
% % h = [0.04 -0.02 0.01 -0.03 0.0 0.0 0.0 0.0 0.0, zeros(1,10)];
% h = zeros(1,20);
% x_init = 0.4; %terrain starting position
% 
% x_grid = [-1, x_init+length(1)/2];
% z = [0,0];
% for i = 1:num
%     x_grid = [x_grid, x_init+sum(c2c(1:i))-c2c(i)/2,x_init+sum(c2c(1:i))-length(i)/2,x_init+sum(c2c(1:i)),x_init+sum(c2c(1:i))+length(i)/2];
%     z = [z,-100,h(i),h(i),h(i)];
% end
% x_grid = [x_grid,5];
% z = [z,0];
% y_grid = [-2 2];
% z_heights = [z;z]';


plot(x_grid,z); ylim([-0.5 0.5]);
end