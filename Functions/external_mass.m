function out = external_mass(t)
    global gait detach pickup dropoff throw
%     Em = 0.001*(t < 1.001) + 10.201*(1.001 <= t && t <= 100) + 0.01*(100 < t);
%     Em = 1*(t < 1.001) + 5*(1.001 <= t && t < 2.001) + 9*(2.001 <= t && t < 3.001) + 13*(3.001 <= t && t < 4.001) + 15*(4.001 <= t && t < 5.001) + 16*(5.001 <= t && t < 6.001) + 17*(6.001 <= t && t < 7);

%     Em = 1*(t < 1.001) + 2*(1.001 <= t && t < 2.001) + 3*(2.001 <= t && t < 4.001) + 5*(4.001 <= t && t < 7.001);
%       Em = 3;
    Em = 0;

%     Em = 3*(t < 1) + 6*(1 <= t && t < 2) + 6*(2 <= t && t < 3) ...
%         + 10*(3 <= t && t < 4) + 10*(4 <= t && t < 5) + 14*(t >= 5);
    %% pickupthrow
%     detach = 0 + 1*(t < 3 || t > 7.7);
%     fx_ext = 0 + (0)*(1.5 <= t && t <= 1.6);
%     fy_ext = 0;
%     fz_ext = 0;
%     
%     pickup = 1*(t < 5);
%     dropoff = 0*(t >= 9);
%     throw = 1*(t >= 5);

    %% pickupwalkdropoff
%     detach = 0 + 1*(t < 3 || t > 10.6);
%     fx_ext = 0 + (0)*(1.5 <= t && t <= 1.6);
%     fy_ext = 0;
%     fz_ext = 0;
%     
%     pickup = 1*(t < 5);
%     dropoff = 1*(t >= 9);
%     throw = 0*(t >= 5);

    %% pickupwalkthrow
%     detach = 0 + 1*(t < 3 || t > 7.6);
%     fx_ext = 0 + (0)*(7.5 <= t && t <= 7.6);
%     fy_ext = 0;
%     fz_ext = 0;
%     
%     pickup = 1*(t < 5);
%     dropoff = 0*(t >= 9);
%     throw = 1*(t >= 7);

    %% pickupwalkthrow
%     detach = 0 + 1*(t < 3 || t > 7.2);
%     fx_ext = 0 + (0)*(7.5 <= t && t <= 7.6);
%     fy_ext = 0;
%     fz_ext = 0;
%     
%     pickup = 1*(t < 5);
%     dropoff = 1*(t >= 7);
%     throw = 0*(t >= 7);

 %% pickupturnbackdropoff
%     detach = 0 + 1*(t < 1.6 || t > 7.6);
%     fx_ext = 0 + (0)*(7.5 <= t && t <= 7.6);
%     fy_ext = 0;
%     fz_ext = 0;
%     
%     pickup = 1*(t < 2.5);
%     dropoff = 1*(t >= 7);
%     throw = 0*(t >= 7);
%% 90 deg table - traditional approach
%     detach = 0 + 1*(t < 2.6 || t > 5.6);
%     fx_ext = 0 + (0)*(7.5 <= t && t <= 7.6);
%     fy_ext = 0;
%     fz_ext = 0;
%     
%     pickup = 1*(t < 3.5);
%     dropoff = 1*(t >= 5);
%     throw = 0*(t >= 7);
%% 90 deg table - faster
%     detach = 0 + 1*(t < 2.4 || t > 3.9);
%     %     detach = 1;
%     fx_ext = 0 + (0)*(7.5 <= t && t <= 7.6);
%     fy_ext = 0;
%     fz_ext = 0;
% 
%     pickup = 1*(t < 3);
%     dropoff = 1*(t >= 3);
%     throw = 0*(t >= 7);
%% doNothing
    detach = 0;
    fx_ext = 0;
    fy_ext = 0;
    fz_ext = 0;
    
    pickup = 0*(t < 5);
    dropoff = 0*(t >= 9);
    throw = 0*(t >= 7);
%% 
    gait = gaitScheduled(t);
    out = [Em;fx_ext;fy_ext;fz_ext];
end

function gait = gaitScheduled(t)
%     gait =  0*(5.6 <= t && t <= 8.4)  ; %pickupthrow
%     gait =  4*(5.6 <= t && t <= 8.4)  ; %pickupwalkdropoff
%     gait =  4*(5.6 <= t && t <= 18.4)  ; %pickupwalkthrow
% gait =  4*(t <= 2) + 4*(3 <= t && t <= 4.8); %90 deg table traditional
    gait = 4;
end