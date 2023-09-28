function [i_MPC_var,dt_MPC] = AssignMPCStage(t)
global acc_t dt_MPC_vec i_MPC_var dt_MPC i_gait gait
    idx = find(acc_t<t+1e-3);
    
    i_MPC_var = idx(end);
    dt_MPC = dt_MPC_vec(i_MPC_var);
    idx_gait = rem(i_MPC_var,10);
    if gait <= 2
        if 1 <= idx_gait && idx_gait <= 5
            i_gait = 1;
        else
            i_gait = 0;
        end
    elseif  gait == 3
        if 1 <= idx_gait && idx_gait <= 4
            i_gait = 1;
        elseif 6 <= idx_gait && idx_gait <= 9
            i_gait = 0;
        else
            i_gait = 2;

        end
    end
end