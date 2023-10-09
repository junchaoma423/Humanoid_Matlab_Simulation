l = length(out.MPCforce);
fx1 = zeros(l/12,1);
fy1 = zeros(l/12,1);
fz1 = zeros(l/12,1);

fx2 = zeros(l/12,1);
fy2 = zeros(l/12,1);
fz2 = zeros(l/12,1);

Mx1 = zeros(l/12,1);
My1 = zeros(l/12,1);
Mz1 = zeros(l/12,1);

Mx2 = zeros(l/12,1);
My2 = zeros(l/12,1);
Mz2 = zeros(l/12,1);

t = zeros(l,1);

for i = 1:(l/12)
    fx1(i) = out.MPCforce(((i-1)*12+1),1);
    fy1(i) = out.MPCforce(((i-1)*12+2),1);
    fz1(i) = out.MPCforce(((i-1)*12+3),1);

    fx2(i) = out.MPCforce(((i-1)*12+4),1);
    fy2(i) = out.MPCforce(((i-1)*12+5),1);
    fz2(i) = out.MPCforce(((i-1)*12+6),1);

end

figure;

plot(fz1)
hold on
plot(fz2)
