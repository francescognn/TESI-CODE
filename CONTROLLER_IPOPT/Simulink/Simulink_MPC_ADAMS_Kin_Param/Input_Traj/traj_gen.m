function [xd,x0,tt] = traj_gen(T,t_max,Npoint)


t = linspace(0,t_max,Npoint);

figure;
points = ginput(Npoint);
grid on


x = [points(:,1)].*10;
y = [points(:,2)].*10;

tt = 0:T:t_max;
slope0 = 0;
slopeF = 0;
xq = spline(t,[slope0; x; slopeF],tt);
yq = spline(t,[slope0; y; slopeF],tt);

xpq = diff(xq)/T;
ypq = diff(yq)/T;
z=xpq+ypq.*(1i);
th = atan2(imag(z),real(z));
xd=[xq(2:end).' yq(2:end).' th.'];
x0=xd(1,:);
tt=tt(1:end-1);
end



