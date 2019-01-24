close 

clc

for i=1:size(simout.Data,3)
    i=real(i);
    
    x(i)=simout.Data(:,1,i);
    y(i)=simout.Data(:,2,i);
    th(i)=simout.Data(:,3,i);
    
end

plot(x,y)
hold on;
grid on;
plot(xd(1,:),xd(2,:))

legend('Simulated Trajectory','Real Trajectory')

if xd(1,end)>25
    traj='arc';
else
    traj='complex';
end

savefile=['MPC_dyn_N=',num2str(N),traj,' traj.mat'];

save(savefile);
