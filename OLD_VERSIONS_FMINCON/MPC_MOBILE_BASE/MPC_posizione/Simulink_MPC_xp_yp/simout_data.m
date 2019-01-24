
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



errore=[x(1:length(xd)).' y(1:length(xd)).' th(1:length(xd)).']-xd.';

figure
plot(errore)
grid on