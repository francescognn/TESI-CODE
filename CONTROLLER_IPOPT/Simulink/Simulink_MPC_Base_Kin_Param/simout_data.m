 
close 
clearvars -EXCEPT  simout xd T_elapsed_vect tt N disturb
clc

for i=1:size(simout.Data,3)
    
    i=real(i);
    
    X(:,i)=simout.Data(1,:,i);   
end

if size(X,2)>=size(xd,2)
    X=X(:,1:size(xd,2));
else
    xd=xd(:,1:size(x,2));
end
x=X(1,:);y=X(2,:);th=X(3,:);
figure
plot(x,y,'-o')
set(gcf,'color','white')
hold on
% plot(xd(1,1:N),xd(2,1:N),'o')
% hold on;
grid on;
plot(xd(1,:),xd(2,:),'-*')
legend('Simulated Trajectory','Real Trajectory')
title('X-Y trajectory')
axis equal

finpos=[ x(length(xd)) y(length(xd)) th(length(xd))];

err_fin_pos=xd(:,end).'-finpos;

errore=[x(1:length(xd)).' y(1:length(xd)).' th(1:length(xd)).']-xd.';

figure
set(gcf,'color','white')
plot(tt,errore)
legend('x','y','theta')
title('Errors (m)')
grid on

figure;
set(gcf,'color','white')
plot(tt,sqrt(errore(:,1).^2+errore(:,2).^2))
legend('Abs Error')
title('Cartesian error (m)')
grid on

figure
set(gcf,'color','white')
plot(T_elapsed_vect)
ylabel('T elapsed')
xlabel('Step')
grid on



savefile=['MPC_kin_Param_N=',num2str(N),'_trajCurves_r=0.3_NoDist'];

save(savefile);


