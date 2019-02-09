
clc

for aa=1:size(X_steps.signals.values,3)
    
    J_parts(aa,:)=J_comp(:,:,aa);
    
for j=1:N+1
    xxee(j)=X_steps.signals.values(10,j,aa);
    yyee(j)=X_steps.signals.values(11,j,aa);
    zzee(j)=X_steps.signals.values(12,j,aa);
    
    cc(:,j)  = SCA(X_steps.signals.values(1:9,j,aa));
end

figure(8)
plot3(xd(10,:),xd(11,:),xd(12,:))
hold on
plot3(xxee,yyee,zzee,'linewidth',2)
grid on
view(0,0)
% axis([-1 1 -1 1 -1 1])
xlabel('x')
ylabel('y')
zlabel('z')
drawnow()
pause(0.1)
hold off
end

figure(9)
plot(J_parts)
grid on
legend('J','h1','h2','h3','h4','h5')

figure(10)
spy(fun<0)

figure(11)
spy(fun2<0)


for jj=1:size(state_x,2)

    X_FK(:,jj)=FK(X(1:9,jj));
    
end

