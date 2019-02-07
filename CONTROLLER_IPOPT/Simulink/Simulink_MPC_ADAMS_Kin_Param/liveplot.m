function [] = liveplot(q)
xd=evalin('base','xd');

[P_ee,Psi_ee,T] = FK(q);
P=zeros(3,10);
for kk = 1:10
    P(:,kk) = T((kk*4-3):(kk*4-1),4);
end
figure(1);
% plot3([P(1,3) P(1,4)],[P(2,3) P(2,4)],[P(3,3)
% P(3,3)],'b','LineWidth',10); hold on
plot3([P(1,4) P(1,4)],[P(2,4) P(2,4)],P(3,3:4),'b','LineWidth',10)
hold on
plot3(P(1,4:5),P(2,4:5),P(3,4:5),'color',[0.5 0.5 0.5],'LineWidth',3)
plot3(P(1,5:6),P(2,5:6),P(3,5:6),'k','LineWidth',3)
plot3(P(1,6:7),P(2,6:7),P(3,6:7),'color',[0.5 0.5 0.5],'LineWidth',3)
plot3(P(1,7:8),P(2,7:8),P(3,7:8),'k','LineWidth',3)
plot3(P(1,8:9),P(2,8:9),P(3,8:9),'color',[0.5 0.5 0.5],'LineWidth',3)
plot3(P(1,9:10),P(2,9:10),P(3,9:10),'k','LineWidth',3)
plot3(xd(10,:),xd(11,:),xd(12,:),'r')
plot3(xd(1,:),xd(2,:),xd(12,:).*0,'k')
grid on
axis([-1 2 -1 2 0 2])
hold off
view(-40, 20);
xlabel('x')
ylabel('y')
zlabel('z')
end

