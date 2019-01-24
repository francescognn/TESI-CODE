close all
figure;
plot3(xd(:,4),xd(:,5),xd(:,6));
hold on
grid on
for i = 1:size(xd,1)
    plot3(x(i,10),x(i,11),x(i,12),'or');
    pause(0.1)
end

tt=0:T:(Ttot+T);
figure;
plot(tt,xd(:,4)-x(:,10),tt,xd(:,5)-x(:,11),tt,xd(:,6)-x(:,12))

figure;
plot(x(:,1));
hold on
grid on
plot(x(:,2));
plot(x(:,3));
hold off
legend('x','y','th');
figure;
hold on
plot(x(:,4));
plot(x(:,5));
plot(x(:,6));
plot(x(:,7));
plot(x(:,8));
plot(x(:,9));
hold off
legend('1','2','3','4,','5','6');
grid on

% %%
% 
% figure;
% q=x(:,1:9);
% for i = 1:size(x,1)
%     robot.plot(q(i,:),'workspace',[-1, 1, -1, 1, -1, 2])  
% end