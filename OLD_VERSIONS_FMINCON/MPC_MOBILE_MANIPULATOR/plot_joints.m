close all

figure;
plot(x(:,1:3));
grid on
legend('x','y','th');

figure;
plot(x(:,4:9));
legend('1','2','3','4,','5','6');
grid on

errors = xs-x;
figure;
plot(errors)
grid on
legend('1','2','3','4,','5','6');


% %%
% 
% figure;
% q=x(:,1:9);
% for i = 1:size(x,1)
%     robot.plot(q(i,:),'workspace',[-1, 1, -1, 1, -1, 2])  
% end