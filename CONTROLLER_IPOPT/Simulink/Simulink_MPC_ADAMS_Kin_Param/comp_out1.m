

for i=1:size(state_q,1)
[P_ee_out(:,i),AAA]=jacobian_MM([0 0 0 state_q(i,:)].');
end

figure(1)

plot3(P_ee_out(1,:),P_ee_out(2,:),P_ee_out(3,:))
hold on
grid on
plot3(xd(10,:),xd(11,:),xd(12,:),'*')
xlabel('x')
ylabel('y')
zlabel('z')
legend('robot','desired')

% error_abs_xyz=sqrt((P_ee_out(1,:)-xd(10,:))^2+(P_ee_out(2,:)-xd(11,:))^2+(P_ee_out(3,:)-xd(12,:))^2);
% figure(2)
% plot(error_abs_xyz)
% grid on
% title('error xyz')

state_qp=state_qp(:,1:6);
figure(3)
subplot(321)
plot(state_qp(:,1))
hold on
grid on
plot(u_given(:,3))
title('thetap1')
legend('meas','giv')

subplot(322)
plot(state_qp(:,2))
hold on
grid on
plot(u_given(:,4))
title('thetap2')
legend('meas','giv')

subplot(323)
plot(state_qp(:,3))
hold on
grid on
plot(u_given(:,5))
title('thetap3')
legend('meas','giv')

subplot(324)
plot(state_qp(:,4))
hold on
grid on
plot(u_given(:,6))
title('thetap4')
legend('meas','giv')

subplot(325)
plot(state_qp(:,5))
hold on
grid on
plot(u_given(:,7))
title('thetap5')
legend('meas','giv')

subplot(326)
plot(state_qp(:,6))
hold on
grid on
plot(u_given(:,8))
title('thetap6')
legend('meas','giv')

warning off

savename = ['REAL_TEST_'  datestr(now, 'HH-MM dd-mmm-yyyy')];
matfile = fullfile('Data_saved/', savename);
save(matfile)
