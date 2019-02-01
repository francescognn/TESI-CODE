clc
close

u=u5;


q=state_q(:,1:6);
qp=state_qp(:,1:6);

figure
plot(u)
hold on
grid on
plot(qp(:,5))

legend('input','output measured')