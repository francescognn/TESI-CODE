function [ dqq ] = sys_dynamics(q,dq,tau)

B=Bmatrix(q);
C=Cmatrix(q,dq);
g=gmatrix(q);

dqq = B \ ( tau - C - g );

end

