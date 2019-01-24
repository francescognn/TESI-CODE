function xd_ee = trajectory_joint(T,Ttot)

t = 0:T:Ttot;
x = linspace(0,1,length(t))';
y = zeros(1,length(t))';
th = zeros(1,length(t))';
th_arm = linspace(-pi/8,pi/4,length(t))';
xd_ee = [x,y,th,th_arm*ones(1,6)];


