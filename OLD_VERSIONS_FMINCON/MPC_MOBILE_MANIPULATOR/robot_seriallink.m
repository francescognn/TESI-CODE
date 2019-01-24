syms X Y TH th1 th2 th3 th4 th5 th6 
q = [X Y TH th1 th2 th3 th4 th5 th6 ];

[DHtable,Abase,CoM_pos,m,In] = parameters_MM(q);

L(1)=Link('theta',pi/2,'a',0,'alpha',pi/2,'offset',0,'prismatic');
L(2)=Link('theta',-pi/2,'a',0,'alpha',-pi/2,'offset',0,'prismatic');
for i = 3:9
    L(i)=Link('a',DHtable(i,1), 'alpha',DHtable(i,2),'d',DHtable(i,3),'offset',0,'revolute');
    L(i).m = m(i);
    L(i).I  = In{i};
    L(i).r = CoM_pos(:,i).';
end

robot=SerialLink(L,'base',Abase);
