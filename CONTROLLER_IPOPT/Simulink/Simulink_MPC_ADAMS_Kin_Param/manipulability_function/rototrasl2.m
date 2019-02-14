function A = rototrasl2(DHTABLE)
% RETURNS A CELL OF ALL ROTOTRASLATION MATRIX OF A MANIPULATOR
N = size(DHTABLE,1);

A = cell(1,N);


for i = 1:N
    
    a = DHTABLE(i,1);
    alp = DHTABLE(i,2);    
    d = DHTABLE(i,3);
    th = DHTABLE(i,4);
    A{i} =  [ cos(th) -sin(th)*cos(alp)  sin(th)*sin(alp) a*cos(th);
              sin(th)  cos(th)*cos(alp) -cos(th)*sin(alp) a*sin(th);
              0          sin(alp)          cos(alp)        d;
              0            0                 0             1];   
end
