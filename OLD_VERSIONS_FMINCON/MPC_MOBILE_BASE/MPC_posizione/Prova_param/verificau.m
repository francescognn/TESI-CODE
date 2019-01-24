close all
clc

for i=1:size(p,1)
ufin(i)=tretratti(N*T,N*T,p(i,:),T);
end
    plot(ufin)