clear all
close all
clc

X=cell(5,5);

for kkk=1:5
    for jjj=1:5
        
        namefile=['prova_mb-mm:' num2str(kkk) '-' num2str(jjj) '.mat'];
        load(namefile)
        
        X{kkk,jjj}=Xcomp;
        clearvars -EXCEPT X kkk jjj
    
    end
end

ZZ_MIO_STEP

clearvars -EXCEPT X xd N

xd_val= [ xd(:,1:N+1); xd(1:2,1:N+1); 1.5*ones(1,N+1); zeros(3,N+1)];
 
% figure
% 
% plot(T_horizon,unew(1:end,:),'-o')
% hold on
% grid on
% legend('v','omega')
% 
% figure
% 
% plot(abs_xy_err)
% grid on
% title('Abs error base xy')
% 
% 
% figure
% 
% plot(Xcomp(1,:),Xcomp(2,:),'-o')
% hold on 
% grid on
% plot(xd_val(1,:),xd_val(2,:),'-*')
% legend('computed','desired')
% 
% %%***TTTTTT>>>>>>
% 
% telapsed(5)=toc(tstart);
% 
% figure
% 
% plot3(Xcomp(4,:),Xcomp(5,:),Xcomp(6,:))
% 
% hold on
% grid on
% 
% plot3(xd_val(4,:),xd_val(5,:),xd_val(6,:))
% 
% legend('computed','desired')
% 
figure
mb=2;
for k=1:5
error_ee=sqrt((X{mb,k}(4,:)-xd_val(4,:)).^2+(X{mb,k}(5,:)-xd_val(5,:)).^2+(X{mb,k}(6,:)-xd_val(6,:)).^2);
plot(error_ee);
grid on
hold on
end
title(['abs error of EE xyz for m_b=' num2str(mb)])
legend('m_m=1','m_m=2','m_m=3','m_m=4','m_m=5')

figure
mm=5;
for k=1:5
error_ba=sqrt((X{mm,k}(1,:)-xd_val(1,:)).^2+(X{mm,k}(2,:)-xd_val(2,:)).^2);
plot(error_ba);
grid on
hold on
end
title(['base abs error xyz for m_m=' num2str(mm)])
legend('m_b=1','m_b=2','m_b=3','m_b=4','m_b=5')


% figure
% 
% set(gca,'xticklabel', 1);
% tdiff=diff(telapsed);
% tesec=[telapsed(1) tdiff(1:end-1) telapsed(5)];
% 
% for k=1:5
% plot(k-1,tesec(k),'s','MarkerSize',10,'LineWidth',20)
% hold on
% grid on
% end
% legend('importing time','definitions','propagazione stato','soluzione','whole script')
