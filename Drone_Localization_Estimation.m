clear
close all
ISD=200; %Inter site distance
No_BS=4; %Number of base stations   
%Location of the base stations
BS_pos=[0 0;ISD/2 ISD/2;ISD/2 -ISD/2;ISD 0]; %[X Y]

Drone_pos=ISD*rand(1,2);     %True location of drone
angle_std=5/180*pi;    %Standard deviation of angle error

for kk=1:No_BS
    th(kk,1)=atan2(Drone_pos(1,2)-BS_pos(kk,2),Drone_pos(1,1)-BS_pos(kk,1));
end

No_real=1000;

for kk=1:No_real
    thr=th+randn(No_BS,1)*angle_std;
    h=BS_pos(:,1).*tan(thr)-BS_pos(:,2);
    G=[tan(thr) -ones(No_BS,1)];
    xhat(kk,:)=transpose(pinv(G)*h);
    e(kk,1)=norm(Drone_pos-xhat(kk,:));
end
figure(1)
plot(BS_pos(:,1),BS_pos(:,2),'kd'); 
hold on 
angle_std=5/180*pi;
plot(xhat(:,1),xhat(:,2),'c.')
plot(Drone_pos(1),Drone_pos(2),'kx'); 
hold off

if 0
    figure(2)
    ecdf(e)
end
[mean(e) std(e)]