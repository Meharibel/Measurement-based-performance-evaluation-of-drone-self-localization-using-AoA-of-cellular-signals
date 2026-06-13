%Angle of Arrival based localization of a drone
%Developed on 10.5.2021
clear 
close all
%Base station position
ISD=250;    %Inter site distance
bs_pos=[0 2*ISD*exp(i*[1:6]*60/180*pi)];    %Base station positions as complex numbers
No_BS=length(bs_pos);
p_BS= [real(bs_pos); imag(bs_pos)];         %Base station positions[x y]

%Initial posion
%Drone positions as complex numer
drone_pos=(ISD/2*[0:.005:2]).*exp(1i*[0:.005:2]*2*pi)*exp(-i*0.333*2*pi);
%Drone position [x y]
p_drone= [real(drone_pos); imag(drone_pos)];
p_drone= p_drone + 1*randn(size(p_drone)); 


T=length(p_drone);
std_angle=[6 3.6]/180*pi;
%th=Inf;
No_real=100;
for ii=1:length(std_angle)
    for jj=1:No_real
        for t=1:T
            for k=1:No_BS
                %Angle measurements
                phi(k,t)=atan((p_drone(2,t)-p_BS(2,k))./(p_drone(1,t)-p_BS(1,k)))+randn*std_angle(ii);
                %Calulated tangent
                tau(k,t)=tan(phi(k,t));
                s(k,t)=sin(phi(k,t));
                c(k,t)=cos(phi(k,t));
                %Scaled BS location
                h(k,1)=[-tau(k,t) 1]*p_BS(:,k);
                b(k,1)=[s(k,t) -c(k,t)]*p_BS(:,k);
            end;
            A=[s(:,t) -c(:,t) -p_BS(1,:)'.*c(:,t)-p_BS(2,:)'.*s(:,t)];
            G=[ -tau(:,t) ones(size(tau(:,t)))];  %G matrix
            x_hat(:,t)=pinv(G)*h; %Linear 'measurement'
            u_hat(:,t)=pinv(A)*b; %Auxilary variable method
            x_hat2(1,t)=(u_hat(1,t)-u_hat(3,t)*u_hat(2,t))/(1+u_hat(3,t)^2);
            x_hat2(2,t)=(u_hat(2,t)+u_hat(3,t)*u_hat(1,t))/(1+u_hat(3,t)^2);
            phi_hat2(t,1)=atan(u_hat(3,t));
            e(ii,t+(jj-1)*T)=norm(x_hat(:,t)-p_drone(:,t));  %Location error
            e2(ii,t+(jj-1)*T)=norm(x_hat2(:,t)-p_drone(:,t));  %Location error
        end;
        figure(ii)
        plot(x_hat(1,:),x_hat(2,:),'c.',x_hat2(1,:),x_hat2(2,:),'b.',p_drone(1,:),p_drone(2,:),'-',p_BS(1,:),p_BS(2,:),'o');
        legend('Estimated','drone position','BS location','BS location')
        hold on;
    end;
    %Calculate CDF of the error
    [FF,XX] = ecdf(e(ii,:));
    [FF2,XX2] = ecdf(e2(ii,:));
    figure(10)
    plot(XX,FF,XX2,FF2,'--')
    xlabel('Absolute error [m]')
    ylabel('CDF')
    hold on

end;

figure(10)
grid on
axis([0 100 0 1])

mean(e') %Mean error
sqrt(mean(e'.^2)) %RMS error
median(e') %Median error
