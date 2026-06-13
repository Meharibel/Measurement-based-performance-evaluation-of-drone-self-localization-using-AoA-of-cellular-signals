%Angle of Arrival based localization of a drone
%10.5.2021
clear 
close all
%Base station position
ISD=500;    %Inter site distance
bs_pos=[0 2*ISD*exp(i*[1:6]*60/180*pi)];    %Base station positions as complex numbers
No_BS=length(bs_pos);
p_BS= [real(bs_pos); imag(bs_pos)];         %Base station positions[x y]
% plot(p_BS)
%Initial posion
%Drone positions as complex numer
drone_pos=(ISD/2*[0:.05:2]).*exp(1i*[0:.05:2]*2*pi)*exp(-i*0.333*2*pi);
%Drone position [x y]
p_drone= [real(drone_pos); imag(drone_pos)];
p_drone= p_drone + 1*randn(size(p_drone)); 
T=length(p_drone);
% 2 3 was before
std_angle=[6.02 3.2]/180*pi;
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
                %Scaled BS location
                h(k,1)=[-tau(k,t) 1]*p_BS(:,k);
                %if abs(tau(k,t))>th
                %    tau(k,t)=0;
                %    h(k,1)=0;
                %end;
            end;
            G=[ -tau(:,t) ones(size(tau(:,t)))];  %G matrix
            x_hat(:,t)=pinv(G)*h; %Linear 'measurement'   
            e(ii,t+(jj-1)*T)=norm(x_hat(:,t)-p_drone(:,t));  %Location error
        end;
    figure(ii)
    set(gca,'FontSize',17)
    plot(x_hat(1,:),x_hat(2,:),'c.',p_drone(1,:),p_drone(2,:),'-',p_BS(1,:),p_BS(2,:),'*');
  legend('Estimated location of the drone','True location of the drone','BS location')
    hold on;
    end;
    %Calculate CDF of the error
    [FF,XX] = ecdf(e(ii,:));
    figure(10)
   set(gca,'FontSize',17)
    plot(XX,FF)
    xlabel('Absolute error [m]')
    ylabel('CDF')
    hold on   
end;

figure(10)
set(gca,'FontSize',17)
grid on
axis([0 100 0 1])
mean(e') %Mean error
median(e') %Median error
%% Updated2 code
%Angle of Arrival based localization of a drone
%10.5.2021
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
std_angle=[12 3.6]/180*pi;
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


%%updated3 Drone AoA sensitivity

clear

d=0.45; %Distance between antennas in wavelengths

vphi=[0:.1:1]'*pi; %Array rotation
stdx=0.4;           %Location uncertainity of the drone in x
stdy=0.4;           %Location uncertainity of the drone in y
x=10;               %Initial location of the drone in x
y=15;               %Initial location of the drone in y
theta0=atan2(y,x);  %Initial angle
SNR=1000;             %Signal to noise ratio
stdn=1/sqrt(SNR);   %Standard deviation of the noise
stdvphi=3/180*pi;   %Standard deviation of the drone + array orientation error

%Simulate antenna rotation and drone movement
for k=1:length(vphi)
    theta(k,1)=atan2(y(k),x(k));
    Gmeas(k,1)=1/2*abs(1+exp(i*2*pi*d*cos(theta(k)-vphi(k)-randn*stdvphi))+stdn/sqrt(2)*(randn+1i*randn)); %Simulated measured antenna pattern
    %Brownian motion 
    x(k+1)=x(k)+randn*stdx; 
    y(k+1)=y(k)+randn*stdy;
end;
Gmeas=Gmeas/max(Gmeas); %Power normalization


%Angle to the mean direction
theta_avg=atan2(mean(y),mean(x));

%Antenna pattern
Gideal=1/2*(1+cos(2*pi*d*cos(theta_avg-vphi)));

figure(1)
plot(vphi*180/pi,10*log10(Gideal),vphi*180/pi,10*log10(Gmeas))
xlabel('Array rotation angle')
ylabel('Normalized power')
legend('Ideal antenna pattern', 'Measured antenna response','Location','Southeast')

figure(2)
plot(x,y,mean(x),mean(y),'x')
xlabel('x [m]')
ylabel('y [m]')




theta_aug=[0:.001:1]*pi;
for k=1:length(theta_aug)
    %Antenna pattern with angle theta_aug(k)
    G_aug=1/2*(1+cos(2*pi*d*cos(theta_aug(k)-vphi)));
    Corr(k)=log(G_aug)'*log(Gmeas); %Correlator
end;    
%Correlator output
[Cmax,Cindex]=max(Corr);
theta_hat=theta_aug(Cindex);

%Starting angle, mean angle and estimated angle)
[theta0, theta_avg theta_hat]*180/pi
AoA_error=theta_hat-theta_avg;
AoA_error*180/pi

figure(3)
plot(theta_aug*180/pi,Corr)
xlabel('Angle')
ylabel('Correlator output')

%Estimated antenna pattern vs measured
Gest=1/2*(1+cos(2*pi*d*cos(theta_hat-vphi)));
figure(4)
plot(vphi*180/pi,10*log10(Gest),vphi*180/pi,10*log10(Gmeas))
xlabel('Array rotation angle')
ylabel('Normalized power')
legend('Ideal antenna pattern', 'Measured antenna response','Location','Southeast')

%%Updated4


clear
No_real=100;        %Number of realizations
f=3.5*1e9;          %Carrier frequency
da=0.5;             %Distance between antennas in wavelengths
c=3e8;             %Speed of light m/s
vphi=[0:.1:1]'*pi; %Array rotation
stdx=0.5;           %Location uncertainity of the drone in x
stdy=0.5;           %Location uncertainity of the drone in y
stdz=0.3;          %Location uncertainity of the drone in z
x=10;               %Initial location of the drone in x
y=15;               %Initial location of the drone in y
z=3;                %Drone height
h=1.2;                %Tx antenna height
theta0=atan2(y,x);  %Initial angle
SNR=1000;             %Signal to noise ratio
stdn=1/sqrt(SNR);   %Standard deviation of the noise
stdvphi=6.5/180*pi;   %Standard deviation of the drone + array orientation error
bias=0.0;          %Phase shift between antennas
GRL=1;            %Reflection loss

%Simulate antenna rotation and drone movement
for ii=1:No_real;
    for k=1:length(vphi)
        theta(k,1)=atan2(y(k),x(k));
        d(k)=sqrt(x(k)^2+y(k)^2+(z(k)-h)^2);
        d1(k)=sqrt(d(k)^2/4+h^2);
        d2(k)=sqrt(d(k)^2/4+z(k)^2);
        %Phase shift between antennas, ground reflection and noise effects
        Gmeas(k,1)=1/2*abs((1+exp(1i*2*pi*da*cos(theta(k)-vphi(k)-randn*stdvphi)+1i*bias))*(exp(-1i*2*pi*f*d(k)/c)-GRL*d(k)^2/(d1(k)+d2(k))^2*exp(-1i*2*pi*f*(d1(k)+d2(k))/c))+stdn/sqrt(2*(1+GRL))*(randn+1i*randn)); %Simulated measured antenna pattern
        %Brownian motion 
        x(k+1)=x(k)+randn*stdx; 
        y(k+1)=y(k)+randn*stdy;
        z(k+1)=z(k)+randn*stdz;
    end;
    Gmeas=Gmeas/max(Gmeas); %Power normalization
    %Angle to the mean direction
    theta_avg=atan2(mean(y),mean(x));
    %Antenna pattern
    Gideal=1/2*(1+cos(2*pi*da*cos(theta_avg-vphi)));
    theta_aug=[0:.001:1]*pi;
    for k=1:length(theta_aug)
        %Antenna pattern with angle theta_aug(k)
        G_aug=1/2*(1+cos(2*pi*da*cos(theta_aug(k)-vphi)));
        Corr(k)=log(G_aug)'*log(Gmeas); %Correlator
    end;    
    %Correlator output
    [Cmax,Cindex]=max(Corr);
    theta_hat=theta_aug(Cindex);

    %Starting angle, mean angle and estimated angle)
   % disp([theta0, theta_avg theta_hat]*180/pi)
    AoA_error(ii)=theta_hat-theta_avg;
end;
[F,X] = ecdf(abs(AoA_error));

figure(1)
plot(X*180/pi,F)
xlabel('Absolute AoA error')
ylabel('CDF')
mean_AoA_error=mean(AoA_error)*180/pi
std_AoA_error=std(AoA_error)*180/pi


if 1
    %Plot single realization;
    figure(2)
    plot(vphi*180/pi,10*log10(Gideal),vphi*180/pi,10*log10(Gmeas))
    xlabel('Array rotation angle')
    ylabel('Normalized power')
    legend('Ideal antenna pattern', 'Measured antenna response','Location','Southeast')

    figure(3)
    plot(x,y,mean(x),mean(y),'x')
    xlabel('x [m]')
    ylabel('y [m]')

    figure(4)
    plot(theta_aug*180/pi,Corr)
    xlabel('Angle')
    ylabel('Correlator output')

    %Estimated antenna pattern vs measured
    Gest=1/2*(1+cos(2*pi*da*cos(theta_hat-vphi)));
    figure(4)
    plot(vphi*180/pi,10*log10(Gest),vphi*180/pi,10*log10(Gmeas))
    xlabel('Array rotation angle')
    ylabel('Normalized power')
    legend('Ideal antenna pattern', 'Measured antenna response','Location','Southeast')
end;