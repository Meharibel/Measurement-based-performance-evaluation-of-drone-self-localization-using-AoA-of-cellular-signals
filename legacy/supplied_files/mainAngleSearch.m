%
% Search for the angle
%

% Mehari Meles
% 10.03.2021
clear all 
close all


dm = 0.05;
f1 = 3e9;
f2 = 3.03e9;

angleLow = 0;
angleUp = 179;

angleSearch = 0:0.02:180;


% load m30.mat
load m30.mat

[n,m,z] = size(m30);

% convert signal to abs value 

% i1 =1;

for i1 = 1:z
tmp = m30(:,:,i1);

angle1 = tmp(:,5);

ant2 = 10.^((tmp(:,2)-max(tmp(:,2)))/10);
ant1 = 10.^((tmp(:,3)-max(tmp(:,3)))/10);

%tmpL1 = feval("twoElemAntenna",angle1,angleLow,dm,f);
%tmpU1 = feval("twoElemAntenna",angle1,angleUp,dm,f);

opt1 = @(x) sum((ant1-feval("twoElemAntenna",angle1,x,dm,f1)).^2);
opt2 = @(x) sum((ant2-feval("twoElemAntenna",angle1,x,dm,f2)).^2);

v1 = opt1(angleSearch);
v2 = opt2(angleSearch);

[v,l] = min(v1);
foundA1(i1)= angleSearch(l);
[v,l] = min(v2);
foundA2(i1)= angleSearch(l);

% x = 0;
% foundAngle = fminsearch("opt1",x)
end

% plot outdoor data 
figure(1)
%hist(foundA1,[89:0.5:93])
hist(foundA1)
xlabel("angle")
print -dpdf firstBS_outdoorNoCrop.pdf
[status,cmdout] = system("pdfcrop firstBS_outdoorNoCrop.pdf firstBS_outdoor.pdf",'-echo')

figure(2)
% hist(foundA2,[114:0.5:119])
hist(foundA2)
xlabel("angle")
print -dpdf secondBS_outdoorNoCrop.pdf
[status,cmdout] = system("pdfcrop firstBS_outdoorNoCrop.pdf firstBS_outdoor.pdf",'-echo')

figure(3)
% hist(foundA2,[114:0.5:119])
hist(abs(foundA2-foundA1),[15:0.5:40])
xlabel("angle")
print -dpdf diff_outdoorNoCrop.pdf
[status,cmdout] = system("pdfcrop diff_outdoorNoCrop.pdf diff_outdoor.pdf",'-echo')
[status,cmdout] = system("rm *NoCrop.pdf",'-echo')