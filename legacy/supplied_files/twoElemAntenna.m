%
% function giving response of a two element antenna 
%

% Mehari Meles
% 10.03.2021

% in
%  aIn angles where direction is computed 
%  aRot rotation angle 
%  d distance between the elements 
%  f frequency 
% out 
%  gainOut estimated gains in given angles


function [gainOut] = twoElemAntenna(aIn, aRot,dm,f)

    d = 0.25;
    
    c = 299792458 ;
    % f = 3.03e9;
    % f = 3.0e9
    lambda = c./f;
    
    % dm = 0.050; % (0.052 - 0.048) /2
    dscale = (dm/(lambda/2));
    d = d*dscale;

  gainOut = 1/2*(exp(-1i*2*pi*(-d*sind(aIn-aRot))) + exp(-1i*2*pi*(d*sind(aIn-aRot))));
  gainOut = gainOut./max(gainOut);

end