%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%
% Script: BTTmissile_bld.m
%
% Author: MC Turner
%         Dept. of Engineering
%         Uni. of Leicester
%
% Date: 26th October 2004
%
% Modified: 19th March 2014 MCT
%           Tidied up a bit
%
%           25th May 2017 MCT 
%           Pole placement constraints added   
%
% Purpose: To build a BTT missile example. Static anti-windup 
%          compensation is feasible
%
%          This version assumes an LQG/LTR controller
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Construct plant 


Ap  =  [-0.818   -0.999    0.349;
         80.29   -0.579    0.009;
        -2734     0.05621 -2.10];

Bp  =  [0.147    0.012;
       -194.4    37.61;
        -2716   -1093];

Cp  =  [1   0   0;
        0   1   0];

Dp  =  zeros(2,2);

Bpd =  zeros(3,2);

Dpd =  zeros(2,2);

% Build LQG/LTR controller

A1  =  [-0.29   -107.8    6.67   -2.58   -0.4;
        107.68  -97.81   63.95   -4.52   -5.35;
         -6.72   64.82  -54.19  -40.79    5.11;
          3.21    2.1    29.56 -631.15  429.89;
          0.36   -3.39    3.09 -460.03   -0.74];

B1  =  [2.28   0.48;
      -40.75   2.13;
       18.47  -0.22;
       -2.07 -44.68;
       -0.98  -1.18];

Ac  =  [A1 B1; zeros(2,5) zeros(2,2)];

Bc  = -[zeros(5,2) ; eye(2)];

Bcr = -Bc;

Dc  = -zeros(2,2);

Dcr = -Dc;

Cc =  [0.86 8.54 -1.71 43.91 1.12 0 0;
       2.17 39.91 -18.39 -8.51 1.03 0 0];

% Control limit   

ubar  =  8;

%--------------------------------------------------------------------------

% Design Anti-windup compensators:

%--------------------------------
%
% Design full-order AW compensator
%
%--------------------------------

disp(' ');
disp('....Designing full-order AW compensator....');
disp(' ');

G   =   ss(Ap,Bp,Cp,Dp);   % Plant without disturbance inputs
Wp  =   eye(2);            % Set at identity is a good start
Wr  =   100*eye(2);        % Robustness weight: increasing tends to give smaller poles
                           % Set to eps*I to begin with (eps <<1)
                           
[AWfull,gam] = fullorder_ctf(G,Wp,Wr); 


%--------------------------------
%
% Design full-order AW compensator with pole placement constraints
%
%--------------------------------

disp(' ');
disp('....Designing full-order AW compensator....with pole-placement constraints');
disp(' ');

G   =   ss(Ap,Bp,Cp,Dp);   % Plant without disturbance inputs
Wp  =   eye(2);            % Set at identity is a good start
Wr  =   1*eye(2);          % Robustness weight: increasing tends to give smaller poles
                           % Set to eps*I to begin with (eps <<1)
                           
e1  =  .001;               % Real part of pole must be less than -e1  
e2  =  2000;               % Real part of pole must be greater than -e2
theta = (pi/2)*0.6;        % Constraint on damping ratio (pi/2 = no constraint)
                           
[AWfull2,gam2] = fullorder_ct_polef(G,Wp,Wr,e1,e2,theta); 



%--------------------------------------------------------------------------

% Setup simulations

tstop = 10;    % Stop time
tstep = 0.001  % Time-step for solver

[np,m]  = size(Bp);
[np,nd] = size(Bpd);
[nc,p]  = size(Bc);
[nc,nr] = size(Bcr);

ref_in = [0 1 1.05 5 5.05 10;
          [0 0 1 1 0 0]*4.2;
          [0 0 1 1 0 0]*-4.2]';  % Reference sequence
      
ref_in = [0 0.05  3 3.05 6 6.05 8 8.05 10;
          [0  1 1 -1 -1 1 1 0 0]*4.2;
          [0  1 1 -1 -1 1 1 0 0]*-6.2]';  % Reference sequence
      


% Simulate linear model
baru   = inf;
AW     = ss([],zeros(0,m),zeros(m+p,0),zeros(m+p,m));
     
sim('genericAW');
      
figure(1);
subplot(211);
plot(y.time,y.signals.values,'b');
ylabel('Output response [deg]');
title('BTT Missile simulation');
hold on;
subplot(212);
ul=plot(um.time,um.signals.values,'b');
ylabel('Control response [deg]');
xlabel('Time [sec]');
hold on;


% Simulate saturated model -no AW

baru   = ubar;
AW     = ss([],zeros(0,m),zeros(m+p,0),zeros(m+p,m));
     
sim('genericAW');
      
figure(1);
subplot(211);
plot(y.time,y.signals.values,'r');
subplot(212);
us=plot(um.time,um.signals.values,'r');


% Simulate saturated model - with full-order AW

baru   = ubar;
AW     = AWfull;
     
sim('genericAW');
      
figure(1);
subplot(211);
plot(y.time,y.signals.values,'g');
subplot(212);
uaw=plot(um.time,um.signals.values,'g');     



% Simulate saturated model - with full-order AW

baru   = ubar;
AW     = AWfull2;
     
sim('genericAW');
      
figure(1);
subplot(211);
plot(y.time,y.signals.values,'k');
subplot(212);
uaw2=plot(um.time,um.signals.values,'k');  

legend([ul(1),us(1),uaw(1),uaw2(1)],'Linear','Sat w/o AW','Sat w/AW','Sat w/AW pole constraints');

hline = findobj(gcf, 'type', 'line');
set(hline,'LineWidth',2)



