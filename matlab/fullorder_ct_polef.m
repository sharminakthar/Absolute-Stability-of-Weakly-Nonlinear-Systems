function [AW,gam] = fullorder_ct_polef(G,Wp,Wr,e1,e2,theta,epsilon);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%
%
% Script: fullorder_ct.m
%
% Author: MC Turner
%         Dept. of Engineering
%         University Of Leicester
%
% Date:  28th October 2004
% 
% Modifications: 19th March 2014 MCT
%                Small improvements
%
% Purpose: This script synthesises a full-order anti-windup compensator
%          for continuous time state-space systems according to the
%         decoupled architecture found in 
% 
%         "Accounting for uncertainty in anti-windup synthesis"
%         M.C. Turner, G. Herrmann and I. Postlethwaite, ACC 2004.
%
%
% Notes:  -   Written for MATLAB 6.5 (Linux)
%         -   Uses Matlab LMI toolbox
%         -   Prompts user to enter data for robustness/performance matrices
%             if variable `quiet'  does not exist.
%         -   Can essentially recover 'performance only' anti-windup if
%             robustness matrix is chosen small and performance matrix 
%             is chosen as identity
%
%         -   e1 - poles must lie to the left of -e1
%         -   e2 - poles must lie to the right of -e2
%         -   theta - minimum damping of poles
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Assumes plant data is:-
%
% xpdot = Ap xp + Bp u + Bpd d
%    yp = Cp xp + Dp u + Dpd d
%
% Assumes controller data is:-
%
% xcdot = Ac xc + Bc yp + Bcr r
%   u   = Cc xc + Dc yp + Dcr r
%
% Note that we do not need to consider the reference/disturbance parts
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[Ap,Bp,Cp,Dp]  =  ssdata(G);              % Extract state-space data - assumption 
                                          % that disturbance term is not in
                                          % ss-description
                                          
[np,m]         =  size(Bp);
p              =  size(Cp,1);

if nargin==6;
       epsilon = 1;                       % Set epsilon = (0,1] to one if not declared
                                          % (Corresponds to global
                                          % stability if plant is stable)
end 

sth         = sin(theta);
cth         = cos(theta);


%----------------------------------
%
% Create system of LMI's
%
%----------------------------------

setlmis([]);

F
gam = lmivar(1,[1,0]);

% First LMI

lmiterm([1,1,1,Q],1,Ap','s');
lmiterm([1,1,1,L],Bp,1,'s');
lmiterm([1,1,1,Q],1,e1,'s'); % Limit slow poles 
lmiterm([1,1,2,U],Bp,1);
lmiterm([1,1,2,-L],-1,epsilon);
lmiterm([1,1,3,0],zeros(np,m));
lmiterm([1,1,4,Q],1,Cp');
lmiterm([1,1,4,-L],1,Dp');
lmiterm([1,1,5,-L],1,1);
lmiterm([1,2,2,U],-1,1,'s');
lmiterm([1,2,3,0],eye(m)*epsilon);
lmiterm([1,2,4,U],1,Dp');
lmiterm([1,2,5,U],1,1);
lmiterm([1,3,3,gam],-1,1);
lmiterm([1,3,4,0],zeros(m,p));
lmiterm([1,3,5,0],-eye(m));
lmiterm([1,4,4,gam],1,-inv(Wp));
lmiterm([1,4,5,0],zeros(p,m));
lmiterm([1,5,5,gam],1,-inv(Wr));


% Second LMI

lmiterm([2,1,1,Q],-1,1);

% Third LMI: No fast poles

lmiterm([3,1,1,Q],-1,Ap','s');
lmiterm([3,1,1,L],-Bp,1,'s');
lmiterm([3,1,1,Q],1,-e2,'s'); % Limit fast poles 

% Damping limitation: 

lmiterm([4,1,1,Q],sth,Ap','s');
lmiterm([4,1,1,L],Bp,sth,'s');
lmiterm([4,1,2,Q],-cth,Ap');
lmiterm([4,1,2,Q],Ap,cth);
lmiterm([4,1,2,L],Bp,cth);
lmiterm([4,1,2,-L],-cth,Bp');
lmiterm([4,2,2,Q],sth,Ap','s');
lmiterm([4,2,2,L],Bp,sth,'s');






   LMISYS=getlmis;

   n = decnbr(LMISYS); % Obtains number of decision variables
   c = zeros(n,1); % Sets the dimensions of c accordingly

   for j=1:n
       [gamj]=defcx(LMISYS,j,gam);
       c(j) = [1 0]*gamj*[1 ; 0];
       c(j) = gamj;
   end


   %c=[zeros(1,n-1) 1];

   [copt,xopt]=mincx(LMISYS,c,[],[],[]);

% Transform decision variables into matrix form

gam =  dec2mat(LMISYS,xopt,gam);
Q   =  dec2mat(LMISYS,xopt,Q);
L   =  dec2mat(LMISYS,xopt,L);
U   =  dec2mat(LMISYS,xopt,U);

% Construct anti-windup compensator

F   =  L*inv(Q);
AW  =  ss(Ap+Bp*F,Bp,[F; Cp+Dp*F],[zeros(m,m); Dp]);
