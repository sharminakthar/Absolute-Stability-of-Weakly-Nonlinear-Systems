function [tmin,K,data] = stability_test_LMI(G,lambda_min,lambda_max)
%--------------------------------------------------------------------------
% 
% 12th September 2022
% 
% Scale independent stability test for network of agents
%   
%--------------------------------------------------------------------------

[A,B,C,D] = ssdata(G);
[np,m]    = size(B);

%% Set up LMIS
setlmis([])

% Specify the structure and size of LMI variables
P   = lmivar(1,[np,1]);
L   = lmivar(2,[m,np]);
W   = lmivar(1,[1,0]*m);

% First LMI
%first term A'P+PA
lmiterm ([1 1 1 P],1,A, 's');   
%second term PB
lmiterm ([1 1 2 P],1, B) ;  
lmiterm ([1 1 2 -L],lambda_min ,1); 
%third term B'P
lmiterm ([1 2 1 P],B',1) ; 
%fourth term -2W
lmiterm ([1 2 2 W],-2 ,1);  

% Second LMI
%first term A'P+PA
lmiterm ([2 1 1 P],1,A, 's');   
%second term PB+L'
lmiterm ([2 1 2 P],1, B) ;   
lmiterm ([2 1 2 -L],lambda_max ,1);   
%third term B'P+L
lmiterm ([2 2 1 P],B',1) ; 
lmiterm ([2 2 1 L],1, 1) ; 
%fourth term -2W
lmiterm ([2 2 2 W],-2 ,1);  

% Third LMI
lmiterm([3,1,1,P],-1,1);

LMISYS1 = getlmis;
% check the feasibility
[tmin,xfeas] = feasp(LMISYS1);

%% Recover decision variables and generate output data

Popt=dec2mat(LMISYS1, xfeas, P);
Lopt=dec2mat(LMISYS1, xfeas, L);
Wopt=dec2mat(LMISYS1, xfeas, W);
K=Lopt*inv(Wopt);

data.P = Popt;
data.L = Lopt;
data.W = Wopt;

end

