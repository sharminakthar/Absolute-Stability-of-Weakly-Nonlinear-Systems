%% Graph Theory code
% Adjacency matrix
Adj=[ 0 1 1 1; 1 0 0 0 ; 1 0 0 1; 1 0 1 0];
% Degree matrix
Deg= [ 3 0 0 0; 0 1 0 0 ; 0 0 2 0; 0 0 0 2];
% Laplacian matrix= Degree-Adjacency 
Lap= Deg-Adj;

[V,D,W]=eig(Lap);

%% State space system
A= [-2 1 0; 1 -1 1; 0 -2 0];
B= [0 ; 0 ; 3];
C= [1 0 0];
I=[1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];

% Define unknown matrix which need to be determined by LMI
setlmis([])
[np,m]         =  size(B);
p              =  size(C,1);

% Specify the structure and size of LMI variables
P   = lmivar(1,[np,1]);
L   = lmivar(2,[m,np]);
W   = lmivar(1,[1,0]*m);

%% First LMI
%first term A'P+PA
lmiterm ([1 1 1 P],1,A, 's');   
%second term PB+L'
lmiterm ([1 1 2 P],1, B) ;   
lmiterm ([1 1 2 -L],1 ,1);   
%third term B'P+L
lmiterm ([1 2 1 P],B',1) ; 
lmiterm ([1 2 1 L],1, 1) ; 
%fourth term -2W
lmiterm ([1 2 2 W],-2 ,1);  

%% Second LMI
lmiterm([2,1,1,P],-1,1);

% Finishing description
LMISYS = getlmis;

%% Check the feasibility
[tmin,xfeas] = feasp(LMISYS);

% LMI Solution
Popt=dec2mat (LMISYS, xfeas, P);

Lopt=dec2mat(LMISYS, xfeas, L);
Wopt=dec2mat(LMISYS, xfeas, W);

% Feedback gain vector K
K=Lopt*inv(Wopt);

%% MAS CL system dynamics
Anew= (kron(I,A));
Bnew=(kron(I,B));
Knew=(kron(Lap,K));
D=0;
sys=ss(Anew,Bnew,Knew,D)

% Agent initial position (random)
x0=rand(12,1);

t = 0:0.01:30;
u=zeros(size(t,2),size(Bnew,2)); 
                                 
[y,t,x]=lsim(sys, u, t, x0);

%% Plot
% first state component of all agents 
figure;
plot(t,x(:,1),'r');   
hold on
plot(t,x(:,4),'b');   
plot(t,x(:,7),'g');   
plot(t,x(:,10),'m'); 
grid on
title('1st state component')
hold off
% second state component of all agents 
figure;
plot(t,x(:,2),'r');  
hold on
plot(t,x(:,5),'b');
plot(t,x(:,8),'g');
plot(t,x(:,11),'m');
grid on
title('2nd state component')
hold off
% third state component of all agents 
figure;
plot(t,x(:,3),'r');
hold on
plot(t,x(:,6),'b');
plot(t,x(:,9),'g');
plot(t,x(:,12),'m');
grid on
title('3rd state component')
hold off