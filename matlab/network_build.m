%% Script to build simple networked system
%
%  12th September 2022  
%  
%  Based on Sharmin's original file
%--------------------------------------------------------------------------

%% Network 
% Adjacency matrix
Adj =[ 0 1 1 1; 1 0 0 0 ; 1 0 0 1; 1 0 1 0];
% Degree matrix
Deg = [ 3 0 0 0; 0 1 0 0 ; 0 0 2 0; 0 0 0 2];
% Laplacian matrix= Degree-Adjacency 
Lap = Deg-Adj;

%% State space realisation of agents
A = [-2 1 0; 1 -1 1; 0 -2 0];
B = [0 ; 0 ; 3];
C = [1 0 0];
G = ss(A,B,C,0);  % state-space object of agent

%% Generate stabilising state-feedback

eig_Lap    = eig(Lap);
lambda_min = 0 % (min eigenvalue of Laplacian)
lambda_max = max(eig_Lap) % eigenvalues always >= 0

[tmin,K,data] = stability_test_LMI(G,lambda_min,lambda_max);

%% Test stability for all agents

for id=1:size(Lap,1)
    Mbig{id} = [A'*data.P+data.P*A data.P*B+eig_Lap(id)*K';
            (data.P*B+eig_Lap(id)*K')' -2*data.W];
    if max(eig(Mbig{id})) < 0
        disp(['Agent ' num2str(id) ' stable']);
    end
end

%% Simulations

Anew = (kron(eye(4),A));
Bnew = (kron(eye(4),B));
Knew = (kron(Lap,K));

% Agent initial position (random)
x0=rand(12,1);

% Stop time of simulation
tstop = 20;

sim('network_sim');

%% Plot
x = ans.x;
% first state component of all agents 
figure;
plot(x.time,x.signals.values(:,1),'r');   
hold on
plot(x.time,x.signals.values(:,4),'b');   
plot(x.time,x.signals.values(:,7),'g');   
plot(x.time,x.signals.values(:,10),'m'); 
grid on
title('1st state component')
hold off
% second state component of all agents 
figure;
plot(x.time,x.signals.values(:,2),'r');  
hold on
plot(x.time,x.signals.values(:,5),'b');
plot(x.time,x.signals.values(:,8),'g');
plot(x.time,x.signals.values(:,11),'m');
grid on
title('2nd state component')
hold off
% third state component of all agents 
figure;
plot(x.time,x.signals.values(:,3),'r');
hold on
plot(x.time,x.signals.values(:,6),'b');
plot(x.time,x.signals.values(:,9),'g');
plot(x.time,x.signals.values(:,12),'m');
grid on
title('3rd state component')
hold off

