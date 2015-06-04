%% srukf_predict.m
%  author: adrian jimenez gonzalez
%  email:  blob.robotics@gmail.com
%  date:   15-jan-2015
%  brief:  function to update measurement in a ukf filter

function [x1,S1]=srukf_predict(dt,x,S,f,u,Sq)
% Square Root - Unscented Kalman Filter - UKF 
% Prediction step:
% [x, P] = ukf_predict(f,x,P) returns a prior state estimate, x and state 
% covariance, P for nonlinear dynamic systems. For simplicity, noises are 
% assumed as additive):
%           x_k+1 = f(x_k) + w_k
% where w ~ N(0,Q) meaning w is gaussian noise with covariance Q
% Inputs:   dt: time step
%           f:  function handle for f(x)
%           x:  state estimate at time t-dt
%           P:  estimated state covariance at time t-dt
%           u:  control input
%           Sq:  sqrt of process noise covariance 
% Output:   x:  prior state estimate at time t
%           P:  prior state covariance at time t
%
% Reference: Julier, SJ. and Uhlmann, J.K., Unscented Filtering and
% Nonlinear Estimation, Proceedings of the IEEE, Vol. 92, No. 3,
% pp.401-422, 2004. 
%
% Inspired on initial script by Yi Cao at Cranfield University, 04/01/2008
% http://es.mathworks.com/matlabcentral/fileexchange/18217-learning-the-unscented-kalman-filter/content/ukf.m

L = numel(x);                                 % number of states
N = 2*L+1;                                    % number of sigma points
alpha = 1e-3;                                 % default, tunable
ki = 0;                                       % default, tunable
beta = 2;                                     % default, tunable
lambda = alpha^2*(L+ki)-L;                    % scaling factor
c = L+lambda;                                 % scaling factor
Wm = [lambda/c 0.5/c+zeros(1,2*L)];           % weights for means
Wc = Wm;
Wc(1) = Wc(1)+(1-alpha^2+beta);               % weights for covariance
c = sqrt(c);

fargs.u = u;
fargs.dt = dt;

x_ = x(:,ones(1,L));          % x_ = [x x x ... x]
X  = [x, (x_+c*S), (x_-c*S)]; % state sigma points

X1 = zeros(L,N);      % prior sigma points
x1 = zeros(L,1);              % prior state

for i = 1:N
    X1(:,i) = f(X(:,i),fargs); % apply f() to calculate prior sigma points
    x1 = x1 + Wm(i)*X1(:,i);   % prior state is ponderated from sigma points
end

X1s = X1 - x1(:,ones(1,N));    % prior std dev sigma points

[Q, S1] = qr([sqrt(abs(Wc(2)))*X1s(:,2:N) Sq]', 0);

csign='+';
if(Wc(1)<0)
  csign='-';
end
x1
S1
X1
X1s
Wm
Wc
[sqrt(abs(Wc(2)))*X1s(:,2:N) Sq]'
csign
sqrt(abs(Wc(1)))*X1s(:,1)
% S1 is upper triangular
S1 = cholupdate(S1, sqrt(abs(Wc(1)))*X1s(:,1),csign);
S1=S1'; % we work with lower triangular
S1
pause()
end
     
