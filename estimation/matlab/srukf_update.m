%% srukf_update.m
%  author: adrian jimenez gonzalez
%  email:  blob.robotics@gmail.com
%  date:   15-jan-2015
%  brief:  function to update measurement in a ukf filter

function [x2,S2] = srukf_update(dt,x1,S1,h,z,Sr)
% Unscented Kalman Filter - UKF 
% Update step:
% [x, P] = ukf_update(dt,x,P,h,z,Q,R) returns posterior state estimate, x
% and state covariance, P for nonlinear dynamic systems. For simplicity, 
% noises are assumed as additive:
%           z_k   = h(x_k) + v_k
% where v ~ N(0,R) meaning v is gaussian noise with covariance R
% Inputs:   dt: time step
%           x:  posterior state estimate at time t
%           S:  posterior state covariance square root at time t
%           h:  function handle for h(x)
%           z:  current measurement
%           Sr:  measurement noise covariance square root
%           X:  prior unscented transformation
%           Xs: prior unscented transformation standard deviation
% Output:   x:  posterior state estimate at time t
%           S:  posterior state covariance square root at time t
%
% Reference: Julier, SJ. and Uhlmann, J.K., Unscented Filtering and
% Nonlinear Estimation, Proceedings of the IEEE, Vol. 92, No. 3,
% pp.401-422, 2004. 
%
% Inspired on initial script by Yi Cao at Cranfield University, 04/01/2008
% http://es.mathworks.com/matlabcentral/fileexchange/18217-learning-the-unscented-kalman-filter/content/ukf.m

L = numel(x1);                                   %number of states
N = 2*L+1;                                      %number of sigma points
M = numel(z);                                   %number of measurements
alpha = 1e-3;                                   %default, tunable
ki = 0;                                         %default, tunable
beta = 2;                                       %default, tunable
lambda = alpha^2*(L+ki)-L;                      %scaling factor
c = L+lambda;                                   %scaling factor
Wm = [lambda/c 0.5/c+zeros(1,2*L)];             %weights for means
Wc = Wm;
Wc(1) = Wc(1)+(1-alpha^2+beta);                 %weights for covariance
c = sqrt(c);

hargs.dt=dt;                                    %number of measurements

%if((exist('X1') ~= 1)||(isempty(X1)))
x1_ = x1(:,ones(1,L));              % x_ = [x x x ... x]
X1  = [x1, (x1_+c*S1), (x1_-c*S1)]; % prior sigma points
X1s = X1 - x1(:,ones(1,N));         % prior std dev
%end

Z1 = zeros(M,N);      % predicted measurement sigma points
z1 = zeros(M,1);      % predicted measurement

for i = 1:N
    Z1(:,i) = h(X1(:,i),hargs); % apply h() to calculate predicted measurement sigma points
    z1 = z1 + Wm(i)*Z1(:,i);    % predicted measurement is ponderated from sigma points
end

Z1s = Z1 - z1(:,ones(1,N));

[Q, Sz] = qr([sqrt(abs(Wc(2)))*Z1s(:,2:N) Sr]', 0);

csign='+';
if(Wc(1)<0)
  csign='-';
end

x1
S1
X1
X1s

z1
Z1
Z1s

[sqrt(abs(Wc(2)))*Z1s(:,2:N) Sr]'
Sz

Sz = cholupdate(Sz, sqrt(abs(Wc(1)))*Z1s(:,1),csign);
Sz = Sz' %needed?

Pxz = X1s*diag(Wc)*Z1s' %transformed cross-stddev
%K = (Pxz/Sz')/Sz; % inv?
%U = K*Sz;
U = Pxz*inv(Sz')
K = U*inv(Sz)

x2 = x1 + K*(z - z1)  % posterior state update

S2=S1' % posterior stddev (needs upper triangular)

for i=1:size(U,2)
    S2 = cholupdate(S2,U(:,i),'-');  % posterior stddev update
end
S2 = S2'
pause()
end
