%% ukf_update.m
%  author: adrian jimenez gonzalez
%  email:  blob.robotics@gmail.com
%  date:   15-jan-2015
%  brief:  function to update measurement in a ukf filter

function [x,P] = ukf_update(dt,x,P,h,z,R,X,Xs)
% Unscented Kalman Filter - UKF 
% Update step:
% [x, P] = ukf_update(dt,x,P,h,z,Q,R) returns posterior state estimate, x
% and state covariance, P for nonlinear dynamic systems. For simplicity, 
% noises are assumed as additive:
%           z_k   = h(x_k) + v_k
% where v ~ N(0,R) meaning v is gaussian noise with covariance R
% Inputs:   dt: time step
%           h:  function handle for h(x)
%           z:  current measurement
%           Q:  process noise covariance 
%           R:  measurement noise covariance
%           X:  prior unscented transformation
%           Xs: prior unscented transformation standard deviation
% Output:   x:  posterior state estimate at time t
%           P:  posterior state covariance at time t
%
% Reference: Julier, SJ. and Uhlmann, J.K., Unscented Filtering and
% Nonlinear Estimation, Proceedings of the IEEE, Vol. 92, No. 3,
% pp.401-422, 2004. 
%
% Inspired on initial script by Yi Cao at Cranfield University, 04/01/2008


L = numel(x);                                   %numer of states
alpha = 1e-3;                                   %default, tunable
ki = 0;                                         %default, tunable
beta = 2;                                       %default, tunable
lambda = alpha^2*(L+ki)-L;                      %scaling factor
c = L+lambda;                                   %scaling factor
Wm = [lambda/c 0.5/c+zeros(1,2*L)];             %weights for means
Wc = Wm;
Wc(1) = Wc(1)+(1-alpha^2+beta);                 %weights for covariance
c = sqrt(c);

m=numel(z);
hargs.dt=dt;                                    %number of measurements

if((exist('X') ~= 1)||(exist('Xs') ~= 1)||(isempty(X))||(isempty(Xs)))
   X = sigmas(x,P,c);                            %sigma points around x1
   Xs = X-x(:,ones(1,size(X,2)));                %deviation of X1
end

[z1,Z1,P2,Z2] = ut(h, hargs, X, Wm, Wc, m, R);  %unscented transformation of measurments
P12 = Xs*diag(Wc)*Z2';                          %transformed cross-covariance
K = P12*inv(P2);                                

x = x + K*(z - z1);                              %state update
P = P - K*P12';                                  %covariance update
