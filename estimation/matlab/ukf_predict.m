%% ukf_predict.m
%  author: adrian jimenez gonzalez
%  email:  blob.robotics@gmail.com
%  date:   15-jan-2015
%  brief:  function to update measurement in a ukf filter

function [x1,P1,X1,X1s]=ukf_predict(dt,x,P,f,u,Q)
% Unscented Kalman Filter - UKF 
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
%           Q:  process noise covariance 
% Output:   x:  prior state estimate at time t
%           P:  prior state covariance at time t
%
% Reference: Julier, SJ. and Uhlmann, J.K., Unscented Filtering and
% Nonlinear Estimation, Proceedings of the IEEE, Vol. 92, No. 3,
% pp.401-422, 2004. 
%
% Inspired on initial script by Yi Cao at Cranfield University, 04/01/2008
% http://es.mathworks.com/matlabcentral/fileexchange/18217-learning-the-unscented-kalman-filter/content/ukf.m

L = numel(x);                                 %number of states
alpha = 1e-3;                                 %default, tunable
ki = 0;                                       %default, tunable
beta = 2;                                     %default, tunable
lambda = alpha^2*(L+ki)-L;                    %scaling factor
c = L+lambda;                                 %scaling factor
Wm = [lambda/c 0.5/c+zeros(1,2*L)];           %weights for means
Wc = Wm;
Wc(1) = Wc(1)+(1-alpha^2+beta);               %weights for covariance
c = sqrt(c);

fargs.u = u;
fargs.dt = dt;

X = sigmas(x,P,c);                         %sigma points around x

[x1,X1,P1,X1s] = ut(f,fargs,X,Wm,Wc,L,Q);  %unscented transformation of process

end
     
