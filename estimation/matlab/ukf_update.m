%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The MIT License (MIT)
%
% Copyright (c) 2015 Blob Robotics
%
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal 
% in the Software without restriction, including without limitation the rights 
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is 
% furnished to do so, subject to the following conditions:
% 
% The above copyright notice and this permission notice shall be included in
% all copies or substantial portions of the Software.
% 
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE 
% SOFTWARE.
% 
% \file       ukf_update.m
% \brief      function to update measurement in a ukf filter
% \author     adrian jimenez-gonzalez (blob.robots@gmail.com)
% \copyright  the MIT License Copyright (c) 2015 Blob Robots.
%
% Reference: Julier, SJ. and Uhlmann, J.K., Unscented Filtering and
% Nonlinear Estimation, Proceedings of the IEEE, Vol. 92, No. 3,
% pp.401-422, 2004.
%
% Inspired on initial script by Yi Cao at Cranfield University, 04/01/2008
% http://es.mathworks.com/matlabcentral/fileexchange/18217-learning-the-unscented-kalman-filter/content/ukf.m
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Unscented Kalman Filter update step:
% returns posterior state estimate, x and state covariance, P for nonlinear 
% dynamic systems. For simplicity, noises are assumed as additive: 
% z_k = h(x_k) + v_k where v ~ N(0,R) meaning v is gaussian noise with 
% covariance R.
% in:
%      dt: sample time
%      x1: prior state at time t
%      P1: priot state covariance at time t
%       h: function handle for h(x) - expected measurement function
%       z: sensor measurement
%       R: process noise covariance 
%      X1: prior state sigma points
%     X1s: prior state sigma points std dev.
% out:
%      x2:  posterior state
%      P2:  posterior state covariance
function [x2,P2] = ukf_update(dt,x1,P1,h,z,R,X1,X1s)

L = numel(x1);                      % numer of states
N = 2*L+1;                          % number of sigma points
M = numel(z);                       % numer of measurements
alpha = 1;                          % default, tunable
ki = 0;                             % default, tunable
beta = 2;                           % default, tunable
lambda = alpha^2*(L+ki)-L;          % scaling factor
c = L+lambda;                       % scaling factor
Wm = [lambda/c 0.5/c+zeros(1,2*L)]; % weights for means
Wc = Wm;
Wc(1) = Wc(1)+(1-alpha^2+beta);     % weights for covariance
c = sqrt(c);

hargs.dt=dt;                        % number of measurements

if((exist('X') ~= 1)||(exist('Xs') ~= 1)||(isempty(X))||(isempty(Xs)))
   X1 = sigmas(x1,P1,c);      % sigma points around prior x1
   X1s = X1-x1(:,ones(1,N));  % deviation of prior X1
end

[z1,Z1,Pz,Z1s] = ut(h, hargs, X1, Wm, Wc, M, R); % unscented transformation of measurments
Pxz = X1s*diag(Wc)*Z1s';                         % transformed cross-covariance
K = Pxz/Pz;                     

x2 = x1 + K*(z - z1); % state update
P2 = P1 - K*Pxz';     % covariance update
