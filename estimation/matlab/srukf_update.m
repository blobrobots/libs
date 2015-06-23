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
% \file       srukf_update.m
% \brief      function to update measurement in a square root ukf filter
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

%% Square Root UKF update step:
% returns posterior state estimate, x and state covariance, P for nonlinear 
% dynamic systems. For simplicity, noises are assumed as additive: 
% z_k = h(x_k) + v_k where v ~ N(0,R) meaning v is gaussian noise with 
% covariance R.
% in:
%      dt: sample time
%      x1: posterior state estimate at time t
%      S1: posterior state covariance square root at time t
%       h: function handle for h(x)
%       z: current measurement
%      Sr: measurement noise covariance square root
% out:   
%      x2: posterior state estimate at time t
%      S2: posterior state covariance square root at time t
function [x2,S2] = srukf_update(dt,x1,S1,h,z,Sr)

L = numel(x1);                      % number of states
N = 2*L+1;                          % number of sigma points
M = numel(z);                       % number of measurements
alpha = 1;                          % default, tunable
ki = 0;                             % default, tunable
beta = 2;                           % default, tunable
lambda = alpha^2*(L+ki)-L;          % scaling factor
c = L+lambda;                       % scaling factor
Wm = [lambda/c 0.5/c+zeros(1,2*L)]; % weights for mean
Wc = Wm;
Wc(1) = Wc(1)+(1-alpha^2+beta);     % weights for covariance
c = sqrt(c);

hargs.dt=dt; % number of measurements

x1_ = x1(:,ones(1,L));              % x_ = [x x x ... x]
X1  = [x1, (x1_+c*S1), (x1_-c*S1)]; % prior sigma points
X1s = X1 - x1(:,ones(1,N));         % prior std dev

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

Sz = cholupdate(Sz, sqrt(abs(Wc(1)))*Z1s(:,1),csign);
Sz = Sz';

% transformed cross-cov. chol.
Pxz = X1s*diag(Wc)*Z1s'; 
K = (Pxz/Sz')/Sz;
U = K*Sz;

% posterior state update
x2 = x1 + K*(z - z1);    

% posterior cov. chol. update (needs upper triangular)
S2 = S1';
for i=1:size(U,2)
  S2 = cholupdate(S2,U(:,i),'-');  
end
S2 = S2';

end
