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
% \file       srukf_predict.m
% \brief      function to predict state in a square root ukf filter
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

%% Square Root UKF prediction step:
% returns posterior state estimate, x and state covariance, P for nonlinear 
% dynamic systems. For simplicity, noises are assumed as additive: 
% z_k = h(x_k) + v_k where v ~ N(0,R) meaning v is gaussian noise with 
% covariance R.
% in: 
%     dt: sample time
%      x: state estimate at time t-dt
%      S: estimated state covariance at time t-dt
%      f: function handle for f(x)
%      u: control input
%     Sq: sqrt of process noise covariance 
% out:   
%     x1: prior state estimate at time t
%     S1: prior state covariance at time t
function [x1,S1] = srukf_predict(dt,x,S,f,u,Sq)

L = numel(x);                       % number of states
N = 2*L+1;                          % number of sigma points
alpha = 1;                          % default, tunable
ki = 0;                             % default, tunable
beta = 2;                           % default, tunable
lambda = alpha^2*(L+ki)-L;          % scaling factor
c = L+lambda;                       % scaling factor
Wm = [lambda/c 0.5/c+zeros(1,2*L)]; % weights for mean
Wc = Wm;
Wc(1) = Wc(1)+(1-alpha^2+beta);     % weights for covariance
c = sqrt(c);

fargs.u = u;
fargs.dt = dt;

x_ = x(:,ones(1,L));          % x_ = [x x x ... x]
X  = [x, (x_+c*S), (x_-c*S)]; % state sigma points

X1 = zeros(L,N); % prior sigma points
x1 = zeros(L,1); % prior state

for i = 1:N
  X1(:,i) = f(X(:,i),fargs); % apply f() to calculate prior sigma points
  x1 = x1 + Wm(i)*X1(:,i);   % prior state is ponderated from sigma points
end

X1s = X1 - x1(:,ones(1,N));    % prior chol. cov. sigma points

[Q, S1] = qr([sqrt(abs(Wc(2)))*X1s(:,2:N) Sq]', 0);

csign='+';
if(Wc(1)<0)
  csign='-';
end

% S1 is upper triangular
S1 = cholupdate(S1, sqrt(abs(Wc(1)))*X1s(:,1),csign);
S1=S1'; % we work with lower triangular

end
     
