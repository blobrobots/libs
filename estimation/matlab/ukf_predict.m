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
% \file       ukf_predict.m
% \brief      function to predict state in a ukf filter
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

%% Unscented Kalman Filter prediction step:
% returns prior state estimate and covariance for nonlinear dynamic systems
% Noises are assumed as additive: x_k+1 = f(x_k) + w_k  where w ~ N(0,Q) 
% meaning w is gaussian noise with covariance Q
% in:   
%      dt: sample time
%       x: state estimate at time t-dt
%       P: estimated state covariance at time t-dt
%       f: function handle for f(x) - state prediction function
%       u: control input
%       Q: process noise covariance 
% out:  
%      x1: prior state estimate at time t
%      P1: prior state covariance at time t
%      X1: prior state unscented transformation at time t
%     X1s: prior state unscented transformation deviations at time t
function [x1,P1,X1,X1s] = ukf_predict(dt,x,P,f,u,Q)

L = numel(x);                        % number of states
alpha = 1;                           % default, tunable
ki = 0;                              % default, tunable
beta = 2;                            % default, tunable
lambda = alpha^2*(L+ki)-L;           % scaling factor
c = L+lambda;                        % scaling factor
Wm = [lambda/c 0.5/c+zeros(1,2*L)];  % weights for mean
Wc = Wm;
Wc(1) = Wc(1)+(1-alpha^2+beta);      % weights for covariance
c = sqrt(c);

fargs.u = u;
fargs.dt = dt;

X = sigmas(x,P,c);  % sigma points around x

[x1,X1,P1,X1s] = ut(f,fargs,X,Wm,Wc,L,Q); % unscented transformation of process

end
     
