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
% \file       ut.m
% \brief      function to perform ukf unscented transformation
% \author     adrian jimenez-gonzalez (blob.robots@gmail.com)
% \copyright  the MIT License Copyright (c) 2015 Blob Robots.
%
% Inspired on initial script by Yi Cao at Cranfield University, 04/01/2008
% http://es.mathworks.com/matlabcentral/fileexchange/18217-learning-the-unscented-kalman-filter/content/ukf.m
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% function to perform ukf unscented transformation
% in:
%     func: nonlinear mapping function
%    fargs: function arguments
%        X: state sigma points
%       Wm: weights for mean
%       Wc: weights for covariance
%        m: numer of outputs of f
%        R: additive covariance
% out:
%        y:  transformed mean
%        Y:  transformed sampling points
%        P:  transformed covariance
%        Ys: transformed deviations
function [y,Y,P,Ys] = ut(func,fargs,X,Wm,Wc,m,R)

N = size(X,2);
y = zeros(m,1);
Y = zeros(m,N);

for i = 1:N                   
    Y(:,i) = func(X(:,i),fargs);       
    y = y + Wm(i)*Y(:,i);       
end

Ys = Y - y(:,ones(1,N));
P  = Ys*diag(Wc)*Ys' + R;
