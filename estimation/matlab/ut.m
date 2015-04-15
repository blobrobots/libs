%% ut.m
%  author: adrian jimenez gonzalez
%  email:  blob.robotics@gmail.com
%  date:   15-jan-2015
%  brief:  function to perform ukf unscented transformation

function [y,Y,P,Ys] = ut(func,fargs,X,Wm,Wc,m,R)
% Input:
%        f: nonlinear map
%        X: sigma points
%       Wm: weights for mean
%       Wc: weights for covraiance
%        n: numer of outputs of f
%        R: additive covariance
% Output:
%        y:  transformed mean
%        Y:  transformed sampling points
%        P:  transformed covariance
%        Ys: transformed deviations
%
% Inspired on initial script by Yi Cao at Cranfield University, 04/01/2008

N = size(X,2);
y = zeros(m,1);
Y = zeros(m,N);
for i = 1:N                   
    Y(:,i) = func(X(:,i),fargs);       
    y = y + Wm(i)*Y(:,i);       
end
Ys = Y - y(:,ones(1,N));
P  = Ys*diag(Wc)*Ys' + R;
