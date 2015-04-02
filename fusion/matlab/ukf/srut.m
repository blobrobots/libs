%% ut.m
%  author: adrian jimenez gonzalez
%  email:  blob.robotics@gmail.com
%  date:   15-jan-2015
%  brief:  function to perform ukf unscented transformation

function [y,Y,S,Ys] = ut(func,fargs,X,Wm,Wc,n,Sr)
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

L = size(X,2);
y = zeros(n,1);
Y = zeros(n,L);
for k = 1:L                   
    Y(:,k) = func(X(:,k),fargs);       
    y = y + Wm(k)*Y(:,k);       
end
Ys = Y - y(:,ones(1,L));
% P  = Ys*diag(Wc)*Ys' + R; % std ukf
[Q, R] = qr([sqrt(Wc(2))*Ys(2:end,:) Sr]);
S=R'; % lower triangular

csign='+';
if(Wc(1)<0)
  csign='-';
end;
S = cholupdate(S,Wc(1)*Ys(1,:),csign);

