%% sigmas.m
%  author: adrian jimenez gonzalez
%  email:  blob.robotics@gmail.com
%  date:   15-jan-2015
%  brief:  function to caculate pdf sigma points around reference point

function X = sigmas(x,P,c)
% Inputs:
%       x: reference point
%       P: covariance
%       c: coefficient
% Output:
%       X: Sigma points
%
% Inspired on initial script by Yi Cao at Cranfield University, 04/01/2008

L = numel(x);
A = c*chol(P)';
Y = x(:,ones(1,L));
X = [x Y+A Y-A];
