function [ R ] = bdivide(A,B)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
n = size(B,1);
m = size(A,1);
R = zeros(m,n);
L = chol(B)';

for c = 1:m
  % forward solve Ly = b
  for i = 1:n
    R(c,i) = A(c,i);
    j=1;
    while(j<i)
      R(c,i) = R(c,i) - L(i,j)*R(c,j);
      j=j+1;
    end
    R(c,i) = R(c,i)/L(i,i);
  end
  % backward solve L'x = y
  i=n; % not needed
  while i > 0
    %x(i) = R(i,c);
    j = i+1; %i+1?
    while j <= n 
      R(c,i) = R(c,i) - L(j,i)*R(c,j);
    j = j+1;
    end
    R(c,i) = R(c,i)/L(i,i);
    i=i-1;
  end
end
end

