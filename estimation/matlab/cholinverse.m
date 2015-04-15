function [ R ] = cholinverse(A)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
n = size(A,1);
R = zeros(n,n);
L = chol(A)';

for c = 1:n
  % forward solve Ly = b
  for i = 1:n
    if (c==i)
      R(c,i) = 1;
    else
      R(c,i) = 0;
    end
      
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

