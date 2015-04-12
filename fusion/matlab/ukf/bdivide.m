function [ R ] = bdivide(A,B)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
n = size(B,1);
nc = size(A,1);
R = zeros(n,n);
L=chol(B)';

for c = 1:n
  % forward solve Ly = b
  for i = 1:n
    R(i,c) = A(c,i);
    j=1;
    while(j<i)
      R(i,c) = R(i,c) - L(i,j)*R(j,c);
      j=j+1;
    end
    R(i,c) = R(i,c)/L(i,i);
  end
  % backward solve L'x = y
  i=n; % not needed
  while i > 0
    %x(i) = R(i,c);
    j = i+1; %i+1?
    while j <= n 
      R(i,c) = R(i,c) - L(j,i)*R(j,c);
    j = j+1;
    end
    R(i,c) = R(i,c)/L(i,i);
    i=i-1;
  end
end

end

