function [ L ] = bchol( A, epsilon )

n = size(A,1);
L = A;
dots = zeros(1,n);
piv = 1:n;

for j=1:n
   if j > 1
      for i=j:n
         dots(i) = dots(i) + L(i, j-1)*L(i, j-1);
      end
   end
   % q = min{p: L(p,p)-dots(p) = max{L(i,i)-dots(i)}_i=j:n}
   mm = L(j,j)-dots(j);
   q = j;
   for i=j+1:n
      res = L(i,i)-dots(i); 
      if(res < mm)
         mm = res;
         q = i;
      end
   end
   %stopping criteria
   for i=j:n
       if L(i,i) <= epsilon
           disp('Non Positive Definite Matrix')
           return;
       end
   end
   % swap L(j,:) and L(q,:)
   for i=1:n
    swap = L(j,i);
    L(j,i)=L(q,i);
    L(q,i)=swap;
   end
   % swap L(:,j) and L(:,q)
   for i=1:n
    swap = L(i,j);
    L(i,j)=L(i,q);
    L(i,q)=swap;
   end
   % swap dots(j) and dots (q)
   swap = dots(j);
   dots(j) = dots(q);
   dots(q) = swap;
   
   % swap piv(j) and piv(q)
   swap = piv(j);
   piv(j) = piv(q);
   piv(q) = swap;
   
   % update
   L(j,j) = sqrt(L(j,j) - dots(j));
   
   if j>1&&j<n
       L(j+1:n,j)=L(j+1:n,j)-L(j+1:n,1:j-1)*L(j,1:j-1)';
   end
   if j<n
       L(j+1:n,j) = L(j+1:n,j)/L(j,j);
   end
end
piv
dots
end
