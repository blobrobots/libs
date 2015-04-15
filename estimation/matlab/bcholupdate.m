function [L] = bcholupdate(L,x,sign)
% L: lower triangular matrix from cholesky decomp. (A=L*L')
% vector used to update A: A = A + x*x' (update); A = A-x*x' (downdate)
% s: sign (only +1 (update) and -1 (downdate) accepted) 
n = length(x);
if(size(L,1) == n && size(L,2) == n && (sign == -1 || sign == 1))
    for i=1:n
        r = sqrt(L(i,i)^2 + sign*x(i)^2);
        c = r/L(i,i);
        s = x(i)/L(i,i);
        L(i,i) = r;
        for j=i+1:n
            L(j,i) = (L(j,i) + sign*s*x(j))/c;
            x(j) = c*x(j) - s*L(j,i);
        end
    end
end

end

