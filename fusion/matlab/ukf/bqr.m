function [Q, R] = bqr(A)
%BQR blobrobots version of qr decomposition
    [m,n] = size(A);
    Q = eye(m);
    R = A;
    i = 1;
	while (i<n+1)&&(i < m)
        H = eye(m)
        r=R(i:m,i);
        H(i:end,i:end) = householder(r)
        Q = Q*H;
        R = H*R;
	    i=i+1;
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [H] = householder(a)
m=size(a,1);
sign=1;
if(a(1) < 0)
    sign=-1;
end
v = a/(a(1) + sign*norm(a));
v(1) = 1;
H = eye(m)-(2/(v'*v))*v*v';
end