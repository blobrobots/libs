function [Q, R] = bqr2(A)
%BQR blobrobots version of qr decomposition
    [m,n] = size(A);
    Q = eye(m);
    R = A;
    i = 1;
	while (i<n+1)&&(i < m)
        H = eye(m)
        %r=R(i:m,i);
        %H(i:end,i:end) = householder(r)
 
        sign = 1; % a=R(i:m,i)
        if(R(i,i) < 0) %if(a(1) < 0)
            sign=-1;
        end
        
        nrm = 0;
        for(k=i:m)
            nrm=nrm+R(k,i)*R(k,i);
        end
        nrm=sqrt(nrm);
        
        d = (R(i,i) + sign*nrm); % v = a/(a(1) + sign*norm(a));
        % v(1) = 1;
        t=1; % v(1) = 1;
        for(k=i+1:m)
          t = t+(R(k,i)/d)*(R(k,i)/d);
        end
        t=2/t; % = 2/(v'*v) with v(1)=1
        for j=i:m
            for k=i:m
               if(j==i) % v(1) = 1;
                   vj = 1;
               else 
                   vj=R(j,i)/d;
               end
               if(k==i) % v(1) = 1;
                vk = 1;
               else
                   vk=R(k,i)/d;
               end
               H(j,k)=H(j,k)-vj*vk*t;
            end
        end
        Q = Q*H;
        R = H*R;
	    i=i+1;
    end
end