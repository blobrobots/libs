function output = f_imu7q(x, u, dt)
% x = [q0, q1, q2, q3, gbx, gby, gbz]
% u = [gx, gy, gz]

q0 = x(1); q1 = x(2); q2 = x(3); q3 = x(4); gbx = x(5); gby = x(6); gbz = x(7);
gx = u(1)-gbx; gy = u(2)-gby; gz = u(3)-gbz;

% predict new state
% FRD
output = [ q0 + (-q1*gx - q2*gy - q3*gz)*dt/2;
           q1 + ( q0*gx + q3*gy - q2*gz)*dt/2;
           q2 + (-q3*gx + q0*gy + q1*gz)*dt/2;
           q3 + ( q2*gx - q1*gy + q0*gz)*dt/2;
           gbx;
           gby;
           gbz;                                                   ];

% FLU
% output = [ q0 - (-q1*gx  - q2*gy - q3*gz)*dt/2;
%            q1 - ( q0*gx  - q3*gy + q2*gz)*dt/2;
%            q2 - ( q3*gx  + q0*gy - q1*gz)*dt/2;
%            q3 - (-q2*gx  + q1*gy + q0*gz)*dt/2;
%            gbx;
%            gby;
%            gbz;                                                   ];

% re-normalize quaternion
qnorm = sqrt(output(1)*output(1) + output(2)*output(2) + output(3)*output(3) + output(4)*output(4));
output(1) = output(1)/qnorm;
output(2) = output(2)/qnorm;
output(3) = output(3)/qnorm;
output(4) = output(4)/qnorm;

end
