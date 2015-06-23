%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The MIT License (MIT)
%
% Copyright (c) 2015 Blob Robotics
%
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal 
% in the Software without restriction, including without limitation the rights 
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is 
% furnished to do so, subject to the following conditions:
% 
% The above copyright notice and this permission notice shall be included in
% all copies or substantial portions of the Software.
% 
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE 
% SOFTWARE.
% 
% \file       f_imu4q.m
% \brief      function to predict attitude given previous attitude and control input
% \author     adrian jimenez-gonzalez (blob.robots@gmail.com)
% \copyright  the MIT License Copyright (c) 2015 Blob Robots.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% function to predict attitude given previous attitude and control input
%  in:
%    x = [q0, q1, q2, q3] = quaternion = previous state
%    u = [gx, gy, gz] = gyro
%  out:
%    output = [q0, q1, q2, q3] = quaternion = state prediction
function output = f_imu4q(x, u, dt)

q0 = x(1); q1 = x(2); q2 = x(3); q3 = x(4);
gx = u(1); gy = u(2); gz = u(3);

% state prediction in FRD reference frame
output = [ q0 + (-q1*gx - q2*gy - q3*gz)*dt/2;
           q1 + ( q0*gx + q3*gy - q2*gz)*dt/2;
           q2 + (-q3*gx + q0*gy + q1*gz)*dt/2;
           q3 + ( q2*gx - q1*gy + q0*gz)*dt/2; ];

% state prediction in FLU reference frame
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
