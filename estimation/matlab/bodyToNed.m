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
% \file       bodyToNed2d.m
% \brief      transform from body to earth NED frame
% \author     adrian jimenez-gonzalez (blob.robots@gmail.com)
% \copyright  the MIT License Copyright (c) 2015 Blob Robots.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% function to transform from body to earth NED frame
% in:
%   roll = roll euler angle
%  pitch = pitch euler angle
%    yaw = heading in NED coord. system
%    bfx = value of x axis in body frame
%    bfy = value of y axis in body frame
%    bfz = value of z axis in body frame
% out:
%    efx = value of x axis in NED frame
%    efy = value of y axis in NED frame
%    efz = value of z axis in NED frame
function [efx, efy, efz] = bodyToNed(roll, pitch, yaw, bfx, bfy, bfz)

efx =  bfx*cos(pitch)*cos(yaw) + bfy*(sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw)) + bfz*(cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw));
efy =  bfx*cos(pitch)*sin(yaw) + bfy*(sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(yaw)) + bfz*(cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*cos(yaw));
efz = -bfx*sin(pitch)          + bfy*sin(roll)*cos(pitch)                                 + bfz*cos(roll)*cos(pitch);
 
end