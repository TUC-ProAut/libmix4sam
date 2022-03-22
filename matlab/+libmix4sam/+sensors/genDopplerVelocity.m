function set = genDopplerVelocity(points, motion, offset)
%GENDOPPLERVELOCITY Doppler velocity measurement model.
%

% @author Karim Haggag (TU Chemnitz, ET/IT, Prozessautomatisierung)

% This file is part of
% libmix4sam - Mixtures for Smoothing and Mapping Library
%
% Copyright (C) 2020 Chair of Automation Technology / TU Chemnitz
% For more information see https://mytuc.org/mix
%
% libmix4sam is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
%
% libmix4sam is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with this software.  If not, see <http://www.gnu.org/licenses/>.
%
% Contact Information: Sven Lange (sven.lange@etit.tu-chemnitz.de)

% Target measurement
% points   : (number of targets)     48 x 3 [ phik rk vk]
% motion   : (1 x 3)                        [vx vy omega]
% offset   : (1 x 3)                        [xs ys alpha]
% 
% Ex1 : point = [0.2458   15.5935 ;...
%                0.4928   21.4556 ;...
%                0.3532   31.7386 ;...
%                0.0770   23.3288 ;...
%               -0.0364   32.8625 ;...
%               -0.6750   14.4914 ;...
%               -0.5330   24.3163 ;...
%               -0.3608   31.1166 ]
%
%      motion : [1 0 deg2rad(10)]
%      offset : [0 0 0]
% Then : 
%        ==> genDopplerVelocity(point,motion,offset)
% 
% The results velocity : 
%    [ -0.9912
%      -0.9223
%      -0.9685
%      -1.0038
%      -0.9962
%      -0.7262
%      -0.8169
%      -0.9048 ]

if nargin < 3, offset = zeros(3,1); end

% Theta 
phik      = points(:,1);

% offset
x         = offset(1);
y         = offset(2);
alpha     = offset(3);

% motion_state
vx        = motion(1);
vy        = motion(2);
omega     = motion(3);

set = polar_2_doppler(phik,vx,vy,omega,x,y,alpha);

function [Vmkp] = polar_2_doppler(phik,vx,vy,omega,xs,ys,alpha)
    t2 = alpha+phik;
    t3 = cos(t2);
    t4 = omega.*xs;
    t5 = t4+vy;
    t6 = sin(t2);
    t7 = vx-omega.*ys;
    temp_Vmkp = -t3.*t7-t5.*t6;

    V_mkp = reshape(temp_Vmkp,[1 1 length(phik)]);
    Vmkp  =  permute(V_mkp,[3 2 1]);
end
end
