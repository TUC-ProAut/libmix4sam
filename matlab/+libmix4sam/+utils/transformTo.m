function [p_C] = transformTo(T_WC,p_W)
%TRANSFORMTO Do the gtsam.Pose.transformTo in Matlab style.
%   E.g. a transformation from Camera to World is given within Pose T_WC
%   and Points are given within the world frame. We want to have the Points
%   now within the Camera frame.
%   T_WC = [R_WC t_W_C;0 1];
%   p_C = (R_WC)^T * (p_W-t_W_C)
%
% INPUT:
%   T_WC ... gtsam.Pose2 or gtsam.Pose3
%   p_W ... 2xN or 3xN matrix
%
% OUTPUT:
%   p_C ... 2xN or 3xN matrix

% @author Sven Lange

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

    t_W_C = T_WC.translation;
    if isnumeric(t_W_C)
        p_C = (T_WC.rotation.matrix)'*(p_W - T_WC.translation);
    else
        % Compatibility to matlab-wrapper of gtsam verson older than 4.1
        p_C = (T_WC.rotation.matrix)'*(p_W - T_WC.translation.vector);
    end

end

