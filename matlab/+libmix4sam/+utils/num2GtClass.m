function [b] = num2GtClass(a, classConst)
%NUM2GTCLASS Create an array of the given gtsam class.
%   Each row in numerical matrix a is used to call the constructor for one
%   element in the gtsam-class array.
%
% INPUT
%   a ... numerical matrix for calling the gtsam class constructor
%   classConst ... Handle to the gtsam class to construct
% OUTPUT
%   Class array
% EXAMPLE
%   a = [1,1;2,1;3,1];
%   b = libmix4sam.utils.num2GtClass(a,@gtsam.Point2)
%

% @author Sven Lange (TU Chemnitz, ET/IT, Prozessautomatisierung)

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

    for i=1:size(a,1)
        l = num2cell(a(i,:));
        b(i) = classConst(l{:});
    end
end

