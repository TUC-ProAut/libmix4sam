function [res] = optimizePsr2DProblem(previous, current, varargin)
%OPTIMIZEPSR2DPROBLEM Wrapper for icpWithCov with simple interface!
%
%   See also RUNPSR2DEXPERIMENT.

% @author Sven Lange (TU Chemnitz, ET/IT, Prozessautomatisierung)
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

p = inputParser();
addRequired( p, 'previous',           @(x)validateattributes(x,{'double'},{'size', [NaN,2]}));
addRequired( p, 'current',            @(x)validateattributes(x,{'double'},{'size', [NaN,2]}));
addOptional( p, 'pP',        [],      @(x)validateattributes(x,{'double'},{'size', [2,2,NaN]}));
addOptional( p, 'cP',        [],      @(x)validateattributes(x,{'double'},{'size', [2,2,NaN]}));
addParameter(p, 'init',      [0 0 0], @(x)validateattributes(x,{'double'},{'numel', 3})); % x,y,theta
addParameter(p, 'useRobust', false,   @islogical); % use ReciprocalCorrespondences

parse(p, previous, current, varargin{:});
p = p.Results;

% Prepare output structure
res = struct('mean', zeros(3,1), 'covar', eye(3), 'correspondence', 0, 'iterations', uint64(0));

% Extend to 3D Points (because of quick and dirty solution in icp2dTr)
p.previous(end,3) = 0;
p.current(end,3) = 0;

R=@(x)[cos(x) -sin(x);sin(x) cos(x)];
init = blkdiag(R(p.init(3)), 1, 1);
init(1:2,4) = p.init(1:2);

% Estimate transformation and correspondences
[T, i, it] = icpWithCov.icp2dTr(p.current, p.previous, init, 'points', p.useRobust);
res.iterations = uint64(it);
res.correspondence = i;
res.mean = [T(1:2,4); atan2(T(2,1),T(1,1))];
%res.mean = [T(1:2,4); -dcm2angle(T(1:3,1:3))];

% If covariances are given, calculate covariance
if ~isempty(p.pP) && ~isempty(p.cP)
    pP = squeeze(num2cell(p.pP(:,:,i(:,2)+1),[1 2]));
    cP = squeeze(num2cell(p.cP(:,:,i(:,1)+1),[1 2]));
    res.covar = icpWithCov.icp2dCov(p.current(i(:,1)+1,:), p.previous(i(:,2)+1,:), T, cP, pP, 'points');
end

end

