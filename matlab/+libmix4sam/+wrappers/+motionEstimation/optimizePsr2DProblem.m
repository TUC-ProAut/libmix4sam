function [res] = optimizePsr2DProblem(previous, current, varargin)
%OPTIMIZEPSR2DPROBLEM Solve a general 2D Point-Set-Registration problem.
%  Here we use our Package radarMotionEstimation to solve a general 2D
%  Point Set Registration problem using Matlab's Optimization functions and
%  optionally the Sum-Approximation of the cost function's structure.
%
%   See also RUNPSR2DEXPERIMENT.

% @author Karim Haggag (TU Chemnitz, ET/IT, Prozessautomatisierung)
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

p = inputParser();
addRequired( p,'previous',     @(x)validateattributes(x,{'double'},{'size', [NaN,2]}));
addRequired( p,'current',      @(x)validateattributes(x,{'double'},{'size', [NaN,2]}));
addOptional( p,'pP',       [], @(x)validateattributes(x,{'double'},{'size', [2,2,NaN]}));
addOptional( p,'cP',       [], @(x)validateattributes(x,{'double'},{'size', [2,2,NaN]}));
addParameter(p,'init',[0 0 0], @(x)validateattributes(x,{'double'},{'numel', 3})); % x,y,theta
% Variant of cost function's structure
validFct = @(x)assert(ismember(x,{'Sum','Likeliehood'}),...
    'Unknown variant given!');
addParameter(p,'variant', 'Likeliehood', validFct);

parse(p, previous, current, varargin{:});
p = p.Results;

current  = permute(p.current',[1 3 2]);
previous = permute(p.previous',[1 3 2]);
cP       = p.cP;
pP       = p.pP;
init     = p.init;

% Prepare output structure
res = struct('mean', zeros(3,1), 'covar', eye(3), 'error', 0, 'iterations', uint64(0));

% Define cost function
switch p.variant
    case 'Sum'
        f1 = @(T) matlabPsrSum(T, current, previous, cP, pP);
    case 'Likeliehood'
        f1 = @(T) matlabPsrLikeliehood(T, current, previous, cP, pP);
    otherwise
        error('Variant unknown!')
end

% Define optimizer options and optimize
options = optimoptions('fminunc', 'Display', 'notify');
[x1,fval1,~,output1,~,hessian1] = fminunc(f1, init, options);

res.iterations = uint64(output1.iterations);
res.error      = fval1;
res.mean       = x1(:);
res.covar      = inv(hessian1);

end

function cost = matlabPsrLikeliehood(T, M_i, M_j, Sigma_i, Sigma_j)
    %T ... x , y , theta
    [M_i, Sigma_i] = radarMotionEstimation.ndtCart2Ref(M_i, Sigma_i, T, 1);
    cost = radarMotionEstimation.ndtObjectFun(M_i, M_j, Sigma_i, Sigma_j,'outlierRatio',0);
end

function cost = matlabPsrSum(T, M_i, M_j, Sigma_i, Sigma_j)
    %T ... x , y , theta
    [M_i, Sigma_i] = radarMotionEstimation.ndtCart2Ref(M_i, Sigma_i, T, 1);
    cost = radarMotionEstimation.ndtObjectFunSum(M_i, M_j, Sigma_i,Sigma_j);
end
