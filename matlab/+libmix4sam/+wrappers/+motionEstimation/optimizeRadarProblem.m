function [res] = optimizeRadarProblem(previous, current, varargin)
%OPTIMIZERADARPROBLEM Wrapper for running the sum approximation algorithm.
%
%   See also RUNRADAREXPERIMENT.

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
addRequired( p,'previous',          @(x)isa(x,'libmix4sam.sensors.Radar'));
addRequired( p,'current',           @(x)isa(x,'libmix4sam.sensors.Radar'));
addParameter(p,'init',       [0 0], @(x)validateattributes(x,{'double'},{'numel', 2})); % x,theta
addParameter(p,'useDoppler', false, @islogical);
% Variant of cost function's structure
validFct = @(x)assert(ismember(x,{'Sum','Likeliehood'}),...
    'Unknown variant given!');
addParameter(p,'gmmImplementation', 'Likeliehood', validFct);

parse(p, previous, current, varargin{:});
p = p.Results;

% Convert data for radarMotionEstimation package
dt         = current.CycleTime;
cP         = current.Cartesian.P;
pP         = previous.Cartesian.P;
M_i        = permute([current.Cartesian.x current.Cartesian.y]',[1 3 2]); %2x1xN
M_j        = permute([previous.Cartesian.x previous.Cartesian.y]',[1 3 2]); %2x1xN
v_i        = current.Doppler;
stdDoppler = current.StdDoppler;
stdPhi     = permute(current.StdPhi',[1 3 2]);
init       = p.init;

% Prepare output structure
res = struct('mean', zeros(2,1), 'covar', eye(2), 'error', 0, 'iterations', uint64(0));

% Define cost function
switch p.gmmImplementation
    case 'Sum'
        f1 = @(T) matlabPsrSum(T, M_i, M_j, cP, pP, dt, v_i, stdDoppler, stdPhi, p.useDoppler);
    otherwise
        error('Variant unknown!')
end

% Define optimizer options and optimize
options = optimoptions('fminunc', 'Display', 'notify');
[x1,fval1,~,output1,~,hessian1] = fminunc(f1, init, options);
                    
res.iterations = uint64(output1.iterations);
res.error = fval1;
res.mean = x1(:) * dt; %ndtObjectFunSum is working with velocities!
res.covar = inv(hessian1) * dt.^2;

end


function cost = matlabPsrSum(T, M_i, M_j, Sigma_i, Sigma_j, dt, v_i, stdDoppler, stdPhi, useDoppler)
    %T ... x , y , theta
    ex_vk = [];
    ex_Sigma = [];
    if useDoppler
        % Implements the measurement model for the doppler velocity 
        % (see Rapp, M. et al. (2017) ‘Probabilistic ego-motion estimation
        % using multiple automotive radar sensors’, Robotics and Autonomous
        % Systems, 89, pp. 136–146. doi:10.1016/j.robot.2016.11.009.  
        
        % Without offset and y-velocity, it simplifies to:
        %-T(1).*x./(x.^2 + y.^2).^(1/2)
        ex_vk = -T(1).*M_i(1,1,:)./(M_i(1,1,:).^2 + M_i(2,1,:).^2).^(1/2);
        
        % additional standard deviation based on angle standard deviation
        % of target:
        %vx^2 * y^2/(x^2 + y^2)
        ex_Sigma = T(1)^2 * M_i(2,1,:).^2./(M_i(1,1,:).^2 + M_i(2,1,:).^2);
        ex_Sigma = ex_Sigma .* stdPhi.^2;
    end 
    
    [M_i, Sigma_i] = radarMotionEstimation.ndtCart2Ref(...
                         M_i, Sigma_i, [T(1) 0 T(2)]', dt);
                     
    cost           = radarMotionEstimation.ndtObjectFunSum(...
                         M_i, M_j, Sigma_i, Sigma_j, ...
                         v_i, ex_vk, stdDoppler, ...
                         ex_Sigma, 'Doppler', useDoppler);
end
