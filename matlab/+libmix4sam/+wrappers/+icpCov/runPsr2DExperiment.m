function [estimation, timing] = runPsr2DExperiment(experiment, varargin)
%RUNPSR2DEXPERIMENT Do a simple 2D point set registration.
%   The ICP implementation using PCL from motionEstimation Package.
%
%   See also OPTIMIZEPSR2DPROBLEM.

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

%% Evaluate input arguments 
p = inputParser;

% Additional information about the experiment. (Properties used to create
% the experiment)
addParameter(p,'experimentProps',[],@isstruct);
% NOT IMPLEMENTED
addParameter(p,'gmmImplementation', [], @isempty);
% If true, a robust correspondence finding algorithm is used. See source
% for more details.
addParameter(p,'doOutlierModeling', false, @islogical)
% NOT IMPLEMENTED
addParameter(p,'doNumericalHessian', false, @islogical);
% NOT IMPLEMENTED
addParameter(p,'useScaling', 0, @isnumeric);

% Save Experiment: if not empty, a folder with the name will be created and
% the result ist stored. 
addParameter(p,'workingFolder',[], @ischar);
% Name of the Experiment
addParameter(p,'resultName',[], @ischar);
% Should we use results from previous runs, if parameters are the same?
addParameter(p,'useCached','always',@(x)ismember(x,{'always','ask','never'}));

parse(p, varargin{:});
p = p.Results;

% NOT IMPLEMENTED
p = rmfield(p,{'gmmImplementation','doNumericalHessian','useScaling'}); 

%% Check, if the experiment is already done

[estimation, timing] = libmix4sam.registration.loadRegistrationResult(p);
if ~isempty(estimation) && ~isempty(timing), return; else, clear estimation; end

%% Initialize parallel pool to avioid counting it to time measurement.
poolobj = gcp('nocreate'); 
if isempty(poolobj); parpool; end

%% Run the optimization for different noise samples and landmark sets
tStart = tic;
for iLmSet = size(experiment,2):-1:1
    
    fprintf('Running landmark configuration Nr. %03d\n',iLmSet);
    
    %for i = size(experiment,1):-1:1
    parfor i=1:size(experiment,1)

        % Get noisy measurement
        previous = experiment(i,iLmSet).previous;
        current = experiment(i,iLmSet).current;
        init = experiment(i,iLmSet).init;
        
        estimation(i,iLmSet) = libmix4sam.wrappers.icpCov.optimizePsr2DProblem(...
            previous.Coordinates, current.Coordinates,...
            previous.Covariance, current.Covariance,...
            'init', init, 'useRobust', p.doOutlierModeling);

    end

end
timing = toc(tStart);

[estimation.gt] = experiment.gt;
[estimation.init] = experiment.init;

%% Save the result!

libmix4sam.registration.saveRegistrationResult(p, estimation, timing);


end

