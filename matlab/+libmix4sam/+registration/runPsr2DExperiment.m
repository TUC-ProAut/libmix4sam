function [estimation, timing] = runPsr2DExperiment(experiment, varargin)
%RUNPSR2DEXPERIMENT Do a simple 2D point set registration.
%
%   See also GENPSR2DPROBLEM, OPTIMIZEPSR2DPROBLEM.

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

%% Evaluate input arguments 
p = inputParser;

% Additional information about the experiment. (Properties used to create
% the experiment)
addParameter(p,'experimentProps',[],@isstruct);
% Switch between GMM-Implementations
%  1 - MaxMix
%  2 - SumMix
%  3 - MaxSumMix
validFct = @(x)assert(ismember(x,{'MaxMix','SumMix','MaxSumMix'}),...
    'Unknown gmmImplementation given!');
addParameter(p,'gmmImplementation', 'MaxMix', validFct);
%
addParameter(p,'doOutlierModeling', false, @islogical)
% Calculate the numerical hessian as additional covariance information.
addParameter(p,'doNumericalHessian', false, @islogical);
% Use Scaling
% If value is zero, scaling is deactivated. Otherwise, the value equals the
% scaling factor of the standard deviations of the current scan. It than
% constructs two optimization problems. One with the scaled covariances and
% one without. The first problem will be solved with a maximum of 3
% iterations. Afterwards, the result will be used as initialization for the
% second optimization problem with the correct covariances. By using
% scaling, local minima can possibly be avoided and the chance of reaching
% the global minima is raised.
validFct = @(x)validateattributes(x,{'numeric'},{'>=',0});
addParameter(p,'useScaling', 0, validFct);
% Save Experiment: if not empty, a folder with the name will be created and
% the result ist stored. 
addParameter(p,'workingFolder',[], @ischar);
% Name of the Experiment
addParameter(p,'resultName',[], @ischar);
% Should we use results from previous runs, if parameters are the same?
addParameter(p,'useCached','always',@(x)ismember(x,{'always','ask','never'}));

parse(p, varargin{:});
p = p.Results;

%% Check, if the experiment is already done
[estimation, timing] = libmix4sam.registration.loadRegistrationResult(p);
if ~isempty(estimation) && ~isempty(timing), return; else, clear estimation; end

%% Initialize parallel pool to avioid counting it to time measurement.
poolobj = gcp('nocreate'); 
if isempty(poolobj); parpool; end

%% (optional) Create outlier model
if p.doOutlierModeling
    %TODO 
    %olModel = libmix4sam.utils.OutlierModelSquare(experimentProps.landmarks.size, 'Resolution', [2 2]);
    %olModel = libmix4sam.utils.OutlierModelSquare([15 15], 'Resolution', [2 2]);
    olModel = libmix4sam.utils.OutlierModelSquare([30 30], 'Resolution', [1 1]);
    %olWeight = experimentProps.OlPercentage;
    olWeight = 0.2;
else
    olModel = []; % needed for parfor
    olWeight = 0;
end

%% Run the optimization for different noise samples and landmark sets
gtsam.tictoc_reset_(); 

tStart = tic;
for iLmSet = size(experiment,2):-1:1
    
    fprintf('Running landmark configuration Nr. %03d\n',iLmSet);
    
    %for i = size(experiment,1):-1:1
    parfor i=1:size(experiment,1)

        % Get noisy measurement
        previous = experiment(i,iLmSet).previous;
        current = experiment(i,iLmSet).current;
        init = experiment(i,iLmSet).init;
        init = gtsam.Pose2(init(1),init(2),init(3));
        
        if p.doOutlierModeling
            olParams = {'outlierWeight', olWeight, 'outlierPdf', olModel.Mixture};
        else
            olParams = {};
        end

        estimation(i,iLmSet) = libmix4sam.registration.optimizePsr2DProblem(...
            struct('points', previous.Coordinates, 'P', previous.Covariance), ...
            struct('points', current.Coordinates, 'P', current.Covariance), ...
            'gmmImplementation', p.gmmImplementation, ...
            'doNumericalHessian', p.doNumericalHessian, ...
            'useScaling', p.useScaling, ...
            'init', init, olParams{:});

    end

end
timing = toc(tStart);

[estimation.gt] = experiment.gt;
[estimation.init] = experiment.init;

gtsam.tictoc_print_()

%% Save the result!
libmix4sam.registration.saveRegistrationResult(p, estimation, timing);

end


