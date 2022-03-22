function [estimation, timing] = runRadarExperiment(experiment, varargin)
%RUNRADAREXPERIMENT Execute a radar registration experiment using SA.
%
%   See also OPTIMIZERADARPROBLEM.

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

% Save Experiment: if not empty, a folder with the name will be created and
% the result ist stored. 
addParameter(p,'workingFolder',[], @ischar);
% Name of the Experiment
addParameter(p,'resultName',[], @ischar);
% Should we use results from previous runs, if parameters are the same?
addParameter(p,'useCached','always',@(x)ismember(x,{'always','ask','never'}));

% We run optimizeRadarProblem() for each measurement pair.
% The following parameters will be passed to this function, if they are
% set. The meaning of the parameters is described within the function!
optimizerParams = {'useDoppler',...
                   'useScaling',...
                   'additionalDopplerScaling',...
                   'useSimplification',...
                   'doNumericalHessian',...
                   'gmmImplementation',...
                   'outlierWeight',...
                   'outlierWeightDoppler'};
% Add optimizerParams to input parser
for param = optimizerParams, addParameter(p,param{1},[]); end

% Evaluate the input parameters and split into here needed params and
% params to pass to other functions
parse(p, varargin{:});
notNeeded = unique([setdiff(p.Parameters, optimizerParams) p.UsingDefaults]);

% NOT IMPLEMENTED in SUMAPPROX
notNeeded = unique([notNeeded {'doNumericalHessian','useScaling','additionalDopplerScaling','useSimplification','doNumericalHessian','outlierWeight','outlierWeightDoppler'}]); 

optimizerParams = rmfield(p.Results, notNeeded); % Params for optimizeRadarProblem()
p = p.Results;

%% Check, if the experiment is already done

[estimation, timing] = libmix4sam.registration.loadRegistrationResult(p);
if ~isempty(estimation) && ~isempty(timing), return; else, clear estimation; end

%% Initialize parallel pool to avioid counting it to time measurement.
poolobj = gcp('nocreate'); 
if isempty(poolobj); parpool; end
D = parallel.pool.DataQueue;

% Code for progressbar
hWaitbar = waitbar(0, 'Please wait ...');
afterEach(D, @nUpdateWaitbar);
NWaitbar = numel(experiment);
pWaitbar = 1;

    function nUpdateWaitbar(~)
        waitbar(pWaitbar/NWaitbar, hWaitbar);
        pWaitbar = pWaitbar + 1;
    end

%% Run the optimization for different noise samples and landmark sets
tStart = tic;
for iLmSet = size(experiment,2):-1:1
    
    fprintf('Running landmark configuration Nr. %03d\n',iLmSet);
    
    %for i = size(experiment,1):-1:1
    %for i = 1:size(experiment,1)
    parfor i=1:size(experiment,1)

        % Get noisy measurement
        previous = experiment(i,iLmSet).previous;
        current = experiment(i,iLmSet).current;
        init = experiment(i,iLmSet).init;
        
        estimation(i,iLmSet) = libmix4sam.wrappers.motionEstimation.optimizeRadarProblem(...
            previous, current,...
            'init', init, optimizerParams);

        send(D, i); % for progress bar update
    end

end
timing = toc(tStart);
delete(hWaitbar); % close progress bar

[estimation.gt] = experiment.gt;
[estimation.init] = experiment.init;

%% Save the result!

libmix4sam.registration.saveRegistrationResult(p, estimation, timing);


end

