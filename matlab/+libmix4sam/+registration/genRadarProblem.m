function [experiment, p] = genRadarProblem(defaultSeed, varargin)
%GENRADARPROBLEM Generate a generic radar-registration problem in 2D.
%   
%   See also RUNRADAREXPERIMENT, OPTIMIZERADARPROBLEM.

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

% Default seed parameter. If not defined otherwise, this parameter will be
% used for generating random numbers. For every reuse of this parameter, an
% offset will be added though. 
if nargin < 1, defaultSeed = 123; end
if isempty(defaultSeed) || defaultSeed == 0, defaultSeed = sum(100*clock); end

p = inputParser;
% Define a transformation range or fixed predefined transformations for the
% monte carlo experiments. If a range is given, the transformation is
% generated randomly based on a uniform distribution within the range.
% [x y theta] or {'name',value, ...}
default = struct('xrange',[-0.25 0.25],'thetarange',[-20 20]*pi/180,'seed',defaultSeed + 24390);
addParameter(p,'transformation', default, @(x)validateattributes(x,{'struct','numeric'},{}));
% Predefine some landmarks or let the script create a random set.
% For predefined landmarks, just give an Nx2 matrix
% For random give parameters for the landmark's generate method.
default = struct('type', 'circle', 'size', [20 35], 'amount', 20, 'seed', defaultSeed + 90);%,'center',[10 0]};
addParameter(p,'landmarks', default, @(x)validateattributes(x,{'cell','struct','numeric'},{}));
% Applies only if above parameter is given as a cell!
% How many different landmark sets should be created? Default is 1.
addParameter(p,'numLandmarkSets', 1, @(x)validateattributes(x,{'numeric'},{'nonnegative','integer'}));
% Add additional landmarks to build clusters of landmarks. Especially for
% MaxMix is this a challanging task, because of the overlapping
% distributions.
addParameter(p,'doLandmarkClustering', false, @islogical)
% Properties for clustering the landmarks (see code below).
addParameter(p,'landmarkClusterProps', {0.4, 3, [0.1 0.1]}, @iscell)
% How many monte carlo experiments should be done with the set of landmarks
addParameter(p,'numMonteCarloExperiments', 100, @isnumeric);
% Seed for the monte carlo experiment
addParameter(p,'measurementSeed', defaultSeed, @isnumeric);
% Initialization for the optimizer. Default is zero, but e.g. for avoiding
% the problem of local minima to evaluate the consistency, GroundTruth 
% could be used here.
validFct = @(x)assert(ismember(x,{'GroundTruth','Zero'}),...
    'Unknown initialization method given!');
addParameter(p,'initialization', 'Zero', validFct);
% Save Experiment: if not empty, a folder with the name will be created and
% the result ist stored. 
addParameter(p,'workingFolder',[], @ischar);
% Name of the Experiment
addParameter(p,'experimentName',[], @ischar);
% Should we use results from previous runs, if parameters are the same?
addParameter(p,'useCached','always',@(x)ismember(x,{'always','ask','never'}));

parse(p, varargin{:});
p = p.Results;

%% Set or create transformations
if isstruct(p.transformation)
    poseBetween = getTransformations(p.numMonteCarloExperiments, p.transformation);
    fovOffset =  sum(abs(p.transformation.thetarange));
else
    poseBetween = repmat(p.transformation(:)', p.numMonteCarloExperiments, 1);
    poseBetween = arrayfun(@(x,y,theta)gtsam.Pose2(x,y,theta),poseBetween(:,1),poseBetween(:,2),poseBetween(:,3));
    fovOffset = max(abs(p.transformation(:,3)));
end
radarDt = 0.2;

%% We only want to generate Landmarks in regions where they are needed
% otherwise we run into problems regarding cluser generation and the
% overall number of landmarks for the measurement.
fovRadar = 80*pi/180;
p.landmarks.fov = fovRadar + fovOffset;

%% Check, if the experiment is already done

% We dont want to save the following parameters as they may be changed
% without having influence on the generated data
useCached = p.useCached; 
workingFolder = p.workingFolder;
experimentName = p.experimentName;
p = rmfield(p,{'useCached','workingFolder','experimentName'}); 

if ~isempty(workingFolder) && ~isempty(experimentName)
    fname = fullfile(workingFolder, ['setup_' experimentName]);
    if exist([fname '.mat'], 'file')
        if strcmp(useCached, 'ask')
            answer = questdlg('Problem generation already done. Do you want to redo it?', ...
                'Redo problem?', ...
                'Redo', 'Skip','Skip');
        end
        if strcmp(useCached,'always') || strcmp(answer,'Skip')
            warning('Problem generation already done. Loading old data.');
            data = load(fname);
            experiment = data.experiment;
            assert(strcmp(libmix4sam.utils.getVariableChecksum(p),libmix4sam.utils.getVariableChecksum(data.p)),...
              'The previously generated experiment''s parameters don''t match the current ones!');
            return; 
        end
    end
end

%% Initialize parallel pool to avioid counting it to time measurement.
poolobj = gcp('nocreate'); 
if isempty(poolobj); parpool; end

%% Set or create landmarks
if iscell(p.landmarks) || isstruct(p.landmarks)
    if iscell(p.landmarks)
        lm = libmix4sam.utils.Landmarks2D.generateMultiple(p.numLandmarkSets, p.landmarks{:});
    else
        lm = libmix4sam.utils.Landmarks2D.generateMultiple(p.numLandmarkSets, p.landmarks);
    end
    %lmProps = cell2struct(default(2:2:end),default(1:2:end),2);
else
    lm = libmix4sam.utils.Landmarks2D(p.landmarks); 
end

%% Add additional landmarks to build clusters of landmarks.
if p.doLandmarkClustering
    lmp = p.landmarkClusterProps; % for convenience
    lm.addClusters(lmp{1}, ... % Percentage of Landmarks beeing converted to clusters
                   lmp{2}, ... % Number of Landmarks within a cluster
                   'Noise',gtsam.noiseModel.Diagonal.Sigmas(lmp{3}(:)),...
                   'Seed',defaultSeed + 2710); % Cluster spread of cluster-landmarks around original landmark
end

%% Generate optimization problem for different noise samples and landmark sets
tStart = tic;
for iLmSet = length(lm):-1:1
        
    %for i = p.numMonteCarloExperiments:-1:1
    parfor i=1:p.numMonteCarloExperiments
    
        % Generate one view of the landmarks and use it as ground truth for the fixed frame
        % Additionally, generate one view from another pose, which is to be
        % estimated by the motion estimation algorithm.
        measurementPreviousGt = lm(iLmSet).getRelativeLm(gtsam.Pose2(), fovRadar);
        measurementCurrentGt = lm(iLmSet).getRelativeLm(poseBetween(i), fovRadar);
        
        % We should have a minimal number of landmarks!
        if measurementPreviousGt.getSize < 5 || measurementCurrentGt.getSize < 5
            error('There are to less landmarks within the field of view of the sensor!');
        end

        % Convert Landmarks into radar measurement
        measurementPreviousGt = libmix4sam.sensors.Radar.generate(...
            measurementPreviousGt, ...
            gtsam.noiseModel.Diagonal.Sigmas([0.0157 0.13 0.3]'),... % old ones: Sigmas([0.03 0.2 0.01]') - Phi Rho Doppler
            'id',measurementPreviousGt.Id);
        measurementCurrentGt = libmix4sam.sensors.Radar.generate(...
            measurementCurrentGt, ...
            gtsam.noiseModel.Diagonal.Sigmas([0.0157 0.13 0.3]'),... %new ones: Sigmas([0.0157 0.13 0.3]
            'Motion', [poseBetween(i).x poseBetween(i).y poseBetween(i).theta] / radarDt,...
            'dt',radarDt,...
            'id',measurementCurrentGt.Id);

        % Use an initialization if option is selected.
        if strcmp(p.initialization,'GroundTruth'), init = poseBetween(i); else, init = gtsam.Pose2(); end

        % Get noisy measurement
        previous = measurementPreviousGt.getNoisy(p.measurementSeed+i);
        current = measurementCurrentGt.getNoisy(p.measurementSeed/2*i);

        % Add Ground truth and additional information to the result struct
        experiment(i,iLmSet).previousGt = measurementPreviousGt;
        experiment(i,iLmSet).previous = previous;
        experiment(i,iLmSet).currentGt = measurementCurrentGt;
        experiment(i,iLmSet).current = current;
        experiment(i,iLmSet).numCorrespondencesGt = length(previous.getCorrespondenceIds(current));
        experiment(i,iLmSet).gt = [poseBetween(i).x, poseBetween(i).y, poseBetween(i).theta]';
        experiment(i,iLmSet).init = [init.x, init.theta]';

    end

end
fprintf('Dataset generation took %0.2d seconds.\n', toc(tStart));

%% Save the result!
if ~isempty(workingFolder) && ~isempty(experimentName)
    if ~exist(workingFolder,'dir'), mkdir(workingFolder); end
    save([fname '.mat'], 'experiment','p');
    % Have the properties human readable too.
    libmix4sam.utils.saveAsJson([fname '.json'],p);
end

end

function t = getTransformations(varargin)
    p = inputParser;
    addRequired(p,'numTransformations');
    addParameter(p,'xrange', [-1 1], @isnumeric);
    addParameter(p,'thetarange', [-45 45]*pi/180, @isnumeric);
    addParameter(p,'seed', 24513, @isnumeric);
    parse(p, varargin{:});
    p = p.Results;
    [r1,r2] = RandStream.create('mrg32k3a','Seed',p.seed,'NumStreams',2);
    
    x = (p.xrange(2)-p.xrange(1)).*rand(r1,p.numTransformations,1) + p.xrange(1);
    theta = (p.thetarange(2)-p.thetarange(1)).*rand(r2,p.numTransformations,1) + p.thetarange(1);
    
    t = arrayfun(@(x,theta)gtsam.Pose2(x,0,theta),x,theta);
end
