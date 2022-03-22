function [experiment, p] = genPsr2DProblem(defaultSeed, varargin)
%GENPSR2DPROBLEM Generate a generic point set registration problem in 2D.
%   
%   See also RUNPSR2DEXPERIMENT, OPTIMIZEPSR2DPROBLEM.

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
default = {'xrange',[-0.5 0.5],'yrange',[-0.5 0.5],'thetarange',[-20 20]*pi/180,'seed',defaultSeed + 24390};
addParameter(p,'transformation', default, @(x)validateattributes(x,{'struct','numeric'},{}));
% Predefine some landmarks or let the script create a random set.
% For predefined landmarks, just give an Nx2 matrix
% For random give parameters for the landmark's generate method.
default = struct('type', 'square', 'size', [10 10], 'amount', 10, 'seed', defaultSeed + 90);%,'center',[10 0]};
addParameter(p,'landmarks', default, @(x)validateattributes(x,{'cell','struct','numeric'},{}));
% Additional Sensor properties like range and fov
default = struct('fov', 0, 'range', []);%,'range',[0.2 100]};
addParameter(p,'sensor', default, @(x)validateattributes(x,{'struct'},{}));
% Applies only if above parameter is given as a cell!
% How many different landmark sets should be created? Default is 1.
addParameter(p,'numLandmarkSets', 2, @(x)validateattributes(x,{'numeric'},{'nonnegative','integer'}));
% Add additional landmarks to build clusters of landmarks. Especially for
% MaxMix is this a challanging task, because of the overlapping
% distributions.
addParameter(p,'doLandmarkClustering', true, @islogical)
% Properties for clustering the landmarks (see code below).
addParameter(p,'landmarkClusterProps', {0.4, 3, [0.1 0.1]}, @iscell)
% How many outliers should be generated within both, the previous and
% current set of landmark measurements? The number is expected as a
% percentage of outliers in regard to the amount of landmarks. (Note: If 10
% landmarks should be generated and clustering is set to 0.4, there will be
% 18 landmarks, the generateOutliers property refers then to the 18
% landmarks.)
validFct = @(x)validateattributes(x,{'numeric'},{'>=',0,'<',1});
addParameter(p,'OlPercentage', [], validFct);
% What is the spread of the outliers? (Depending on the outlier percentage,
% an amount of landmarks is selected for duplication. For Duplication, the
% given spread is applied. Small values mean outliers close to existing
% landmarks.)
validFct = @(x)validateattributes(x,{'numeric'},{'numel',2});
addParameter(p,'OlSpreadStd', [0.5 0.5], validFct);
% Instead of using the same noise for every measurement, we could simulate
% different noise values, as is the case for an automotive radar sensor.
addParameter(p,'doNoisyMeasurementNoise', false, @islogical);
default = {'d',[0.2 0.3],'theta',[3 5]*pi/180,'seed', defaultSeed + 85};
addParameter(p,'noisyMeasurementNoiseProps', default, @iscell);
% How many monte carlo experiments should be done with the set of landmarks
addParameter(p,'numMonteCarloExperiments', 100, @isnumeric);
% Seed for the monte carlo experiment
addParameter(p,'measurementSeed', defaultSeed, @isnumeric);
% Initialization for the optimizer. Default is zero, but e.g. for avoiding
% the problem of local minima to evaluate the consistency, GroundTruth 
% could be used here.
validFct = @(x)assert(isnumeric(x) || ismember(x,{'GroundTruth','Zero'}),...
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

%% Set or create transformations
if isstruct(p.transformation)
    poseBetween = getTransformations(p.numMonteCarloExperiments, p.transformation);
else
    if size(p.transformation,1) == p.numMonteCarloExperiments && size(p.transformation,2) == 3
        % Transformations are given completely (for each monte carlo experiment)
        poseBetween = p.transformation;
    else
        % It is expected, that only one transformation is given
        poseBetween = repmat(p.transformation(:)', p.numMonteCarloExperiments, 1);
    end
    poseBetween = arrayfun(@(x,y,theta)gtsam.Pose2(x,y,theta),poseBetween(:,1),poseBetween(:,2),poseBetween(:,3));
end

%% Initialization
if isnumeric(p.initialization)
    assert(size(p.initialization,1) == p.numMonteCarloExperiments && size(p.initialization,2) == 3, 'Initialization has a wrong dimension!');
    init = p.initialization;
else
    if strcmp(p.initialization,'GroundTruth')
        init = libmix4sam.utils.gtClass2num(poseBetween);
    else
        init = zeros(length(poseBetween),3);
    end
end

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
        measurementPreviousGt = lm(iLmSet).getRelativeLm(gtsam.Pose2(),p.sensor.fov,p.sensor.range);
        measurementCurrentGt = lm(iLmSet).getRelativeLm(poseBetween(i),p.sensor.fov,p.sensor.range);
        
        % Add outliers
        if ~isempty(p.OlPercentage) && p.OlPercentage > 0
            olPercentage = p.OlPercentage;
            % (optional) Add Outliers to Previous
            measurementPreviousGt.addClusters(olPercentage, 2, ...
                'Noise', gtsam.noiseModel.Diagonal.Sigmas(p.OlSpreadStd(:)), ...
                'Seed', defaultSeed + 32458, 'Ids', -1);
            % (optional) Add Outliers to Current
            measurementCurrentGt.addClusters(olPercentage, 2, ...
                'Noise', gtsam.noiseModel.Diagonal.Sigmas(p.OlSpreadStd(:)), ...
                'Seed', defaultSeed + 478454, 'Ids', -1);
        end
        
        % If we want to have random noise models, we change it now.
        if p.doNoisyMeasurementNoise
            pp = p.noisyMeasurementNoiseProps;
            pp_seed_idx = find(contains(pp(1:2:end),'seed'))*2;
            pp{pp_seed_idx} = defaultSeed + 270 + i;
            measurementCurrentGt.addNoiseModel(pp, 'NoiseType', 'NoisyNoise', 'CoordinateSystem', 'Polar');
            pp{pp_seed_idx} = defaultSeed + 45270 + i;
            measurementPreviousGt.addNoiseModel(pp, 'NoiseType', 'NoisyNoise', 'CoordinateSystem', 'Polar');
        else
            measurementCurrentGt.addNoiseModel([3*pi/180 0.2].^2,...
                'NoiseType', 'Constant', 'CoordinateSystem', 'Polar');
            measurementPreviousGt.addNoiseModel([3*pi/180 0.2].^2,...
                'NoiseType', 'Constant', 'CoordinateSystem', 'Polar')
        end

        % Get noisy measurement
        previous = measurementPreviousGt.getNoisyCartesian(p.measurementSeed+i);
        current = measurementCurrentGt.getNoisyCartesian(p.measurementSeed/2*i);

        % Add Ground truth and additional information to the result struct
        experiment(i,iLmSet).previousGt = measurementPreviousGt;
        experiment(i,iLmSet).previous = previous;
        experiment(i,iLmSet).currentGt = measurementCurrentGt;
        experiment(i,iLmSet).current = current;
        experiment(i,iLmSet).numCorrespondencesGt = length(previous.getCorrespondenceIds(current));
        experiment(i,iLmSet).gt = [poseBetween(i).x, poseBetween(i).y, poseBetween(i).theta]';
        experiment(i,iLmSet).init = init(i,:)'; %x,y,theta

    end

end
fprintf('Dataset generation took %0.2d seconds.\n', toc(tStart));

%% Check if we have enough correspondences to solve the problem
corr = reshape([experiment.numCorrespondencesGt],size(experiment));
if any(min(corr) < 5)
    error('There are generated Problems with less than 5 correspondences!');
end

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
    addParameter(p,'yrange', [-1 1], @isnumeric);
    addParameter(p,'thetarange', [-45 45]*pi/180, @isnumeric);
    addParameter(p,'seed', 24513, @isnumeric);
    parse(p, varargin{:});
    p = p.Results;
    [r1,r2,r3] = RandStream.create('mrg32k3a','Seed',p.seed,'NumStreams',3);
    
    x = (p.xrange(2)-p.xrange(1)).*rand(r1,p.numTransformations,1) + p.xrange(1);
    y = (p.yrange(2)-p.yrange(1)).*rand(r2,p.numTransformations,1) + p.yrange(1);
    theta = (p.thetarange(2)-p.thetarange(1)).*rand(r3,p.numTransformations,1) + p.thetarange(1);
    
    t = arrayfun(@(x,y,theta)gtsam.Pose2(x,y,theta),x,y,theta);
end
