function [estimation, graph, p] = optimizeRadarProblem(radar_previous, radar_current, varargin)
%OPTIMIZERADARPROBLEM Solve a general radar-registration problem in 2D.
%
% Compatibility to old call of function:
% use p_args = reshape([fieldnames(p),struct2cell(p)]',1,[]);
% to convert to varargin parameter
%
%   See also GENRADARPROBLEM, RUNRADAREXPERIMENT.

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

p = inputParser;
addOptional(p,'init',[0;0],@(x)assert(length(x)==2,'Initialization must be a vector of two!'));

% Switch between GMM-Implementations
%  1 - MaxMix
%  2 - SumMix
%  3 - MaxSumMix
validFct = @(x)assert(ismember(x,{'MaxMix','SumMix','MaxSumMix'}),...
    'Unknown gmmImplementation given!');
addParameter(p,'gmmImplementation', 'MaxMix', validFct);

% Use GMM simplification (Calculates a distance metric between the
% components of the reference point set and merges similar components to
% one.)
addParameter(p,'useSimplification', false, @islogical);

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

% Use different scaling for dopper (only works, if useScaling is > 0)
% The doppler scaling is then useScaling * additionalDopplerScaling
validFct = @(x)validateattributes(x,{'numeric'},{'>=',0});
addParameter(p,'additionalDopplerScaling', 1, validFct);

% Depending on how good the initialization for our optimization algorithm
% is, the result will get trapped in a wrong local minima. For the
% credibility test, we calculate the true covariance (actualMSE) based on
% the estimation errors. If the estimator returns wrong results because of
% local minima, the actualMSE will also be wrong. A possible solution for
% some test cases can be to use the ground truth initialization first to
% precalculate the actualMSE and use this for Credibility Test calculations
% for the results with difficult initialization.
addParameter(p,'usePrecalculatedMSE', false, @islogical);


% Adding an outlier distribution for each target with the given weight. The
% value stands in relation to the wohle mixture of inliers and outliers.
% Meaning, the summed weight of all inliers is 1-outlierWeight and the
% summed weight of all outlier components is outlierWeight accordingly.
%  0-9 ... entspricht 0 bis 9%
validFct = @(x)validateattributes(x,{'uint8'},{'>=',0,'<',100});
addParameter(p,'outlierWeight', uint8(20), validFct);

addParameter(p,'useDoppler', false, @islogical);

validFct = @(x)validateattributes(x,{'uint8'},{'>=',0,'<',100});
addParameter(p,'outlierWeightDoppler', uint8(5), validFct);

% Calculate the numerical hessian as additional covariance information.
addParameter(p,'doNumericalHessian', false, @islogical);

% Optimizer type
validFct = @(x)assert(ismember(x,{'LevenbergMarquardt','Dogleg'}),...
    'Can not recognize the choosen optimizer!');
addParameter(p,'optimizer','LevenbergMarquardt', validFct);

% Additional parameters for the choosen optimizer
% - Verbosity: NONE ERROR VALUES DELTA LINEAR TERMINATION
addParameter(p,'optimizerParams',{'Verbosity','NONE'}, @iscell);

parse(p,varargin{:});
p = p.Results;

lmk_fixed = radar_previous.CartesianCalibrated;
lmk_current = radar_current.CartesianCalibrated;

%% Prepare for a second optimization problem, if scaling is used
if p.useScaling > 0
    scaling = p.useScaling^2; 
    scaledGraph = gtsam.NonlinearFactorGraph();
else
    scaling = 1; 
end

%% Output
estimation = struct('mean', zeros(2,1), 'covar', eye(2), 'iterations',uint64(0), 'error', 0);
if p.doNumericalHessian, estimation.numericalCovar = eye(3); end

%% Initialize variables and the nonlinear factor graph for optimization
initialEstimate = gtsam.Values;
initialEstimate.insert(1, gtsam.Point2(p.init(:)));
graph = gtsam.NonlinearFactorGraph();

%% Model fixed Pointset as gaussian mixture type
refPdf = libmix4sam.Mixture();

nFixedPoints = length(lmk_fixed.x);
for j=1:nFixedPoints
    % One target from reference measurement, modelled as GMM-component.
    refNoise = gtsam.noiseModel.Gaussian.Covariance(lmk_fixed.P(:,:,j));
    refPdf.add(libmix4sam.MixComponent( refNoise, 1/nFixedPoints, [lmk_fixed.x(j) lmk_fixed.y(j)]' ));
end

%% (optional) Simplify the reference GMM
if p.useSimplification
    refPdfClustered = libmix4sam.MixtureClustered(refPdf); 
    %[d,i] = refPdfClustered.distBhattacharyya()
    refPdfClustered.simplify(0.2); % threshold
    refPdf = refPdfClustered.getAsMixture();
end

%% (optional) Add outlier probability to the fixed
oWeight = double(p.outlierWeight) / 100.0; % Influence of Outlier probability

% Add Outlier probability
if oWeight > 0
    rOutl = libmix4sam.sensors.Radar.generateOutlierDistribution(0.7, 1.8, [0.5 50], 80*pi/180);
    outlierPdf = rOutl.getMixture('norm');
    refPdf.add(outlierPdf, oWeight);
end

%% Based on the GMM, create a noise model implementation

refNoiseModel = libmix4sam.noiseModelNew.(p.gmmImplementation).Create(refPdf);

%% Model current Pointset
nFixedPoints = length(lmk_current.x);
for iPoint = 1:nFixedPoints
    currMean = gtsam.Point2(lmk_current.x(iPoint), lmk_current.y(iPoint));
    currNoise = gtsam.noiseModel.Gaussian.Covariance(lmk_current.P(:,:,iPoint));
    graph.add( libmix4sam.PsrRadarPriorFactor(1, ...
        currMean, gtsam.Point2(0,0), ...
        refNoiseModel, currNoise));
end

if p.useScaling > 0
    for iPoint = 1:nFixedPoints
        currMean = gtsam.Point2(lmk_current.x(iPoint), lmk_current.y(iPoint));
        currNoise = gtsam.noiseModel.Gaussian.Covariance(scaling*lmk_current.P(:,:,iPoint));
        scaledGraph.add( libmix4sam.PsrRadarPriorFactor(1, ...
            currMean, gtsam.Point2(0,0), ...
            refNoiseModel, currNoise));
    end
end

%% Add Doppler Velocity Factor
if p.useDoppler
    targets_doppler = [radar_current.Phi radar_current.Doppler];
    targets_doppler_std = [radar_current.StdPhi radar_current.StdDoppler];
    time_cycle_std = radar_current.StdCycleTime;

    if p.outlierWeightDoppler > 0
        dWeight = double(p.outlierWeightDoppler)/100;
        dWeight = [1-dWeight dWeight];
    else
        dWeight = [];
    end
    
    n = size(targets_doppler,1);
    for i=1:n    
        %OUTLIER! (dynamic targets)

        dopplerNoise = getDopplerNoiseModel([targets_doppler_std(i,2), time_cycle_std], dWeight);
        phiNoise = gtsam.noiseModel.Isotropic.Sigma(1,targets_doppler_std(i,1));
        if radar_current.hasCalibration()
            graph.add(libmix4sam.RadarDopplerFactor(1, targets_doppler(i,2),targets_doppler(i,1), dopplerNoise, phiNoise, radar_current.CycleTime, radar_current.getT_BS()));
        else
            graph.add(libmix4sam.RadarDopplerFactor(1, targets_doppler(i,2),targets_doppler(i,1), dopplerNoise, phiNoise, radar_current.CycleTime));
        end
    end
    
    if p.useScaling > 0
        dopplerScaling = sqrt(scaling) * p.additionalDopplerScaling;
        for i=1:n    
            dopplerNoise = getDopplerNoiseModel([targets_doppler_std(i,2) * dopplerScaling, time_cycle_std], dWeight);
            phiNoise = gtsam.noiseModel.Isotropic.Sigma(1,targets_doppler_std(i,1) * dopplerScaling);
            if radar_current.hasCalibration()
                scaledGraph.add(libmix4sam.RadarDopplerFactor(1, targets_doppler(i,2),targets_doppler(i,1), dopplerNoise, phiNoise, radar_current.CycleTime, radar_current.getT_BS()));
            else
                scaledGraph.add(libmix4sam.RadarDopplerFactor(1, targets_doppler(i,2),targets_doppler(i,1), dopplerNoise, phiNoise, radar_current.CycleTime));
            end
        end
    end
end

%% (optional) Get a new initial estimate by optimizing with scaled covariances.
if p.useScaling > 0
    scaledOptimizerParameters = feval(['gtsam.' p.optimizer 'Params']);
    for iParameter = 1:2:length(p.optimizerParams)
        feval(['set' p.optimizerParams{iParameter}], scaledOptimizerParameters, p.optimizerParams{iParameter+1});
    end
    scaledOptimizerParameters.setMaxIterations(5);
    scaledOptimizer = feval(['gtsam.' p.optimizer 'Optimizer'], scaledGraph, initialEstimate, scaledOptimizerParameters);
    initialEstimate = scaledOptimizer.optimizeSafely();
end

%% Optimize
optimizerParameters = feval(['gtsam.' p.optimizer 'Params']);
for iParameter = 1:2:length(p.optimizerParams)
    feval(['set' p.optimizerParams{iParameter}], optimizerParameters, p.optimizerParams{iParameter+1});
end
optimizer = feval(['gtsam.' p.optimizer 'Optimizer'], graph, initialEstimate, optimizerParameters);

result = optimizer.optimizeSafely();

estimation.error = optimizer.error;
estimation.iterations = optimizer.iterations;
if p.useScaling > 0, estimation.iterations = estimation.iterations + scaledOptimizer.iterations; end
%estimation.mean = result.atVector(1); % OLD FACTOR
if isnumeric(gtsam.Point2(0,0)) % Compatibility between old and new gtsam
    estimation.mean = result.atPoint2(1);
else
    estimation.mean = result.atPoint2(1).vector;
end

M = gtsam.Marginals(graph, result);
estimation.covar = M.marginalCovariance(1);

if p.doNumericalHessian
    H = numericalHessian(graph, result);
    estimation.numericalCovar = inv(H);
end


end

%% Helper Functions

function dopplerNoise = getDopplerNoiseModel(doppler_std, dWeight)
    if ~isempty(dWeight)
        % Noise model with outlier probability
        dopplerNoise = libmix4sam.Mixture();
        dopplerNoise.add(libmix4sam.MixComponent( ...
            gtsam.noiseModel.Diagonal.Sigmas([doppler_std(1); doppler_std(2)]),... % sigma_v and sigma_t
            dWeight(1) ));
        dopplerNoise.add(libmix4sam.MixComponent( ...                               % Outlier component
            gtsam.noiseModel.Diagonal.Sigmas([100; doppler_std(2)]),...
            dWeight(2) )); 
        dopplerNoise = libmix4sam.noiseModelNew.MaxMix.Create(dopplerNoise);
    else
        % Simple noise model
        dopplerNoise = gtsam.noiseModel.Diagonal.Sigmas([doppler_std(1); doppler_std(2)]); % sigma_v and sigma_t
    end
end

function [H, err_H] = numericalHessian(nonlinFg,result)
% Calculation of hessian with gtsam is not correct!
vector = result.atVector(1);
[H,err_H] = hessian(@(x)computeError(nonlinFg,result,x),vector);
end

function err = computeError(g,v,x)
    vv = gtsam.Values(v);
    vv.update(1, [x(1); x(2)]);
    err = g.error(vv);
end
