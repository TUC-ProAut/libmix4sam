function [estimation, graph] = optimizePsr2DProblem(lmkFixed, lmkCurrent, varargin)
%OPTIMIZEPSR2DPROBLEM Solve a general 2D Point-Set-Registration problem.
%   
%   See also GENPSR2DPROBLEM, RUNPSR2DEXPERIMENT.

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

% Initialization for optimization (starting point)
validFct = @(x)assert(isa(x,'gtsam.Pose2'),...
    'Initialization must be of type gtsam.Pose2!');
addParameter(p,'init', gtsam.Pose2(), validFct);

% Switch between GMM-Implementations
%  1 - MaxMix
%  2 - SumMix
%  3 - MaxSumMix
validFct = @(x)assert(ismember(x,{'MaxMix','SumMix','MaxSumMix'}),...
    'Unknown gmmImplementation given!');
addParameter(p,'gmmImplementation', 'MaxMix', validFct);

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

% Calculate the covariance matrix numerically for comparison. 
% The following additional toolbox has to be within the path for this:
% John D'Errico (2020). Adaptive Robust Numerical Differentiation 
% (https://www.mathworks.com/matlabcentral/fileexchange/13490-adaptive-robust-numerical-differentiation), 
% MATLAB Central File Exchange. Retrieved June 30, 2020. 
addParameter(p,'doNumericalHessian', false, @islogical);

% Optimizer type
validFct = @(x)assert(ismember(x,{'LevenbergMarquardt','Dogleg','GaussNewton'}),...
    'Can not recognize the choosen optimizer!');
addParameter(p,'optimizer','LevenbergMarquardt', validFct);

% Additional parameters for the choosen optimizer
% - Verbosity: NONE ERROR VALUES DELTA LINEAR TERMINATION
addParameter(p,'optimizerParams',{'Verbosity','NONE'}, @iscell);

% Optional parameters for using outlier distribution.
validFct = @(x)validateattributes(x,{'numeric'},{'>=',0,'<',1});
addParameter(p, 'outlierWeight', 0, validFct);
addParameter(p, 'outlierPdf', [], @(x)isa(x,'libmix4sam.Mixture'));

parse(p,varargin{:});
p = p.Results;

%% Prepare for a second optimization problem, if scaling is used
if p.useScaling > 0
    scaledLmkCurrent = P2Noise(lmkCurrent, p.useScaling^2); 
    scaledGraph = gtsam.NonlinearFactorGraph();
end

%% Prepare input
% If no noise model is given within lmk, try to convert from Covariance
% matrix (Has to be done after calculating the scaledLmkCurrent version!).
lmkFixed = P2Noise(lmkFixed);
lmkCurrent = P2Noise(lmkCurrent);

%% Output
estimation = struct('mean', zeros(3,1), 'covar', eye(3), 'iterations', 0, 'error', 0);
if p.doNumericalHessian, estimation.numericalCovar = eye(3); end

%% Initialize variables and the nonlinear factor graph for optimization
initialEstimate = gtsam.Values;
initialEstimate.insert(1, p.init);
graph = gtsam.NonlinearFactorGraph();
    
%% Model fixed point-set as Gaussian mixture type
refPdf = libmix4sam.Mixture();

nFixedPoints = size(lmkFixed.points, 1);
for iPoint=1:nFixedPoints
    % One target from reference measurement, modelled as GMM-component.
    refPdf.add(libmix4sam.MixComponent( lmkFixed.noiseModel{iPoint}, 1/nFixedPoints, lmkFixed.points(iPoint,:)' ));
end

%% (optional) Add outlier probability to the fixed
if p.outlierWeight > 0
    assert(~isempty(p.outlierPdf),'If outlierWeight is >0, an outlierPdf has to be given!');
    % Add the mixture distribution of outliers to the fixed point set pdf
    % The overal weight of the fixed point-set GMM is scaled down to 1-outlierWeight
    % and the overal weight of the outlier GMM is scaled down to outlierWeight
    refPdf.add(p.outlierPdf, p.outlierWeight);
end

%% Based on the GMM, create a noise model implementation
refNoiseModel = libmix4sam.noiseModelNew.(p.gmmImplementation).Create(refPdf);

%% Model current pointset as D2D-Problem
nFixedPoints = size(lmkCurrent.points,1);
for iPoint = 1:nFixedPoints
    currMean = gtsam.Point2(lmkCurrent.points(iPoint,:)');
    graph.add( libmix4sam.Psr2DPriorFactor(1, ...
        currMean, gtsam.Point2(0,0), ...
        refNoiseModel, lmkCurrent.noiseModel{iPoint}));
end

if p.useScaling > 0
    for iPoint = 1:nFixedPoints
        currMean = gtsam.Point2(scaledLmkCurrent.points(iPoint,:)');
        scaledGraph.add( libmix4sam.Psr2DPriorFactor( 1, ...
            currMean, gtsam.Point2(0,0), ...
            refNoiseModel, scaledLmkCurrent.noiseModel{iPoint}));
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
pose2num = @(x)[x.x x.y x.theta]';
estimation.mean = pose2num(result.atPose2(1));

M = gtsam.Marginals(graph, result);
estimation.covar = M.marginalCovariance(1);

if p.doNumericalHessian
    H = numericalHessian(graph, result);
    estimation.numericalCovar = inv(H);
end

end

%% Helper Functions
function lmk = P2Noise(lmk, scaling)
% Check for noise model. If not given, check for other noise formats.
    if nargin < 2, scaling = 1; end
    
    if ~isfield(lmk,'noiseModel')
        P = [];
        
        if isfield(lmk,'P'), P = num2cell(lmk.P, [1 2]); end
        if isfield(lmk,'Covariance'), P = num2cell(lmk.Covariance, [1 2]); end
        assert(~isempty(P),'No covariance found for given points!');
        
        P = num2cell(scaling * lmk.P, [1 2]);
        lmk.noiseModel = cellfun(@(x)gtsam.noiseModel.Gaussian.Covariance(x), P(:),'UniformOutput',false);
        % UniformOutput false is neccessary, because gtsam can return
        % different class types! If only main diagonal is present, a
        % diagonal noise type is returned and matlab throws an error, if
        % also non-diagonal types are present in the array.
    else
        assert(scaling == 1,'Noise Model can not be scaled!');
    end
end

function [H, err_H] = numericalHessian(nonlinFg,result)
    pose2num = @(x)[x.x x.y x.theta]';
    fun = @(x)computeError(nonlinFg,result,x);
    [H, err_H] = hessian(fun, pose2num(result.atPose2(1)));
end

function err = computeError(g,v,x)
    vv = gtsam.Values(v);
    vv.update(1, gtsam.Pose2([x(1); x(2); x(3)]));
    err = g.error(vv);
end
