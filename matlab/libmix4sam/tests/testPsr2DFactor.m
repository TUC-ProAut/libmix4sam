% This file implements a simple example to test the robust point set 
% registration in 2D.

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

% Compatibility to old and new gtsam matlab wrapper
isPoint2 = ~isnumeric(gtsam.Point2(0,0));

initialEstimate = gtsam.Values;
initialEstimate.insert(1, gtsam.Pose2(0,0,0));
graph = gtsam.NonlinearFactorGraph();

% Define some targets for the previous frame
fixed = [10 1; 20 -1; 30 10; 30 -10]';

% Define a ground truth transformation and transform the previous targets 
% into the current frame.
T = gtsam.Pose2(0.25, -0.25, 0.18).inverse;
if isPoint2
    current = (T.rotation.matrix * fixed + T.translation.vector);
else
    current = (T.rotation.matrix * fixed + T.translation);
end

% Model fixed Pointset as gaussian mixture type
refPdf = libmix4sam.Mixture();
for j=1:size(fixed,2)
    % One target from reference measurement, modelled as GMM-component.
    refNoise = gtsam.noiseModel.Diagonal.Sigmas([0.2; 0.3]);
    refPdf.add(libmix4sam.MixComponent( refNoise, 1/size(fixed,2), fixed(:,j)));
end

% Use the GMM to instantiate a robust noise model (here we use MaxSumMix).
refNoiseModel = libmix4sam.noiseModelNew.MaxSumMix.Create(refPdf);

% Add each target of current point set to the graph with mixture 
% distribution of the fixed set as noise model.
for iPoint = 1:size(current,2)
    currNoise = gtsam.noiseModel.Diagonal.Sigmas([0.2; 0.3]);
    graph.add( libmix4sam.Psr2DPriorFactor(1, ...
        gtsam.Point2(current(:,iPoint)), gtsam.Point2(0,0), ...
        refNoiseModel, currNoise));
end

% Show the actual graph.
disp("================= GRAPH ==================");
graph.display();

% Optimize
fprintf("\n============== OPTIMIZATION ==============\n");
optimizerParameters = gtsam.LevenbergMarquardtParams();
% SILENT = 0, SUMMARY, TERMINATION, LAMBDA, TRYLAMBDA, TRYCONFIG, DAMPED, TRYDELTA
optimizerParameters.setVerbosityLM('SUMMARY');
optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initialEstimate, optimizerParameters);

% Show the result
result = optimizer.optimizeSafely();
fprintf("\n================= RESULT =================\n");
result.display();

% For convenience show the ground truth again
disp("Ground truth reference: ");
T.inverse().display();

% Show covariance for result
M = gtsam.Marginals(graph, result);
fprintf("\nEstimated Covariance: \n");
disp(M.marginalCovariance(1));
