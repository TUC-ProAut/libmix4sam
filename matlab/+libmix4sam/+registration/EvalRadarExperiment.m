classdef EvalRadarExperiment < libmix4sam.registration.EvalExperiment
%EVALRADAREXPERIMENT A collection of evaluation possibilities for a radar
%   scenario.
%
%   See also CREDIBILITYTESTS, CREDIBILITYRADAR.

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

    properties (Constant)
        DoF = 2;
    end
    
    methods
        function self = EvalRadarExperiment()
        %EVALRADAREXPERIMENT Construct an instance of this class.

        end
        
        function ret = getResultMeanAsMatrix(self,resultNumber)
        %GETRESULTMEANASMATRIX Needs overload, because in radar data, only
        %   x and theta are estimated.
            ret = [self.Results(resultNumber).data(:).mean]';
            ret(:,3) = ret(:,2); ret(:,2) = 0;
        end
        
        function procCredibility(self)
        %PROCCREDIBILITY Extends base class with specific credibility class
        %   for radar.
            
            % Specific credibility class for radar
            self.CredibilityRadar = CredibilityRadarPlot(...
                self.credibilityClassFromExperiment(self.Results(1)));
            for iResult = 2:length(self.Results)
                self.CredibilityRadar.addComparison(...
                    self.credibilityClassFromExperiment(self.Results(iResult))); 
            end
            
            % General credibility class
            procCredibility@libmix4sam.registration.EvalExperiment(self);
        end
        
        function estimateCorrespondences(self)
        %ESTIMATECORRESPONDENCES Estimate correspondences by using
        %   bhattacharyyaDist and ground truth translation
            if isfield(self.Experiment,'gt'), hasGt = true; else, hasGt = false; end
            for iExperiment = 1:length(self.Experiment)
                gt = [0,0,0];
                if hasGt, gt = self.Experiment(iExperiment).gt; end
                [pID, cID] = libmix4sam.sensors.Radar.EstimateCorrespondenceIds(...
                    self.Experiment(iExperiment).previous, ...
                    self.Experiment(iExperiment).current, ...
                    gt);
                self.Experiment(iExperiment).current.addIDs(cID);
                self.Experiment(iExperiment).previous.addIDs(pID);
                self.Experiment(iExperiment).numCorrespondencesGt = ...
                    length(self.Experiment(iExperiment).previous.getCorrespondenceIds(...
                    self.Experiment(iExperiment).current));
            end
        end
        
    end
    
    methods (Static)
        
        function err = estimationError(e)
        %ESTIMATIONERROR Calculate the error between mean and gt of a
        %   result.
            mean = [e.data(:).mean]';
            mean = [mean(:,1) zeros(size(mean,1),1) mean(:,2)];
            err = toNumeric(SE2Type(mean) - SE2Type([e.data(:).gt]'));
            err = [err(:,1) err(:,3)];
        end
        
        function gt = gt2matrix(e)
        %ADAPTGT Convert ground truth into correct format depending on DoF.
            gt = [e.data(:).gt]';
            gt = [gt(:,1) gt(:,3)];
        end
        
        function [estimation, experiment] = solveVisProblem(seed)
        % Visualize and solve one Problem with specific transformation
            if nargin < 1, seed = 123; end
            if isempty(gcp('nocreate')), parpool('local',1); end % We need only one thread for plotting!
            [experiment, experimentProps] = libmix4sam.registration.genRadarProblem(seed, ...
                'numLandmarkSets',1,'numMonteCarloExperiments',1,...
                'transformation',[0.5 0.0 15*pi/180],...
                'landmarks',struct('type', 'circle', 'size', [20 35], 'amount', 6, 'seed', seed + 90));
            estimation = libmix4sam.registration.runRadarExperiment(experiment,...
                'gmmImplementation','MaxSumMix','experimentProps',experimentProps);
            libmix4sam.registration.EvalRadarExperiment.visProblem(experiment.previous, experiment.current, estimation.mean, experiment.gt);
        end
        
        function visProblem(previous, current, estimation, gt)
        % Helper Function to Visualize one Problem
        
            h = figure('Name','1) Registration Problem');
            libmix4sam.sensors.Radar.plotRegistration(gca(h),previous,[],current,[],[0 0 0]');

            h = figure('Name','2) Transformed with Ground Truth');
            libmix4sam.sensors.Radar.plotRegistration(gca(h),previous,[],current,[],gt);

            h = figure('Name','3) Transformed with Estimation');
            libmix4sam.sensors.Radar.plotRegistration(gca(h),previous,[],current,[],[estimation(1) 0 estimation(2)]');

        end
        
    end
end

