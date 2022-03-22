classdef EvalPsr2DExperiment < libmix4sam.registration.EvalExperiment
%EVALPSR2DEXPERIMENT A collection of evaluation possibilities for a 2D PSR
%   scenario.
%
%   See also CREDIBILITYTESTS.

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
        DoF = 3;
    end
    
    methods
        function self = EvalPsr2DExperiment()
        %EVALPSR2DEXPERIMENT Construct an instance of this class.

        end
        
        function procCredibilityPerSet(self)
        %PROCCREDIBILITYPERSET Needed to evaluate metrics regarding credibility.
        %   Combine experiments into evaluation classes for consistency
        %   analysis. This time for each landmark configuration separately.
        
            classFromExperiment = @(e,iLM) CredibilityTests(...
                [e.data(:,iLM).mean]',[e.data(:,iLM).gt]',...
                'EstimationError', ...   % We calculate it here because the Credibility Class is not aware of angles in the last column
                   toNumeric(SE2Type([e.data(:,iLM).mean]')-SE2Type([e.data(:,iLM).gt]')), ... 
                'EstimatorMSE',reshape([e.data(:,iLM).covar],3,3,[]),...
                'Name',e.name);
           
            % Consider only results with covar
            isCovar = find(arrayfun(@(x)isfield(x.data,'covar'),self.Results));
            
            for iLM = 1:size(self.Results(isCovar(1)).data, 2)
                self.CredibilityPerSet(iLM) = CredibilityTestsPlot(classFromExperiment(self.Results(1),iLM));
                for iResult=2:length(isCovar)
                    if ~isfield(self.Results(isCovar(iResult)).data,'covar'), continue; end
                    self.CredibilityPerSet(iLM).addComparison(classFromExperiment(self.Results(isCovar(iResult)),iLM)); 
                end
            end              
        end
        
    end
    
    methods (Static)
                
        function visProblem(previous, current, estimation, gt)
        %VISPROBLEM Helper function to visualize one problem
        
            reg = [previous, current];
            h = reg.plotAsRegistrationProblem('Name','1) Registration Problem','Transformation',gt);
            axis(gca(h),'equal'); grid(gca(h),'on'); box(gca(h),'on');

            % Plot with registered targets by GT
            tmp = current.copy(); 
            tmp.transform( struct('x',gt(1),'y',gt(2),'theta',gt(3)) );
            reg = [previous, tmp];
            reg.plotAsRegistrationProblem('Name','2) Transformed with Ground Truth');
            % Plot with registered targets by Estimation
            tmp = current.copy(); 
            tmp.transform( struct('x',estimation(1),'y',estimation(2),'theta',estimation(3)) );
            reg = [previous, tmp];
            reg.plotAsRegistrationProblem('Name','3) Transformed with Estimation');
            
        end
        
        function [estimation, experiment] = solveVisProblem(seed)
        %SOLVEVISPROBLEM Visualize and solve one Problem with specific
        %   transformation.
        %   TODO: make generic.
        
            if nargin < 1, seed = 123; end
            if isempty(gcp('nocreate')), parpool('local',1); end % We need only one thread for plotting!
            [experiment, experimentProps] = libmix4sam.registration.genPsr2DProblem(seed, ...
                'numLandmarkSets',1,'numMonteCarloExperiments',1,...
                'transformation',[0.5 0.0 15*pi/180]);
            estimation = libmix4sam.registration.runPsr2DExperiment(experiment,...
                'gmmImplementation','MaxSumMix','experimentProps',experimentProps);
            libmix4sam.registration.EvalPsr2DExperiment.visProblem(experiment.previous, experiment.current, estimation.mean, experiment.gt);
        end
        
    end
end

