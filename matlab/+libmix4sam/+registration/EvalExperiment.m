classdef (Abstract) EvalExperiment < matlab.mixin.Copyable
%EVALEXPERIMENT Base class for evaluation classes.
%
%   See also EVALPSR2DEXPERIMENT, EVALRADAREXPERIMENT.

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

    properties(Abstract, Constant)
        DoF;   % Degrees of Freedom
    end
    
    properties (SetAccess = protected)
        Accuracy = [];
        Credibility = [];
        CredibilityRadar = [];
        Experiment = [];
        Results = [];
    end
    
    methods
        function self = EvalExperiment()
        %EVALEXPERIMENT Construct an instance of this class.
        
        end
        
        function addExperimentData(self, experiment)
        %ADDEXPERIMENTDATA Add the data of the experiment itself.
        %   Only one experiment is possible.
            self.Experiment = experiment;
        end
        
        function addResultData(self, experiments)
        %ADDRESULTDATA Add a result for evaluation.
        %   This method can be called multiple times with different
        %   results.
            if isempty(self.Results)
                self.Results = experiments;
            else
                self.Results = [self.Results experiments];
            end
        end
        
        function procAccuracy(self, addDoNothing)
        %PROCACCURACY Needed to evaluate metrics regarding accuracy.
        
            if nargin < 2, addDoNothing = false; end
            
            % We use the Class MetricsSE2Plot for error analysis 
            % and initialize it with our ground truth
            self.Accuracy = MetricsSE2Plot(...
                SE2Type([self.Experiment.gt]','Name','Reference'));
            
            % We add the different results for comparison
            for iResult = 1:length(self.Results)
                tmp = self.getResultMeanAsMatrix(iResult);
                self.Accuracy.addData(...
                    SE2Type(tmp, 'Name', self.Results(iResult).name));
            end
            
            % How good are we, if we do nothing?
            if addDoNothing
                self.Accuracy.addData(...
                    SE2Type(zeros(size([self.Results(1).data(:).mean]',1), 3),'Name','DoNoting'));
            end
            
        end
        
        function procCredibility(self)
        %PROCCREDIBILITY Needed to evaluate metrics regarding credibility.
        %   Combine experiments into evaluation classes for consistency
        %   analysis.
            
            % Consider only Results with covar
            isCovar = find(arrayfun(@(x)isfield(x.data,'covar'),self.Results));
            
            % General credibility class
            self.Credibility = CredibilityTestsPlot(...
                self.credibilityClassFromExperiment(self.Results(isCovar(1))));
            for iResult = 2:length(isCovar)
                if ~isfield(self.Results(isCovar(iResult)).data,'covar'), continue; end
                self.Credibility.addComparison(...
                    self.credibilityClassFromExperiment(self.Results(isCovar(iResult)))); 
            end
            
        end
        
        function ret = getResultMeanAsMatrix(self,resultNumber)
        %GETRESULTMEANASMATRIX Convert the estimation result into matrix
        %   type.
            ret = [self.Results(resultNumber).data(:).mean]';
        end
        
        function v = credibilityClassFromExperiment(self,e)
        %CREDIBILITYCLASSFROMEXPERIMENT Initialize a CredibilityTests class
        %   from experiment data for consistency analysis.
            dof = self.DoF;
            estimationError = self.estimationError(e);
            gt = self.gt2matrix(e);
            v = CredibilityTests(...
                [e.data(:).mean]',gt,...
                'EstimationError', estimationError, ...   % We calculate it here because the Credibility Class is not aware of angles in the last column
                'EstimatorMSE',reshape([e.data(:).covar],dof,dof,[]),...
                'Name',e.name);
            v.DegreeOfFreedom = dof;
        end
        
        function rpg = toXYZqxqyqzqw(self)
        %TOXYZQXQYQZQW Convert to format for rpg_trajectory_evaluation.
        %   For the evaluation toolbox, see:
        %   https://github.com/uzh-rpg/rpg_trajectory_evaluation
        
            rpg = struct('Name',{},'Gt',{},'Result',{});
            
            %stamped_groundtruth.txt
            gt = SE2Type([self.Experiment.gt]','Name','Reference');
            gt = gt.cumsum();
            gt = [arrayfun(@(x)x.current.Time,self.Experiment), gt.toXYZqxqyqzqw()];
            
            %stamped_traj_estimate.txt
            for iResult = 1:length(self.Results)
                tmp = self.getResultMeanAsMatrix(self, iResult);
                t = SE2Type(tmp, 'Name', self.Results(iResult).name);
                t = t.cumsum();
                rpg(iResult).Name = self.Results(iResult).name;
                rpg(iResult).Gt = gt;
                rpg(iResult).Result = [arrayfun(@(x)x.current.Time,self.Experiment), t.toXYZqxqyqzqw()];
            end
        end
        
        function TAdditional = tableAdditional(self)
        %TABLEADDITIONAL Create table with timing and other additional
        %   results. 
        
            TAdditional = table(...
                string({self.Results.name}'),...
                arrayfun(@(x)length(x.data(:)),self.Results)',...
                round([self.Results.timing]'),...
                arrayfun(@(x)x.timing/length(x.data(:)),self.Results)',...
                round(arrayfun(@(x)mean([x.data(:).iterations]),self.Results)',1),...
                [arrayfun(@(x)min([x.data(:).iterations]),self.Results)' ...
                 arrayfun(@(x)max([x.data(:).iterations]),self.Results)'],...
                'VariableNames',{'Name','Experiments','AbsTime','AverageTime','Iterations','It. min/max'});
        end
                
        function h = plotOutlierAnalysis(self)
        %PLOTOUTLIERANALYSIS Analyze percentage of outliers within data.
            
            % Outliers in current1
            %arrayfun(@(x)x.current.getSize-x.numCorrespondencesGt,self.Experiment);
            % Outlier-percentage in current
            %arrayfun(@(x)1-(x.numCorrespondencesGt/x.current.getSize),self.Experiment);

            bp = struct();
            bp.NoCorrespondences = [self.Experiment.numCorrespondencesGt];
            bp.NoOutliersPrevious = arrayfun(...
                @(x)x.previous.getSize - x.numCorrespondencesGt, self.Experiment);
            bp.NoOutliersCurrent = arrayfun(...
                @(x)x.current.getSize - x.numCorrespondencesGt, self.Experiment);

            h = figure('Name','Target numbers');
            boxplot([bp.NoCorrespondences(:), bp.NoOutliersCurrent(:), bp.NoOutliersPrevious(:)],...
                'Labels',{'No.Lmks corresponding','No.Outliers current','No.Outliers previous'},...
                'Orientation','horizontal');
            title('Number of corresponding targets vs. outliers');
            h.Position(3:4) = [1500 250];
        end
        
        function plotOneProblem(self, visLMSet, visExperiment, resultSet)
        %PLOTONEPROBLEM Visualize a registration problem.
            if nargin < 4, resultSet = 1; end
            fprintf('Plotting "%s" Result for LMSet %i, Experiment %i\n', ...
                self.Results(resultSet).name, visLMSet, visExperiment);
            self.visProblem(...
                self.Experiment(visExperiment,visLMSet).previous, ...
                self.Experiment(visExperiment,visLMSet).current, ...
                self.Results(resultSet).data(visExperiment,visLMSet).mean, ...
                self.Experiment(visExperiment,visLMSet).gt);
        end
        
        function h = plotTrajectory(self)
        %PLOTTRAJECTORY Little helper, to visualize data as trajectory.
            
            % First, we have to accumulate the relative transformations
            gt = SE2Type([self.Experiment.gt]','Name','Reference');
            gt = gt.cumsum();
            
            % We use the Class MetricsSE2Plot for plotting 
            % and initialize it with our ground truth
            trajectory = MetricsSE2Plot(gt);
            
            % We add the different results for comparison
            for iResult = 1:length(self.Results)
                tmp = self.getResultMeanAsMatrix(iResult);
                t = SE2Type(tmp, 'Name', self.Results(iResult).name);
                t = t.cumsum();
                trajectory.addData(t);
            end
            
            h = trajectory.plotTrajectory();
        end
        
        function [h, hlines] = plotTargetsOverTime(self, varargin)
        %PLOTTARGETSOVERTIME Can be used to evaluate the number of targets
        %   available over time.
        %   Should be used for real world data, to locate difficult
        %   situations.
            p = inputParser;
            % Use Axes to plot, if given. Otherwise create new figure.
            validFct = @(x)isgraphics(x,'Axes');
            addOptional(p, 'hax', [], validFct);
            % Which LM-Set should be used?
            %validFct = @(x)validateattributes(x,{'numeric'},{'integer','nonnegative'});
            %addOptional(p, 'LMSet', 1, validFct);
            % Define a style for plotting
            addParameter(p, 'style', struct('Color','r'), @isstruct);
            addParameter(p, 'movmean', 15, @isnumeric);
            addParameter(p, 'useTime', true, @islogical);
            parse(p,varargin{:}); 
            
            useNewFigure = false;
            if ismember('hax',p.UsingDefaults), useNewFigure = true; end
            p = p.Results;
            
            hax = p.hax;
            if useNewFigure
                hax = gca(figure('Name','TargetsOverTime'));
                grid on; hold on; legend;
            end
            h = hax.Parent();
            if isempty(hax.XLabel.String)
                if p.useTime, xlabel(hax,'Time [s]'); else, xlabel(hax,'Steps'); end
            end
            
            ex = self.Experiment;
            n = [arrayfun(@(x)numel(x.previous.Rho),ex); numel(ex(end).current.Rho)];
            if p.useTime
                t = [arrayfun(@(x)x.previous.Time,ex); ex(end).current.Time];
                t = t-t(1);
            else
                t = 1:numel(n);
            end
            if ~isempty(p.movmean) && p.movmean > 1
                n = movmean(n, p.movmean);
            end
            
            p.style.DisplayName = sprintf('Targets movmean(%d)', p.movmean);
            hlines = plot(hax, t, n, p.style);
        end
        
        function [h, hax] = plotErrorOverTime(self)
        %PLOTERROROVERTIME Plot the error over time.
        %   Useful for real world data to discover difficult situations.
        
            h = figure('Name','Error over Time');
            style = struct('DisplayName',''); % Default style Properties
            hax = subplot(3,1,1); hold(hax,'on'); legend(hax); ylabel(hax,'No. Targets');
            hax(2) = subplot(3,1,2); hold(hax(2),'on'); legend(hax(2)); ylabel(hax(2),'PosError [m]');
            hax(3) = subplot(3,1,3); hold(hax(3),'on'); legend(hax(3)); ylabel(hax(3),'AngError [deg]');
            
            self.plotTargetsOverTime(hax(1),'useTime',false,'movmean',1,'style',struct('Color','y'));
            self.plotTargetsOverTime(hax(1),'useTime',false,'movmean',15);
            
            if isfield(self.Experiment,'numCorrespondencesGt')
                corr = [self.Experiment.numCorrespondencesGt]';
                plot(hax(1), (1:length(corr))+1, corr, 'DisplayName','Correspondences','Color','g');
            end
            
            posErr = self.Accuracy.absPositionError;
            for i=1:length(posErr)
                style.DisplayName = self.Accuracy.Data{i+1}.Name;
                plot(hax(2),1:length(posErr{i}), posErr{i}, style);
            end
            angErr = self.Accuracy.geodesicDistance;
            for i=1:length(angErr)
                style.DisplayName = self.Accuracy.Data{i+1}.Name;
                plot(hax(3),1:length(angErr{i}), angErr{i}*180/pi, style);
            end
            linkaxes([hax(1) hax(2) hax(3)],'x');
        end
        
        function [h] = plotTargetsHist(self,varargin)
        %PLOTTARGETSHIST Plots a histogram over the number of available
        %   targets.
        
            p = inputParser;
            % Use Axes to plot, if given. Otherwise create new figure.
            validFct = @(x)isgraphics(x,'Axes');
            addOptional(p, 'hax', [], validFct);
            parse(p,varargin{:}); 
            
            useNewFigure = false;
            if ismember('hax',p.UsingDefaults), useNewFigure = true; end
            p = p.Results;
            
            hax = p.hax;
            if useNewFigure
                hax = gca(figure('Name','TargetNumbers'));
                grid on; hold on; legend;
            end
            h = hax.Parent();
            
            ex = self.Experiment;
            n = [arrayfun(@(x)numel(x.previous.Rho),ex); numel(ex(end).current.Rho)];
            histogram(n,'DisplayName','No. Targets per Measurement');
        end
        
    end
    
    methods (Static)
        
        function err = estimationError(e)
        %ESTIMATIONERROR Calculate the error between mean and gt of a
        %   result.
            err = toNumeric(SE2Type([e.data(:).mean]') - SE2Type([e.data(:).gt]'));
        end
        
        function gt = gt2matrix(e)
        %ADAPTGT Convert ground truth into correct format depending on DoF.
            gt = [e.data(:).gt]';
        end
    end
end

