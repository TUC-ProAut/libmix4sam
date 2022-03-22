classdef CredibilityTestsPlot < CredibilityTests
%CREDIBILITYTESTSPLOT Extend CredibilityTests with plotting methods.
%
%   See also CREDIBILITYTESTS.

% @author Sven Lange (TU Chemnitz, ET/IT, Prozessautomatisierung)
% @author Karim Haggag (TU Chemnitz, ET/IT, Prozessautomatisierung)

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

    properties
        TODO
    end
    
    properties (SetAccess = private)
        Comparison = [];
    end
    
    methods
        function self = CredibilityTestsPlot(varargin)
        %CREDIBILITYTESTSPLOT Construct an instance of this class.
        %   Class can be constructed either with the same input arguments
        %   as CredibilityTests or with an instance of this class.
        %   By using the superclass as initialization, additional arguments
        %   for CredibilityTestsPlot can be given starting from the second
        %   argument.
            
            if isa(varargin{1},'CredibilityTests')
                % CredibilityTests class and additional plotting parameters
                % are given.
                super_args = varargin(1);
            else
                % Only parameters for CredibilityTests class are given and
                % additional plotting parameters are given.
                super_args = varargin;
            end
            
            self@CredibilityTests(super_args{:});
            
            % Evaluate additional arguments for CredibilityTestsPlot Class only
            if isa(varargin{1},'CredibilityTests')
                sub_args = varargin(2:end);
            else
                sub_args = self.UnmatchedArguments;
            end
            
            p = inputParser;
            addParameter(p,'TODO',[],@isnumeric); 
            parse(p,sub_args{:})
            p = p.Results;
            self.TODO = p.TODO;
            
        end
        
        function T = showComparisonTable(self)
        %SHOWCOMPARISONTABLE Create table with evaluation results.
        
            [anees, aneesbound] = self.ANEES(0.95);
            if ~isempty(self.Comparison)
                T = table({self.Name self.Comparison.Name}',...
                    [self.NCIndex [self.Comparison.NCIndex]]',...
                    [self.Inclination [self.Comparison.Inclination]]',...
                    [self.TotalTestValidity [self.Comparison.TotalTestValidity]]',...
                    [self.TotalCredibilityRatio [self.Comparison.TotalCredibilityRatio]]',...
                    anees,...
                    aneesbound);
            else
                T = table({self.Name},...
                    self.NCIndex,...
                    self.Inclination,...
                    self.TotalTestValidity,...
                    self.TotalCredibilityRatio,...
                    anees,...
                    aneesbound);
            end

            % Column naming
            T.Properties.VariableNames = {...
                'Name','NCIIndex','Inclination','TotalTestValidity',...
                'TotalCredibilityRatio','ANEES','ANEESBound95'};
            
            % Format numbers (optional)
            %T.ANEES = arrayfun(@(x) sprintf("%1.3f", x),T.ANEES);
            %T.ANEESBound95 = arrayfun(@(x) sprintf("%1.3f", x),T.ANEESBound95);
        end
        
        function addComparison(self,other)
        %ADDCOMPARISON Add other result for comparison plots and tables.
        
            if isempty(self.Comparison)
                self.Comparison = other;
            else
                self.Comparison(end+1) = other;
            end
        end
        
        function [anees, bound] = ANEES(self, interval)
        %ANEES Overlay ANEES to incorporate comparison capabilities. 
            if nargin < 2, interval = 0.95; end
            [anees, bound] = ANEES@CredibilityTests(self, interval);
            if ~isempty(self.Comparison)
                for i=1:length(self.Comparison)
                    [anees(i+1,1), bound(i+1,:)] = ANEES@CredibilityTests(self.Comparison(i), interval);
                end
            end
        end
        
        function h = plotNEESComparison(self)
        %PLOTNEESCOMPARISON Plot multiple NEES histograms into subplots for
        %   better comparison.
        
            if ~isempty(self.Comparison)
                N = length(self.Comparison)+1;
                h = figure('Name','NEES for different results');
                ax = subplot(N,1,1);
                plotNEES4Estimator(self, ax(1), 'Name', self.Name);
                title(ax(1),'Empirical NEES for multiple Scenarios');
                for i=1:length(self.Comparison)
                    ax(i+1) = subplot(N,1,i+1);
                    plotNEES4Estimator(CredibilityTestsPlot(self.Comparison(i)), ax(i+1), 'Name', self.Comparison(i).Name);
                end
                linkaxes(ax,'xy');
            else
                plotNEES4Estimator(self);
                h = gcf;
            end
        end
        
        function h = plotNEES4Estimator(self, ha, varargin)
        %PLOTNEES4ESTIMATOR Plot the estimator's NEES values into a
        %   histogram.
            
            if nargin < 2
                h = figure('Name','Nees for the estimator');
                ha = gca;
            end
            
            p = inputParser;
            addParameter(p, 'Name', 'NEES4Estimator', @ischar);
            parse(p,varargin{:}); p = p.Results;
            
            props = {'BinWidth', 0.1};
            h1 = histogram(ha, self.NEES4Estimator, 100, 'Normalization', 'pdf', props{:});
            hold on;
            h1.FaceColor = ones(1,3)*0.2;
            %h1.LineStyle = 'none';
            h1.EdgeColor = ones(1,3)*0.45;
    
            % plot Chi-square probability density function with according dof 
            plot(ha, 0:0.1:15, chi2pdf(0:0.1:15, self.DegreeOfFreedom), '--r','LineWidth',1.5);

            xline(ha, self.DegreeOfFreedom,      'g-',  'LineWidth', 1);       
            xline(ha, mean(self.NEES4Estimator), 'b--', 'LineWidth', 1); % the mean should be equal to degree of freedom 
            l = legend(ha, {...
                p.Name, ...
                sprintf('$\\chi^2$ (%d DoF)',    self.DegreeOfFreedom),...
                sprintf('Perfect ANEES = %d',    self.DegreeOfFreedom), ...
                sprintf('Empiric ANEES = %0.1f', mean(self.NEES4Estimator)) ...
                });
            set(l,'Interpreter','latex');
            h = gcf;
        end


    end
end

