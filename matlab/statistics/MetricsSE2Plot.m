classdef MetricsSE2Plot < MetricsSE2
%METRICSSE2PLOT Extend MetricsSE2 with plotting methods.
%
%   See also METRICSSE2.

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

    properties
        OutputFolder = [];
    end
    
    methods
        function self = MetricsSE2Plot(varargin)
        %METRICSSE2PLOT Construct an instance of this class.
        %   Class can be constructed either with the same input arguments
        %   as MetricsSE2 or with an instance of this class.
        %   By using the superclass as initialization, additional arguments
        %   for MetricsSE2Plot can be given starting from the second
        %   argument.
        
            if nargin < 1
                super_args = {};
            else
                if isa(varargin{1},'MetricsSE2')
                    % MetricsSE2 class and additional plotting parameters are given.
                    super_args = varargin(1);
                else
                    % Only parameters for MetricsSE2 class are given and additional plotting parameters are given.
                    super_args = varargin;
                end
            end
            
            self@MetricsSE2(super_args{:});
            
            if nargin < 1, return; end
            
            % Evaluate additional arguments for MetricsSE2Plot Class only
            if isa(varargin{1},'MetricsSE2')
                sub_args = varargin(2:end);
            else
                sub_args = self.UnmatchedArguments;
            end
            
            p = inputParser;
            addParameter(p,'OutputFolder',[],@(x)isempty(x) || ischar(x));
            parse(p,sub_args{:})
            p = p.Results;
            self.OutputFolder = p.OutputFolder;
            
        end
        
        function T = showComparisonTable(self)
        %SHOWCOMPARISONTABLE Create table with evaluation results.
        
            names = cellfun(@(x)x.Name,self.Data(2:end),'UniformOutput',false);
            posErr = cell2mat(self.absPositionError');
            posRMSE = sqrt(mean(posErr.^2))';
            angErr = cell2mat(self.geodesicDistance(true)');
            T = table(names',...
                mean(posErr)',...
                posRMSE,...
                [min(posErr)' max(posErr)'],...
                mean(angErr)',...
                sqrt(mean(angErr.^2))',...
                [min(angErr)' max(angErr)']);

            % Column naming
            T.Properties.VariableNames = {...
                'Name','PosError','PosRMSE','PosError min/max',...
                'AngError','AngRMSE','AngError min/max'};
            
        end
        
        function hh = plotAbsRotationErrorHist(self, varargin)
        %PLOTABSROTATIONERRORHIST Plot absolute rotation error as histogram.
            
            p = inputParser;
            addOptional(p,'parent',[],@(x)isempty(x) || isgraphics(x,'axes'));
            addParameter(p,'style',{'BinWidth', 0.1});
            addParameter(p,'showLegend',true,@islogical);
            parse(p,varargin{:})
            p = p.Results;
            
            if isempty(p.parent)
                h = figure('Name','Rotation Error');
                hold on; grid on; p.parent = gca(h);
            end
            
            absErr = self.geodesicDistance(true);
            for i=1:length(absErr)
                hh = histogram(p.parent, absErr{i}, p.style{:});
                hh.DisplayName = [sprintf('(%d) ',i) self.Data{i+1}.Name];
            end
            
            if p.showLegend, legend(p.parent,'show'); end
        end
        
        function plotAbsRotationErrorBox(self,varargin)
        %PLOTABSROTATIONERRORBOX Plot absolute rotation error as boxplot.
        
            p = inputParser;
            addOptional(p,'parent',[],@(x)isempty(x) || isgraphics(x,'axes'));
            addParameter(p,'showLegend',true,@islogical);
            parse(p,varargin{:})
            p = p.Results;
            if isempty(p.parent)
                h = figure('Name','Rotation Error');
                hold on; grid on; p.parent = gca(h);
            end
            
            absErr = self.geodesicDistance(true);
            absErr = absErr(:)';
            grp_box_x = cell(1,size(absErr,2));
            err_box_legend = cell(1,size(absErr,2));
            for i=1:size(absErr,2)
                grp_box_x{1,i} = repmat(i,length(absErr{1,i}),1);
                err_box_legend{1,i} = [sprintf('(%d) ',i) self.Data{i+1}.Name];
            end
            absErr = cell2mat(absErr');
            grp_box_x = cell2mat(grp_box_x');
            grp_box_x = categorical(grp_box_x, 0:max(grp_box_x),arrayfun(@(x)sprintf('(%d)',x),(0:max(grp_box_x)),'UniformOutput',false));
            boxplot(p.parent, absErr, grp_box_x);
            
            if p.showLegend
                box_vars = findall(p.parent,'Tag','Box');
                legend(box_vars, err_box_legend);
            end
        end
        
        function hh = plotAbsTranslationErrorHist(self, varargin)
        %PLOTABSTRANSLATIONERRORHIST Plot absolute translation error as
        %   histogram.
            
            p = inputParser;
            addOptional(p,'parent',[],@(x)isempty(x) || isgraphics(x,'axes'));
            addParameter(p,'style',{'BinWidth', 0.04});
            addParameter(p,'showLegend',true,@islogical);
            parse(p,varargin{:})
            p = p.Results;
            
            if isempty(p.parent)
                h = figure('Name','Translation Error');
                hold on; grid on; p.parent = gca(h);
            end
            
            absErr = self.absPositionError();
            for i=1:length(absErr)
                hh = histogram(p.parent, absErr{i}, p.style{:});
                hh.DisplayName = [sprintf('(%d) ',i) self.Data{i+1}.Name];
            end
            
            if p.showLegend, legend(p.parent,'show'); end
        end

        function plotAbsTranslationErrorBox(self,varargin)
        %PLOTABSTRANSLATIONERRORBOX Plot absolute translation error as
        %   boxplot. 
        
            p = inputParser;
            addOptional(p,'parent',[],@(x)isempty(x) || isgraphics(x,'axes'));
            addParameter(p,'showLegend',true,@islogical);
            parse(p,varargin{:})
            p = p.Results;
            if isempty(p.parent)
                h = figure('Name','Translation Error');
                hold on; grid on; p.parent = gca(h);
            end
            
            absErr = self.absPositionError();
            absErr = absErr(:)';
            grp_box_x = cell(1,size(absErr,2));
            err_box_legend = cell(1,size(absErr,2));
            for i=1:size(absErr,2)
                grp_box_x{1,i} = repmat(i,length(absErr{1,i}),1);
                err_box_legend{1,i} = [sprintf('(%d) ',i) self.Data{i+1}.Name];
            end
            absErr = cell2mat(absErr');
            grp_box_x = cell2mat(grp_box_x');
            grp_box_x = categorical(grp_box_x, 0:max(grp_box_x),arrayfun(@(x)sprintf('(%d)',x),(0:max(grp_box_x)),'UniformOutput',false));
            boxplot(p.parent, absErr, grp_box_x);
            
            if p.showLegend
                box_vars = findall(p.parent,'Tag','Box');
                legend(box_vars, err_box_legend);
            end
        end        
        
        function [h, ax] = plotError(self, prepareForPrint)
        %PLOTERROR Generate a plot with histograms and boxplots for
        %   translation and rotation error in one figure.
        
            if nargin < 2, prepareForPrint = false; end
            h = figure('Name','MetricsSE2 Error Plot');
            ax = struct();
            ax.TransHist = subplot(2,3,[1 2]); hold on; title('Absolute Translation Error in m');
            ax.RotHist   = subplot(2,3,[4 5]); hold on; title('Rotation Error in Deg');
            ax.TransBox  = subplot(2,3,3); hold on; title('as Boxplot');
            ax.RotBox    = subplot(2,3,6); hold on; title('as Boxplot');
            
            self.plotAbsTranslationErrorHist(ax.TransHist);
            self.plotAbsRotationErrorHist(ax.RotHist, 'showLegend', false);
            
            self.plotAbsTranslationErrorBox(ax.TransBox,'showLegend',false);
            self.plotAbsRotationErrorBox(ax.RotBox, 'showLegend', false);
            
            if prepareForPrint
                h.Units = 'centimeters';
                h.Position(3:4) = [25 15];
                h.Renderer = 'painters';
                h.Color = [1 1 1];
%                 style = hgexport('factorystyle');
%                 style.Format = 'svg';
%                 style.Units = 'centimeters';
%                 style.Width = '16';
%                 style.Height = '8';
%                 style.Bounds = 'tight';
%                 style.Renderer = 'painters';
%                 hgexport(h,'tmp.svg',style,'applystyle', true);
            end
        end 
        
        function h = plotTrajectory(self,varargin)
        %PLOTTRAJECTORY Plot a trajectory.
        %   Here, it is assumed, that the data is based on a trajectory and
        %   it is already aligned.
        %   Method is called recursively for the contained datasets.
            
            p = inputParser;
            addOptional(p,'parent',[],@(x)isempty(x) || isgraphics(x,'axes'));
            addParameter(p,'showLegend',true,@islogical);
            addParameter(p,'showOrientation',true,@islogical);
            parse(p,varargin{:})
            p = p.Results;
            
            if isempty(p.parent)
                h = figure('Name','MetricsSE2 Trajectory Plot');
                hold on; grid on; axis equal;
                p.parent = gca(h);
            else
                h = p.parent.Parent;
            end
            
            % E.g. ground truth data
            self.Reference.plotTrajectory(p.parent, {'k.-'},'showOrientation',p.showOrientation)
            
            c = jet(length(self.Data)-1);
            for i=2:length(self.Data)
                self.Data{i}.plotTrajectory(p.parent, ...
                    {'b.-','Color',c(i-1,:)}, ...
                    'name', sprintf('(%d) %s',i-1,self.Data{i}.Name), ...
                    'showOrientation',p.showOrientation);
            end
            
            xlabel('x in m'); ylabel('y in m');
            if p.showLegend, legend; end
        end
        
        
    end
end

