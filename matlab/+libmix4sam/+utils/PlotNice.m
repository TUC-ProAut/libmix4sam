classdef PlotNice < handle
%PLOTNICE Little helper class for customizing plot properties.
%   Can be used to make plots look similar. 

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
        FigHandle  % Handle of figure to be modified.
    end
    
    methods
        function self = PlotNice(h)
        %PLOTNICE Construct an instance of this class.
            if nargin < 1, h = gcf; end
            self.FigHandle = h;
        end
        
        function setCommonBinWidth(self,width,reduceData)
        %SETCOMMONBINWIDTH Set binwidth for all subplots accordingly.
            if nargin < 3, reduceData = false; end
            for i = findobj(self.FigHandle, 'Type','Histogram'), set(i,'BinWidth',width), end
            if reduceData
                % In case, the bin width can not be set because of maximum
                % number of bins, change the data.
                maxData = 65000*width;
                hists = findobj(self.FigHandle, 'Type','Histogram');
                for iHist = 1:numel(hists)
                    hists(iHist).Data(hists(iHist).Data > maxData) = maxData;
                end
            end
        end
        
        function setCommonLimits(self,xlimit,ylimit)
        %SETCOMMONLIMITS Set the limits for all subplots accordingly.
            if ~isempty(xlimit)
                for i = findobj(self.FigHandle, 'Type','Axes'), set(i,'XLim',xlimit), end
            end
            if ~isempty(ylimit)
                for i = findobj(self.FigHandle, 'Type','Axes'), set(i,'YLim',ylimit), end
            end
        end
        
        function subVertCommonX(self)
        %SUBVERTCOMMONX Use only one x-axis for all subplots.
            ax = findobj(self.FigHandle.Children,'Type','Axes');
            linkaxes(ax,'x');
            titlespace = 0.05;
            axisspace = 0.05;
            space = (1-(titlespace + axisspace))/numel(ax);
            positions = reshape([ax.Position]',4,[])';
            positions = [(1:size(positions,1))', positions];
            % Assuming that positions is already ordered begining from
            % bottom subplot.
            positions(:,3) = ((1:size(positions,1))-1)*space + axisspace;
            positions(:,5) = space;
            for iSub = 1:size(positions,1)
                set(ax(iSub),'Position',positions(iSub,2:end))
                if iSub ~= 1, set(ax(iSub),'XTickLabel',[]); end
            end
        end
    end
end

