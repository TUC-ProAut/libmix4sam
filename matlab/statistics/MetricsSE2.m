classdef MetricsSE2 < matlab.mixin.Copyable
%METRICSSE2 Useful metrics for the SE2 space.
%   Collection of metrics for evaluation.
%
%   See also METRICSSE2PLOT.

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

    properties (SetAccess = protected)
        Data = {};
    end
    
    properties (Dependent = true)
        Reference;
    end
    
    properties (Access = protected)
        UnmatchedArguments = {};   % For Superclass creation
    end    
    
    methods
        function self = MetricsSE2(varargin)
        %METRICSSE2 Construct an instance of this class
            
            if nargin > 0 && isa(varargin{1}, 'MetricsSE2')
                % Construct by copy of given Instance of this class.
                % (using the copy method of matlab.mixin.Copyable will not
                % work within constructor!)
                C = metaclass(varargin{1});
                P = C.Properties;
                for k = 1:length(P)
                    if ~P{k}.Dependent
                        try % Unfortunately everytime, the getMethod for the property will be called, which won't work everytime.
                            self.(P{k}.Name) = varargin{1}.(P{k}.Name);
                        end
                    end
                end
            else
                p = inputParser;
                addOptional(p,'reference',[],@(x)isempty(x) || isa(x,'SE2Type'));
                p.KeepUnmatched = true;
                parse(p,varargin{:});
                self.UnmatchedArguments = reshape(horzcat(fieldnames(p.Unmatched),struct2cell(p.Unmatched))',1,[]);
                
                p = p.Results;
                
                if ~isempty(p.reference), self.Data = {p.reference}; end
                
            end
            
        end
        
        function r = get.Reference(self)
            r = self.Data{1};
        end
        
        function setReference(self,ref)
            assert(isa(ref,'SE2Type'),'Data has to be of Type SE2!');
            self.Data{1} = ref;
        end
        
        function addData(self,d)
            assert(isa(d,'SE2Type'),'Data has to be of Type SE2!');
            self.Data{end+1} = d;
        end
        
        function [ret] = geodesicDistance(self,inDeg)
        %GEODESOCDOSTAMCE In 2D space, this is only the angle difference.
        %   Geodesic distance is used as distance metric for rotation 
        %   changes in 3D. Here it is only used as synonym.
            if nargin < 2, inDeg = false; end
            if inDeg, s = 180/pi; else, s = 1; end
            ret = cell(length(self.Data)-1,1);
            for i=1:(length(self.Data)-1)
                ret{i} = abs(angdiff(self.Reference.Theta, self.Data{i+1}.Theta) * s);
            end
        end
        
        function [ret] = absPositionError(self)
        %ABSPOSITIONERROR The absolute position error. 
            ret = cell(length(self.Data)-1,1);
            for i=1:(length(self.Data)-1)
                ret{i} = ...
                    sqrt((self.Reference.X - self.Data{i+1}.X).^2 + ...
                    (self.Reference.Y - self.Data{i+1}.Y).^2);
            end
        end
        
    end
end

