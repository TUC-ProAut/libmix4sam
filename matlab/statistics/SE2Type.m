classdef SE2Type < handle
%SE2TYPE Helper class to handle SE2 data.
%   It is partly a replacement for GTSAM's Pose2 class as pure matlab
%   implementation.
% 

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
        Name        = []; % Optional, Name of Dataset
        X           = []; % (N x 1)
        Y           = []; % (N x 1)
        Theta       = []; % (N x 1)
    end
            
    properties (Dependent = true, Hidden = true)
        Translation;  % (N x 2)
        Rotation;     % (N x 1)
        rotm;         % (N x 2 x 2)
    end
    
    properties(SetAccess = protected)
        IsGtsamAvailable = false;
    end
    
    methods
        function self = SE2Type(varargin)
        %SE2TYPE Construct an instance of this class
            
            self.IsGtsamAvailable = self.checkForGtsam() && self.checkForLibisf2();
            
            p = inputParser;
            addOptional(p,  'X',     [], @isnumeric);
            addOptional(p,  'Y',     [], @isnumeric);
            addOptional(p,  'Theta', [], @isnumeric);
            addParameter(p, 'Name',  [], @(x)isempty(x) || ischar(x));
            parse(p,varargin{:});
            p = p.Results;
            
            if (~isempty(p.X))
                if size(p.X,2) == 3
                    self.X = p.X(:,1);
                    self.Y = p.X(:,2);
                    self.Theta = p.X(:,3);
                else
                    self.X = p.X;
                end
            end
            if (~isempty(p.Y)), self.Y = p.Y; end
            if (~isempty(p.Theta)), self.Theta = p.Theta; end
            self.Name = p.Name;
            
        end
        
        function set.X(self,t)
            if isempty(t), self.X = t; return; end
            assert(min(size(t)) == 1, 'Only one dimension needed!');
            self.X = t(:);
        end
        
        function set.Y(self,t)
            if isempty(t), self.Y = t; return; end
            assert(min(size(t)) == 1, 'Only one dimension needed!');
            self.Y = t(:);
        end
        
        function set.Theta(self,t)
            if isempty(t), self.Theta = t; return; end
            assert(min(size(t)) == 1, 'Only one dimension needed!');
            self.Theta = t(:);
        end

        function v = get.Translation(self), v = [self.X self.Y]; end
        function v = get.Rotation(self), v = self.Theta; end
        
        function R = get.rotm(self)
            R = arrayfun(@(x)[cos(x) -sin(x);sin(x) cos(x)],self.Theta,'UniformOutput',false);
            R = reshape(R,1,1,[]);
            R = permute(cell2mat(R),[3 1 2]);
        end
        
        function new = minus(self, other)
            selfType = mfilename('class');      
            new = feval(selfType, self.X - other.X, self.Y - other.Y, angdiff(other.Theta,self.Theta)); 
        end
        
        function new = diff(self)
            R = self.rotm;
            t = self.Translation;
            theta = zeros(length(self.X)-1,1);
            t_new = zeros(length(self.X)-1,2);
            for i=2:length(self.X)
                t_new(i-1,:) = squeeze(R(i-1,:,:))'*( t(i,:)' - t(i-1,:)' );
                R_new = squeeze(R(i-1,:,:))'*squeeze(R(i,:,:));
                theta(i-1) = atan2(R_new(2,1),R_new(1,1));
            end
            selfType = mfilename('class');
            new = feval(selfType, t_new(:,1),t_new(:,2),theta,'Name',self.Name);
        end
        
        function new = cumsum(self)
        %CUMSUM Accumulate relative measurements.
        %   E.g. the contained data are relative motion (odometry)
        %   measurements only, this method can be used to accumulate them
        %   to represent trajectory data.
        
            R = self.rotm;
            t = self.Translation;
            theta = self.Theta;
            %R = vertcat(startPose.R, R);
            %t = self.velvec .* diff(self.Time);
            %t = vertcat(startPose.t(:)',t);
            for i=2:size(R,1) 
                R(i,:,:) = squeeze(R(i-1,:,:))*squeeze(R(i,:,:)); 
                theta(i) = atan2(R(i,2,1),R(i,1,1));
            end
            for i=2:size(t,1), t(i,:) = t(i-1,:)' + squeeze(R(i-1,:,:)) * t(i,:)'; end
            selfType = mfilename('class');  
            new = feval(selfType, t(:,1),t(:,2),theta,'Name',self.Name);
        end
        
        function ret = toNumeric(self)
            ret = [self.X, self.Y, self.Theta];
        end
        
        function asPoses = toGtsamPoses(self)
        %TOGTSAMPOSES Convert data to GTSAM's Pose2 class.
            if self.IsGtsamAvailable
                asPoses = arrayfun(@(x,y,t)gtsam.Pose2(x,y,t),self.X,self.Y,self.Theta);
            else
                asPoses = [];
            end
        end
        
        function rpg = toXYZqxqyqzqw(self)
        %TOXYZQXQYQZQW Convert to format for rpg_trajectory_evaluation.
        %   For the evaluation toolbox, see:
        %   https://github.com/uzh-rpg/rpg_trajectory_evaluation
        
            e = self.Theta(:);
            e(end,3) = 0;
            rot = eul2quat(e); % w x y z
            rpg = [self.X(:), self.Y(:), e(:,2), rot(:,4), rot(:,1:3)];
        end
        
        function asValues = toGtsamValues(self)
        %TOGTSAMVALUES Convert data to GTSAM's Values containing Pose2.
            if self.IsGtsamAvailable
                asPoses = self.toGtsamPoses();
                asValues = gtsam.Values;
                for i=1:length(asPoses)
                    asValues.insert(i-1,asPoses(i));
                end
            else
                asValues = [];
            end
        end
        
        function h = plotTrajectory(self,varargin)
        %PLOTTRAJECTORY Little helper, to visualize data as trajectory.
            
            p = inputParser;
            addRequired(p,'parent',@(x)isgraphics(x,'axes'));
            addOptional(p,'lineStyle',{'k.'});
            addParameter(p,'name',self.Name,@(x)isempty(x) || ischar(x)); % Name for legend
            addParameter(p,'showOrientation',true,@islogical);
            parse(p,varargin{:})
            p = p.Results;
            if isempty(p.name), p.name = 'SE2trajectory'; end
            
            h = hggroup;
   
            plot(self.X,self.Y,p.lineStyle{:},'Parent',h);
            if p.showOrientation
                quiver(self.X, self.Y, cos(self.Theta), sin(self.Theta), 0.1, p.lineStyle{:},'Parent',h);
            end
            
            % Modify hggroup for legend information.
            set(get(get(h,'Annotation'),'LegendInformation'), 'IconDisplayStyle','on'); 
            set(h,{'DisplayName'},{p.name});
            
        end
    end
    
    methods (Static)
        
        function r = checkForGtsam()
        %CHECKFORGTSAM For some methods, gtsam is needed -- check if it is 
        %   available.
        
            if ~exist('gtsam','dir') || isempty(which('gtsam.Pose2'))
                warning('Ignoring call of method, because gtsam library is not within path!');
                r = false;
            else
                r = true;
            end
        end
        
        function r = checkForLibisf2()
            if ~exist('libmix4sam','dir') || isempty(which('libmix4sam.utils.plot2DTrajectory'))
                warning('Ignoring call of method, because gtsam library is not within path!');
                r = false;
            else
                r = true;
            end
        end
        
        function self = FromPose2Array(v)
            selfType = mfilename('class');
            self = feval(selfType, libmix4sam.utils.gtClass2num(v)); 
        end
        
        function self = FromPose2Values(v)
            selfType = mfilename('class');
            self = feval(selfType, gtsam.utilities.extractPose2(v)); 
        end
        
    end
end

