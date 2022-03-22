classdef SE3Type < handle
%SE3TYPE Helper class to handle SE3 data.
%   It is partly a replacement for GTSAM's Pose3 class as pure matlab
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
        Translation = []; % (N x 3)
        Rotation    = []; % (N x 3 x 3) or (N x 1, struct)
        Velocity    = []; % (N-1)x1 struct
        TurnRate    = []; % (N-1)x1 struct, fields: Roll Pitch Yaw
        Time        = []; % Nx1
    end
    
    properties (SetAccess = protected)
        Calibration = []; % If we need to transform the data into another Frame
    end
        
    properties (Dependent = true, Hidden = true)
        trvec
        velvec
        rotm
        drotm
        ypr
    end
    
    properties(SetAccess = protected)
        IsGtsamAvailable = false;
    end
    
    methods
        function self = SE3Type(varargin)
            %SE3TYPE Construct an instance of this class
            
            self.IsGtsamAvailable = self.checkForGtsam() && self.checkForLibisf2();
            
            p = inputParser;
            addOptional(p, 'time', [], @isnumeric);
            addParameter(p, 'Translation', [], @(x)isnumeric(x) || isstruct(x));
            addParameter(p, 'Rotation', [], @(x)isnumeric(x));
            addParameter(p, 'Name', [], @(x)isempty(x) || ischar(x))
            parse(p,varargin{:});
            p = p.Results;
            
            if (~isempty(p.time)), self.Time = p.time; end
            if (~isempty(p.Translation)), self.Translation = p.Translation; end
            if (~isempty(p.Rotation)), self.Rotation = p.Rotation; end
            self.Name = p.Name;
            
        end
        
        function t = get.trvec(self)
            t = [self.Translation.X(:), self.Translation.Y(:), self.Translation.Z(:)];
            if ~isempty(self.Calibration)
                t = cellfun(@(R,t)R * self.Calibration.t(:)+t(:), squeeze(num2cell(permute(self.Rotation, [2 3 1]), [1 2])), squeeze(num2cell(t,2)), 'UniformOutput', false);
                t = cell2mat(t')';
            end 
        end
        
        function t = get.velvec(self)
            t = [self.Velocity.X(:), self.Velocity.Y(:), self.Velocity.Z(:)];
            if ~isempty(self.Calibration)
                t = t * self.Calibration.R; 
            end
        end
        
        function ypr = get.ypr(self)
            ypr = rotm2eul(permute(self.rotm,[2,3,1]));
        end
        
        function R = get.rotm(self)
            R = self.Rotation;
            if ~isempty(self.Calibration)
                R = cellfun(@(x)x * self.Calibration.R, num2cell(permute(R, [2 3 1]), [1 2]), 'UniformOutput', false);
                R = permute(cell2mat(R),[3 1 2]);
            end
        end
        
        function R = get.drotm(self)
            R = eul2rotm([self.TurnRate.Yaw(:), self.TurnRate.Pitch(:), self.TurnRate.Roll(:)].*diff(self.Time));
            R = permute(R,[3 1 2]); % Transform into correct format
%             if ~isempty(self.Calibration)
%                 R = cellfun(@(x)self.Calibration.R * x, num2cell(permute(R, [2 3 1]), [1 2]), 'UniformOutput', false);
%                 R = permute(cell2mat(R),[3 1 2]);
%             end
        end
        
        function setCalibration(self,R,t)
            self.Calibration = struct('R',R,'t',t);
        end
        
        function addTransformation(self,R,t)
            R_new = cellfun(@(x)R * x, num2cell(permute(self.Rotation, [2 3 1]), [1 2]), 'UniformOutput', false);
            self.Rotation = permute(cell2mat(R_new),[3 1 2]);
            self.Translation = self.trvec * R' + t;
        end
        
        function set.Translation(self,t)
            if isempty(t), self.Translation = t; return; end
            if isstruct(t)
                % Already in correct form?
                assert(isfield(t,'X') && isfield(t,'Y') && isfield(t,'Z'),...
                    'Wrong format!');
                self.Translation = t;
                return;
            end
            if size(t,2) == 3
                self.Translation = struct('X',t(:,1),'Y',t(:,2),'Z',t(:,3));
                return;
            end
            error('Wrong format!');
        end
        
        function set.Rotation(self,v)
            if isempty(v), self.Rotation = v; return; end
            if isstruct(v)
                % Already in correct form?
                assert(isfield(v,'X') && isfield(v,'Y') && isfield(v,'Z'),...
                    'Wrong format!');
                R = eul2rotm([v.Z(:), v.Y(:), v.X(:)]);
                self.Rotation = permute(R, [3 1 2]); % Transform into correct format
                return;
            end
            if size(v,2) == 3 && size(v,3) == 3
                self.Rotation = v;
                return;
            end
            error('Wrong format!');
        end
        function newSE2Type = getAsSE2(self)
        %GETASSE2 Convert vom SE3 to SE2, assuming 2D-Problem in x-y-Plane.
            se2type = strrep(mfilename('class'), 'SE3', 'SE2');
            t = self.trvec; ypr_ = self.ypr;
            newSE2Type = feval(se2type, [t(:,1),t(:,2),ypr_(:,1)]);
        end
        
        function h = plotTrajectory(self,varargin)
        %PLOTTRAJECTORY Little helper, to visualize data as trajectory.
        %   Till now, we show the trajectory as SE2 Type.
            asSE2 = self.getAsSE2();
            h = asSE2.plotTrajectory(varargin{:});
        end
        
    end
    
    methods (Static)
        
        function r = checkForGtsam()
        %CHECKFORGTSAM For some methods, gtsam is needed -- check if it is 
        %   available.
        
            if ~exist('gtsam','dir') || isempty(which('gtsam.Pose3'))
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
        function self = fromPose3(poses, varargin)
        %FROMPOSE3 Crate an instance of this class from gtsam's pose3
        %   classes.
            assert(isa(poses,'gtsam.Pose3'),'First argument has to be of type Pose3');
            selfType = mfilename('class');
            
            % Translation Vector
            translation = cell2mat(arrayfun(@(p)[p.x p.y p.z]',poses,'UniformOutput',false))';
            
            % Rotation Matrices
            rotation = arrayfun(@(p)p.rotation.matrix,poses,'UniformOutput',false);
            rotation = permute(cell2mat(reshape(rotation,1,1,[])),[3 1 2]);
            
            % Create class with additional parameters or without
            if nargin > 1
                self = feval(selfType, 'Translation',translation,'Rotation',rotation,varargin{:});
            else
                self = feval(selfType, 'Translation',translation,'Rotation',rotation);
            end
        end
        
    end
    
end

