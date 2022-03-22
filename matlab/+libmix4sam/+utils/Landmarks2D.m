classdef Landmarks2D < libmix4sam.utils.Landmarks
%LANDMARKS2D Generate and store 2D landmarks.
%   Class extends functionalities of the base class LANDMARKS specific to
%   two dimensions. 
%
%   Examples
%     % Generate and view landmarks within given field of view in circular
%     % style:
%        libmix4sam.utils.Landmarks2D.generateMultiple(1,'type', 'circle', 'size', [20 35], 'amount', 80,'fov',20).plot
%
%   See also LANDMARKS, LANDMARKS3D.

% @author Sven Lange

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
        Dim = 2;
        defaultGenerate = struct(...
            'type','square',...
            'size',[20 20],...  % in meters
            'amount',40,...     % number of landmarks to generate
            'seed',213);
    end
    
    methods
        function self = Landmarks2D(varargin)
        %LANDMARKS2D Construct an instance of this class.
            self@libmix4sam.utils.Landmarks(varargin{:});
        end
        
        function coords = generate(self,varargin)
        %GENERATE Generate a random distributed set of landmarks.
            
            p = inputParser;
            addParameter(p, 'type', self.defaultGenerate.type, @ischar);
            addParameter(p, 'seed', self.defaultGenerate.seed, @isnumeric);
            % For circle type, size means [radius of circle, width of circle]
            addParameter(p, 'size', self.defaultGenerate.size, @(x)validateattributes(x,{'numeric'},{'numel',2}));
            % Applies only for gridsize type.
            addParameter(p, 'gridsize', [1 1], @(x)validateattributes(x,{'numeric'},{'>',0,'integer','numel',2}));
            addParameter(p, 'center', [0 0]', @(x)isnumeric(x)||isempty(x));
            % Applies only for circular type and generates landmarks only
            % in a segment of  a circle.
            addParameter(p, 'fov', 2*pi, @(x)validateattributes(x,{'numeric'},{'>',0,'<=',2*pi}));
            addParameter(p, 'amount', self.defaultGenerate.amount, @isnumeric);
            parse(p,varargin{:});
            
            center = p.Results.center;
            
            genType = p.Results.type;
            
            switch genType
                case 'squaregrid'
                    r = RandStream('mt19937ar','Seed',p.Results.seed);
                    gridsize = p.Results.gridsize;
                    x = linspace(0,p.Results.size(1),gridsize(1)+1);
                    y = linspace(0,p.Results.size(2),gridsize(2)+1);
                    amountPerGrid = floor(p.Results.amount/prod(gridsize));
                    amountRemaining = mod(p.Results.amount,prod(gridsize));
                    self.Coordinates = zeros(p.Results.amount,2);
                    pos = 1;
                    for ix = 2:length(x)
                        for iy = 2:length(y)
                            self.Coordinates(pos:(pos+amountPerGrid-1),:) = ...
                                rand(r,amountPerGrid,2) .* ...
                                [x(ix)-x(ix-1) y(iy)-y(iy-1)] + ...
                                [x(ix-1) y(iy-1)];
                            pos = pos + amountPerGrid;
                        end
                    end
                    self.Coordinates(pos:end,:) = rand(r,amountRemaining,2).*[p.Results.size(1) p.Results.size(2)];
                    self.Coordinates = self.Coordinates - [p.Results.size(1)/2 - center(1) p.Results.size(2)/2 - center(2)];
                case 'square'
                    r = RandStream('mt19937ar','Seed',p.Results.seed);
                    self.Coordinates = rand(r,p.Results.amount,2).*[p.Results.size(1) p.Results.size(2)] - [p.Results.size(1)/2 - center(1) p.Results.size(2)/2 - center(2)];
                case 'circle'
                    % Size means [radius of circle, width of circle]
                    [r1, r2] = RandStream.create('mrg32k3a','Seed',p.Results.seed,'NumStreams',2);
                    rand_angle = rand(r1,1,p.Results.amount);
                    rand_angle = rand_angle * p.Results.fov - p.Results.fov/2;
                    rand_dist = rand(r2,1,p.Results.amount);
                    radius = p.Results.size(1);
                    width = p.Results.size(2);
                    rand_dist = rand_dist * width + radius - width/2;
                    [x,y] = pol2cart(rand_angle,rand_dist);      
                    self.Coordinates = [x(:)-p.Results.center(1), y(:)-p.Results.center(2)];  
                otherwise
                    error('No matching type!');
            end
            
            self.genInfo.type = genType;
            self.genInfo.size = p.Results.size;            
            coords = self.Coordinates;
            self.Id = (1:size(coords,1))';
        end
        
        function transform(self,pose)
        %TRANSFORM Transorm points and covariance.
        %
        %   EXAMPLE:
        %      lmk = libmix4sam.utils.Landmarks([0 1; -3*pi/4 2],'CoordinateSystem','Polar','Covariance',[0.4 0.1;0.4 0.1]);lmk.plot()
        %      lmk.transform(struct('x',0,'y',0,'theta',pi/4)); lmk.plot()
        %
            R=@(x)[cos(x) -sin(x);sin(x) cos(x)];
            self.Coordinates = (R(pose.theta)*self.Coordinates' + [pose.x; pose.y])';
            J = R(pose.theta);
            cov = cellfun(@(P) J*P*J', num2cell(self.Covariance,[1 2]), 'UniformOutput', false);
            self.Covariance = cell2mat(cov);
        end
        
        function [coords, id, polar] = getRelative(self, atPose, fov, range)
            % Return only the landmarks within the fov for a given pose.
            
            if nargin < 3, fov = 0; end
            if nargin < 4, range = []; end
            
            % Transform into local coordinate frame
            % V4 coords_local = arrayfun(@(x,y) atPose.transformTo(gtsam.Point2(x,y)),self.Coordinates(:,1),self.Coordinates(:,2));
            % V4 coords_local = [arrayfun(@(x)x.x,coords_local),arrayfun(@(x)x.y,coords_local)];
            coords_local = libmix4sam.utils.transformTo(atPose, self.Coordinates')'; % V4

            % Calculate polar coordinates if needed.
            if fov > 0 || nargout > 2
                [theta, r] = cart2pol(coords_local(:,1),coords_local(:,2));
            end

            % Cut FoV
            if fov > 0
                m = (theta > -fov/2) & (theta < fov/2);    
            else
                m = true(size(coords_local,1),1);
            end
            
            % Cut Range
            if ~isempty(range) && numel(range)==2 && range(1) >= 0 && range(2) > range(1)
                m = m & ((r > range(1)) & (r < range(2)));
            end
            
            id = find(m);
            coords = coords_local(m,:);
            if nargout > 2, polar = [theta(m) r(m)]; end

        end
        
        function [lm] = getRelativeLm(self, varargin)
            % Return only the landmarks within the fov for a given pose.
            % This time as new Landmarks Class Instance
            
            if length(self) > 1
                for i=length(self):-1:1
                    lm(i) = self(i).getRelativeLm(varargin{:});
                end
                return;
            end
            
            p = inputParser;
            addRequired(p, 'atPose',@(x)isa(x,'gtsam.Pose2'));
            addOptional(p, 'fov', 0, @isnumeric);
            addOptional(p, 'range', [], @isnumeric);
            % Define in which coordinate system the Noise is given
            addParameter(p, 'CoordinateSystem', 'Polar', @ischar);   % Cartesian, Polar
            addParameter(p, 'Noise',[],@(x)isempty(x) || isa(x,'gtsam.noiseModel.Unit') || isa(x,'gtsam.noiseModel.Diagonal')); % Covariance given as variance for each dimension [sigma_theta^2 sigma_rho^2] or [sigma_x^2 sigma_y^2]
            addParameter(p, 'NoiseRandomizationModel',[],@isnumeric); % Add noise to noise [sigma_theta^2 sigma_rho^2]
            parse(p,varargin{:});
            p = p.Results;
            
            [~, id, polar] = getRelative(self, p.atPose, p.fov, p.range);
            
            cov = [];
            if ~isempty(p.Noise)
                switch p.CoordinateSystem
                    case 'Polar'
                        cov = reshape(p.Noise.sigmas.^2,1,[]);
                        cov = repmat(cov,length(id),1);
                    otherwise
                        error('Not implemented yet!');
                end
            end
            
            selfType = mfilename('class');      
            lm = feval(selfType, polar, 'CoordinateSystem', 'Polar', 'LandmarkIds', id, 'Covariance', cov); 
        end
        
        function [ids] = getCorrespondenceIds(this, other)
        %getCorrespondenceIds Return all IDs which are present in both,
        %   previous and current.
        %
            [~,ia] = intersect(this.Id,other.Id);
            ia(this.Id(ia) == -1) = []; % delete outlier indexes
            ids = this.Id(ia);
        end
        
    end
    
    methods(Static)
        
        function [ids] = GetCorrespondenceIds(previous, current)
        %GetCorrespondenceIds Return all IDs which are present in both,
        %   previous and current.
            [~,ia] = intersect(previous.Id,current.Id);
            ia(previous.Id(ia) == -1) = []; % delete outlier indexes
            ids = previous.Id(ia);
        end      
        
        function [self] = generateMultiple(varargin)
            
            p = inputParser; p.KeepUnmatched = true;
            addRequired(p,'numSets',@(x)validateattributes(x,{'numeric'},{'nonnegative','integer'}));
            addParameter(p,'seed',0,@(x)validateattributes(x,{'numeric'},{'nonnegative','integer'}));
            parse(p,varargin{:});
            unmatchedArguments = reshape(horzcat(fieldnames(p.Unmatched),struct2cell(p.Unmatched))',1,[]);
            p = p.Results;
            
            if p.seed == 0, p.seed = sum(100*clock); end

            selfType = mfilename('class');
            s = RandStream('mt19937ar','Seed',p.seed);
            seeds = s.randi(1e5, p.numSets, 1);
            for i = p.numSets:-1:1
                self(i) = feval(selfType);
                self(i).generate('seed', seeds(i), unmatchedArguments{:});
            end
        end
    end
end

