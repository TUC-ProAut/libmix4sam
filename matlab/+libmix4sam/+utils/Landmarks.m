classdef (Abstract) Landmarks < matlab.mixin.Copyable
%LANDMARKS Base class for generating and storing landmarks.
%   This is a helper class for managing landmarks. Especially, it helps
%   generating random sets of landmarks or manages transformations,
%   coordinate system conversion, and other things.
%
%   See also LANDMARKS2D, LANDMARKS3D.

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
    
    properties(Abstract)
        defaultGenerate;
        Dim;   % Dimension: for 2D = 2 and for 3D = 3
    end
    
    properties
        Coordinates;            % For storing the current landmark configuration
        Covariance = [];        % Optional, for storing the landmarks covariance matrix
        Id = [];                % Optional, for storing tha landmarks unique id
    end

    properties (Access = protected)
        genInfo;  % used Settings for generating the landmark configuration
    end
    
    methods (Abstract)
        generate(self,varargin);
        transform(self,pose);
        getRelative(self, atPose, fov);
    end
    
    methods
        function self = Landmarks(varargin)
        %LANDMARKS Construct an instance of this class
        %
        %  (For copy and paste type:
        %    'help libmix4sam.utils.Landmarks.Landmarks')
        %
        %  EXAMPLE:
        %    rng(100); 
        %    A = (rand(6,2) - [0.5 0]) .* [2*30*pi/180 50];
        %    var = ((rand(6,2) + [-0.5 0.1]) .* [2*5*pi/180 1]).^2;
        %    lmk = libmix4sam.utils.Landmarks(A,'CoordinateSystem','Polar','Covariance',var)
        %    figure(); plot(lmk.Coordinates(:,1),lmk.Coordinates(:,2),'ro'); grid on; axis equal; hold on
        %    gtsam.covarianceEllipse(lmk.Coordinates(1,:),lmk.Covariance(:,:,1),'r')
        %
            
            self.genInfo = struct();
            
            p = inputParser;
            % Landmark coordinates
            addOptional(p,'Coordinates',[], @isnumeric);
            % Given Coordinates are Cartesian or Polar?
            validFcn = @(x) assert(ismember(x,{'Cartesian','Polar','Sphere'}),...
                'No valid coordinate frame given!');
            addParameter(p, 'CoordinateSystem', 'Cartesian', validFcn);
            % Covariance for each given point in the same coordinate system
            % as the points are!
            addParameter(p, 'Covariance', [], @isnumeric);
            % Optional ID for each landmark to have ground truth
            % correspondences later on.
            addParameter(p, 'LandmarkIds', [], @isnumeric);
            parse(p,varargin{:});
            p = p.Results;
            
            switch p.CoordinateSystem
                case 'Cartesian'
                    % In case of coordinates, which are already in
                    % cartesian space, we just copy them!
                    self.Coordinates = p.Coordinates;
                    self.Covariance = p.Covariance;
                    
                case 'Polar'
                    % In case, the coordinates and covariance is in polar
                    % space, we have to convert into cartesian space.
                    [x, y] = pol2cart(p.Coordinates(:,1),p.Coordinates(:,2));
                    self.Coordinates = [x(:) y(:)];
                    if ~isempty(p.Covariance)
                        % Covariance given as variance for each dimension
                        % [sigma_theta^2 sigma_rho^2]xN
                        N = size(p.Covariance,1); % Number of Landmarks
                        dim = size(p.Covariance,2);
                        cov = reshape(p.Covariance',1,dim,[]).*repmat(eye(2),[1,1,N]); % Save as Covariance Matrices
                        % syms t r real; [x,y] = pol2cart(t,r); jacobian([x;y],[t r])
                        J_xy = arrayfun(@(t,r)[-r*sin(t), cos(t); r*cos(t), sin(t)],p.Coordinates(:,1),p.Coordinates(:,2),'UniformOutput',false);
                        cov_xy = cellfun(@(J,P) J*P*J', reshape(J_xy,1,1,[]), num2cell(cov,[1 2]), 'UniformOutput', false);
                        self.Covariance = cell2mat(cov_xy); % Convert into Cartesian Space
                    end
                case 'Sphere'
                    % In case, the coordinates and covariance is in sphere
                    % space, we have to convert into cartesian space.
                    [x,y,z] = sph2cart(p.Coordinates(:,1),p.Coordinates(:,2),p.Coordinates(:,3));
                    self.Coordinates = [x(:) y(:) z(:)];
                    if ~isempty(p.Covariance)
                        % Covariance given as variance for each dimension
                        % [sigma_phi^2 sigma_theta^2 sigma_r^2]xN
                        N = size(p.Covariance,1); % Number of Landmarks
                        cov = reshape(p.Covariance',1,self.Dim,[]).*repmat(eye(3),[1,1,N]); % Save as Covariance Matrices
                        % syms p t r real; [x,y,z] = sph2cart(p,t,r); jacobian([x;y;z],[p t r])
                        J = @(p,t,r)[ -r*cos(t)*sin(p), -r*cos(p)*sin(t), cos(p)*cos(t);...
                                       r*cos(p)*cos(t), -r*sin(p)*sin(t), cos(t)*sin(p);...
                                                     0,         r*cos(t),        sin(t)];
                        J_xyz = arrayfun(J,p.Coordinates(:,1),p.Coordinates(:,2),p.Coordinates(:,3),'UniformOutput',false);
                        cov_xyz = cellfun(@(J,P) J*P*J', reshape(J_xyz,1,1,[]), num2cell(cov,[1 2]), 'UniformOutput', false);
                        self.Covariance = cell2mat(cov_xyz); % Convert into Cartesian Space
                    end
            end
            
            if ~isempty(p.LandmarkIds)
                assert(length(p.LandmarkIds) == size(self.Coordinates,1),'Length of given ids does not correspond with the number of landmarks!');
                self.Id = p.LandmarkIds(:);
            else
                self.Id = (1:size(self.Coordinates,1))';
            end

        end
               
        function [] = addNoiseModel(self, value, varargin)
            p = inputParser;
            % Type depends on subsequent function parameters.
            addRequired(p, 'Value');
            % Given Coordinates are Cartesian or Polar?
            validFcn = @(x) assert(ismember(x,{'Cartesian','Polar','Sphere'}),...
                'No valid coordinate frame given!');
            addParameter(p, 'CoordinateSystem', 'Cartesian', validFcn);
            validFcn = @(x) assert(ismember(x,{'Constant','PerLandmark','NoisyNoise'}),...
                'No valid Noise Type given!');
            addParameter(p, 'NoiseType', 'Constant', validFcn);
            parse(p,value,varargin{:});
            
            if strcmp(p.Results.CoordinateSystem,'Polar') && strcmp(p.Results.NoiseType,'Constant')
                % Covariance given as variance for each dimension, constant
                % for all landmarks.
                % [sigma_theta^2 sigma_rho^2]x1
                numPoints = size(self.Coordinates,1);
                perLandmark = repmat(reshape(p.Results.Value,1,2),numPoints,1);
                self.addNoiseModel(perLandmark, 'CoordinateSystem', 'Polar', 'NoiseType', 'PerLandmark');
                return;
            end
            
            if strcmp(p.Results.CoordinateSystem,'Sphere') && strcmp(p.Results.NoiseType,'Constant')
                % Covariance given as variance for each dimension, constant
                % for all landmarks.
                % [sigma_phi^2 sigma_theta^2 sigma_r^2]xN
                numPoints = size(self.Coordinates,1);
                perLandmark = repmat(reshape(p.Results.Value,1,3),numPoints,1);
                self.addNoiseModel(perLandmark, 'CoordinateSystem', 'Sphere', 'NoiseType', 'PerLandmark');
                return;
            end
            
            if strcmp(p.Results.CoordinateSystem,'Polar') && strcmp(p.Results.NoiseType,'PerLandmark')
                % Covariance given as variance for each dimension
                % [sigma_theta^2 sigma_rho^2]xN
                numPoints = size(self.Coordinates,1);
                numCov = size(p.Results.Value,1);
                assert(numPoints == numCov, 'Number of points does not correspond to number of covariances!');
                N = numPoints;
                assert(size(p.Results.Value,2) == 2, 'Wrong dimension of standard deviations. (Should be [sigma_theta^2 sigma_rho^2]xN');
                polarCoords = self.getPolarCoordinates();
                cov = reshape(p.Results.Value',1,2,[]).*repmat(eye(2),[1,1,N]); % Save as Covariance Matrices
                % syms t r real; [x,y] = pol2cart(t,r); jacobian([x;y],[t r])
                J_xy = arrayfun(@(t,r)[-r*sin(t), cos(t); r*cos(t), sin(t)],polarCoords(:,1), polarCoords(:,2), 'UniformOutput', false);
                cov_xy = cellfun(@(J,P) J*P*J', reshape(J_xy,1,1,[]), num2cell(cov,[1 2]), 'UniformOutput', false);
                self.Covariance = cell2mat(cov_xy); % Convert into Cartesian Space
                return;
            end
            
            if strcmp(p.Results.CoordinateSystem,'Sphere') && strcmp(p.Results.NoiseType,'PerLandmark')
                % Covariance given as variance for each dimension
                % [sigma_phi^2 sigma_theta^2 sigma_r^2]xN
                numPoints = size(self.Coordinates,1);
                numCov = size(p.Results.Value,1);
                assert(numPoints == numCov, 'Number of points does not correspond to number of covariances!');
                N = numPoints;
                assert(size(p.Results.Value,2) == 3, 'Wrong dimension of standard deviations. (Should be [sigma_phi^2 sigma_theta^2 sigma_r^2]xN');
                
                sphereCoords = self.getSphereCoordinates();
                cov = reshape(p.Results.Value',1,3,[]).*repmat(eye(3),[1,1,N]); % Save as Covariance Matrices
                % syms p t r real; [x,y,z] = sph2cart(p,t,r); jacobian([x;y;z],[p t r])
                J = @(p,t,r)[ -r*cos(t)*sin(p), -r*cos(p)*sin(t), cos(p)*cos(t);...
                               r*cos(p)*cos(t), -r*sin(p)*sin(t), cos(t)*sin(p);...
                                             0,         r*cos(t),        sin(t)];
                J_xyz = arrayfun(J,sphereCoords(:,1),sphereCoords(:,2),sphereCoords(:,3),'UniformOutput',false);
                cov_xyz = cellfun(@(J,P) J*P*J', reshape(J_xyz,1,1,[]), num2cell(cov,[1 2]), 'UniformOutput', false);
                self.Covariance = cell2mat(cov_xyz); % Convert into Cartesian Spacereturn;
                return;
            end
            
            if strcmp(p.Results.CoordinateSystem,'Polar') && strcmp(p.Results.NoiseType,'NoisyNoise')
                pp = inputParser;
                addParameter(pp,'d',     [0.1 0.3], @(x)validateattributes(x,{'double'},{'numel', 2}));
                addParameter(pp,'theta', [1 3]*pi/180, @(x)validateattributes(x,{'double'},{'numel', 2}));
                addParameter(pp,'seed',  0, @isnumeric);
                parse(pp,p.Results.Value{:});
                params = pp.Results;
                numPoints = size(self.Coordinates,1);
                % [sigma_theta^2 sigma_rho^2]xN
                [r1, r2] = RandStream.create('mrg32k3a', 'Seed', params.seed, 'NumStreams', 2);
                sigma_d = rand(r1,numPoints,1) * diff(params.d) + params.d(1);
                sigma_theta = rand(r2,numPoints,1) * diff(params.theta) + params.theta(1);
                self.addNoiseModel([sigma_theta sigma_d].^2, 'CoordinateSystem', 'Polar', 'NoiseType', 'PerLandmark');
                return;
            end
            
            
            error('Not implemented yet!');
        end

        
        function [] = addClusters(self, varargin)
        %addClusters Transform given amount of landmarks into clusters of
        %   landmarks.
            p = inputParser;
            addOptional(p,'LmsToChange', 0.5, @isnumeric); % in percent
            addOptional(p,'ClusterSize', 3,@isnumeric);
            addParameter(p,'Noise',[],@(x)isempty(x) || isa(x,'gtsam.noiseModel.Unit') || isa(x,'gtsam.noiseModel.Diagonal')); % [sigma_x^2 sigma_y^2]
            addParameter(p,'Seed',2833,@(x)validateattributes(x,{'numeric'},{'nonnegative','integer'}));
            addParameter(p,'Ids', [], @isnumeric);
            parse(p,varargin{:});
            p = p.Results;
            
            if length(self) > 1
                s = RandStream('mt19937ar','Seed',p.Seed);
                p.Seed = s.randi(1e5, length(self), 1);
            end
            
            % Apply clusters for all landmark sets
            for iSet=1:length(self)
                % LM-Mask
                r = RandStream('mt19937ar', 'Seed', p.Seed(iSet) + 231380);
                m = r.randperm(size(self(iSet).Coordinates,1),round(size(self(iSet).Coordinates,1) * p.LmsToChange));
                % Add Points
                r = RandStream('mt19937ar', 'Seed', p.Seed(iSet));
                lmsToAdd = repmat(self(iSet).Coordinates(m,:),p.ClusterSize-1,1);
                lmsToAdd = lmsToAdd + randn(r, size(lmsToAdd,1), self(iSet).Dim) * diag(p.Noise.sigmas);
                self(iSet).Coordinates = [self(iSet).Coordinates;lmsToAdd];
                if ~isempty(self(iSet).Id)
                    idsToAdd = (1:size(lmsToAdd,1))'+ max(self(iSet).Id);
                    if length(p.Ids) == 1 && p.Ids == -1, idsToAdd = repmat(-1,size(lmsToAdd,1),1); end  % Mark as Outliers                                        
                    self(iSet).Id = [self(iSet).Id; idsToAdd];
                end
            end
        end
        
        function add(self,coordinates,varargin)
            p = inputParser;
            addRequired(p,'Coordinates',@isnumeric);
            addOptional(p,'Id', [], @isnumeric); % Default: next available id
            addParameter(p,'Covariance',[],@isnumeric);
            parse(p,coordinates,varargin{:});
            p = p.Results;
            self.Coordinates(end+1,:) = p.Coordinates;
            if isempty(p.Id), id = max(self.Id)+1; else, id = p.Id; end
            self.Id(end+1) = id;
            if ~isempty(p.Covariance)
                self.Covariance(:,:,end+1) = p.Covariance;
            end
        end
        
        function s = getSize(self)
            s = size(self.Coordinates,1);
        end
        
        function lm = getNoisyCartesian(self, s)
            % Create noise within the landmarks based on the landmarks'
            % covariance definition.
            if nargin < 2, s = []; end
            if isempty(s), s = 'shuffle'; end
            r = RandStream('mt19937ar','Seed',s);
            
            numPoints = size(self.Coordinates,1);
            P = squeeze(num2cell(self.Covariance,[1 2]));
            rPoints = num2cell(randn(r, numPoints, self.Dim), 2);
            rPoints = cellfun(@(rPoint, Sigma) rPoint * chol(Sigma), rPoints, P, 'UniformOutput', false);
            coordinates = self.Coordinates + cell2mat(rPoints);

            selfType = class(self);      
            lm = feval(selfType, coordinates, 'CoordinateSystem', 'Cartesian', 'Covariance', self.Covariance,'LandmarkIds',self.Id); 
        end
        
        function [phi, r, cov] = getPolarCoordinates(self)
            [phi, r] = cart2pol(self.Coordinates(:,1),self.Coordinates(:,2));
            if nargout > 2
                cov = [];
                error('Not implemented yet');
            end
            if nargout < 2, phi = [phi(:) r(:)]; end
        end
        
        function [phi, theta, r, cov] = getSphereCoordinates(self)
            [phi, theta, r] = cart2sph(self.Coordinates(:,1),self.Coordinates(:,2),self.Coordinates(:,3));
            if nargout > 3
                cov = [];
                error('Not implemented yet');
            end
            if nargout < 3, phi = [phi(:) theta(:) r(:)]; end
        end
        
        function map = getAsOccupancyMap(self,varargin)
        %getAsOccupancyMap Used for GPMP2 as input for the signed distance
        %   field.
        %
            p = inputParser;
            addParameter(p, 'cellSize', 0.05, @isnumeric);
            parse(p,varargin{:});
            
            map = struct();
            map.cell_size = p.Results.cellSize;
            map.cols = ceil(self.genInfo.size(1) / map.cell_size);
            map.rows = ceil(self.genInfo.size(2) / map.cell_size);
            map.origin_x = -self.genInfo.size(1) / 2;
            map.origin_y = -self.genInfo.size(2) / 2;
            map.map = zeros(map.rows, map.cols);
            
            centers = floor([self.Coordinates(:,2) - map.origin_y, self.Coordinates(:,1) - map.origin_x]./map.cell_size);
            ind = sub2ind([map.rows map.cols],centers(:,1),centers(:,2));
            map.map(ind) = 1;
        end
        
        function [h, hp, hg] = plot(self,varargin)
        % PLOT Minimalistic Plot method.
            
            p = inputParser;
            addOptional(p, 'hax', []);
            addOptional(p, 'Color', 'r', @(x)validateattributes(x,{'char','double'},{'nonempty'}));
            % Sigma-Bound of error ellipse. (default is 1-Sigma)
            addParameter(p, 'StdBound', 1, @isnumeric);
            parse(p,varargin{:}); 
            if ismember('hax',p.UsingDefaults)
                h = figure('Name','Landmarks');
                grid on; axis equal; hold on;
                hax = gca(h);
                isNewFigure = true;
            else
                hax = p.Results.hax;
                h = hax.Parent();
                isNewFigure = false;
            end
            
            p = p.Results;
            
            dim = size(self.Coordinates,2);
            
            hold on;
            switch dim
                case 2
                    k = chi2inv(diff(normcdf([-1 1]*p.StdBound,0,1)),dim);
                    hp = plot(hax, self.Coordinates(:,1),self.Coordinates(:,2),'Color',p.Color,'LineStyle','none','Marker','+'); 
                    if exist('gtsam','dir') && ~isempty(self.Covariance)
                        for iLmk=1:size(self.Covariance,3)
                            he(iLmk) = gtsam.covarianceEllipse(self.Coordinates(iLmk,:),self.Covariance(:,:,iLmk),p.Color,k); 
                        end
                    else
                        he = [];
                    end
                    if isNewFigure, xlabel(hax,'x in [m]'); ylabel(hax,'y in [m]'); end
                case 3
                    k = diff(normcdf([-1 1]*p.StdBound,0,1));
                    hp = plot3(hax, self.Coordinates(:,1),self.Coordinates(:,2),self.Coordinates(:,3),'Color',p.Color,'LineStyle','none','Marker','+'); 
                    %if exist('libmix4sam/externals.
                    isErrorEllipseExists = ~isempty(which('libmix4sam.externals.error_ellipse'));
                    if ~isempty(self.Covariance)
                        if isErrorEllipseExists
                            for iLmk=1:size(self.Covariance,3)
                                he(iLmk) = libmix4sam.externals.error_ellipse(self.Covariance(:,:,iLmk),'mu',self.Coordinates(iLmk,:),'conf',k);
                            end
                        else
                            error('You need to have the function error_ellipse from matlab file exchange within the libmix4sam.externals package!');
                        end
                    else
                        he = [];
                    end
                    if isNewFigure, xlabel(hax,'x'); ylabel(hax,'y'); zlabel(hax,'z'); end
                otherwise
                    error('Not implemented yet!');
            end
                
            % Modify hggroup for legend information.
            hg = hggroup; for i=he, set(i,'Parent',hg); end
            set(get(get(hg,'Annotation'),'LegendInformation'), 'IconDisplayStyle','on'); 
        end
        
        function h = plotAsRegistrationProblem(self, varargin)
            if length(self) ~= 2, error('Plot as registration problem needs exactly two Landmark sets!'); end
            previous = self(1);
            current = self(2);
            
            p = inputParser;
            addParameter(p, 'Name','Landmark Registration Problem', @ischar);
            addParameter(p, 'Transformation', [], @(x)validateattributes(x,{'numeric'},{'numel',3}))
            parse(p,varargin{:}); 
            
            h = figure('Name',p.Results.Name);
            ax = gca(h);
            xlabel(ax,'x in [m]'); ylabel(ax,'y in [m]');
            
            [~, hp, he] = previous.plot(ax,'k','StdBound',1);
            set(he.Children,'LineWidth',1.5,'Color',ones(1,3)*0.4,'LineStyle',':');
            set(he,{'DisplayName'},{'Previous 1\sigma-ellipse'});
            set(hp,'Marker','o','DisplayName','Previous mean');
            [~, hc, he] = current.plot(ax,'r','StdBound',1);
            set(he.Children,'LineWidth',1.5);
            set(he,{'DisplayName'},{'Current 1\sigma-ellipse'});
            set(hc,'DisplayName','Current mean');
            % tmp, todo
            if (isempty(previous.Id) && isempty(current.Id))
                id = 1:size(previous.Coordinates,1);
                previous.Id = id(:);
                current.Id = id(:);
            end
            if ~isempty(previous.Id) && ~isempty(current.Id)
                [~,ia,ib] = intersect(previous.Id,current.Id);
                previous_mean = [get(hp,'XData'); get(hp,'YData')];
                current_mean = [get(hc,'XData'); get(hc,'YData')];
                ia(previous.Id(ia) == -1) = []; % delete outlier indexes
                ib(current.Id(ib) == -1) = []; % delete outlier indexes
                x_data = [previous_mean(1,ia);current_mean(1,ib)];
                y_data = [previous_mean(2,ia);current_mean(2,ib)];
                hg = hggroup(ax,'DisplayName','Correspondences');
                hg.Annotation.LegendInformation.IconDisplayStyle = 'on';
                line(x_data,y_data,'Parent',hg);
                set(hg.Children,'Color',ones(1,3)*0.4);
                ax.Children = circshift(ax.Children,-1);
                
                % Mark Outliers in Previous
                if(any(previous.Id == -1))
                    previous_outliers = previous_mean(:,previous.Id == -1);
                    hg = hggroup(ax,'DisplayName','Outliers Previous');
                    hg.Annotation.LegendInformation.IconDisplayStyle = 'on';
                    plot(previous_outliers(1,:),previous_outliers(2,:),'ro','Parent',hg);
                    set(hg.Children,'MarkerEdgeColor','#8200a6','MarkerFaceColor','#8200a6','MarkerSize',8);
                    ax.Children = circshift(ax.Children,-1);
                end
                
                % Mark Outliers in Current
                if(any(current.Id == -1))
                    current_outliers = current_mean(:,current.Id == -1);
                    hg = hggroup(ax,'DisplayName','Outliers Current');
                    hg.Annotation.LegendInformation.IconDisplayStyle = 'on';
                    plot(current_outliers(1,:),current_outliers(2,:),'ro','Parent',hg);
                    set(hg.Children,'MarkerEdgeColor','m','MarkerFaceColor','m','MarkerSize',8);
                    ax.Children = circshift(ax.Children,-1);
                end
                
            end
            legend;
            
            if ~isempty(p.Results.Transformation)
                % Draw coordinate frames
                trans = -p.Results.Transformation;
                s = 0.8; R=@(x)[cos(x) -sin(x);sin(x) cos(x)];
                extraStyle = {'HeadStyle','vback1','HeadLength',8*s,'HeadWidth',8*s};
                % Previous
                an_x = annotation('arrow','Color',ones(1,3)*0.4); set(an_x,'Parent',gca,'Position',[0,0,0,s]); set(an_x,extraStyle{:});
                an_y = annotation('arrow','Color',ones(1,3)*0.4); set(an_y,'Parent',gca,'Position',[0,0,s,0]); set(an_y,extraStyle{:});
                % Current
                an_x = annotation('arrow','Color','red'); set(an_x,'Parent',gca,'Position',[trans(1),trans(2),(R(trans(3))*[0;1]*s)']); set(an_x,extraStyle{:});
                an_y = annotation('arrow','Color','red'); set(an_y,'Parent',gca,'Position',[trans(1),trans(2),(R(trans(3))*[1;0]*s)']); set(an_y,extraStyle{:});
            end
    
        end
        
    end
    
    methods(Static, Abstract)
        generateMultiple(varargin);
    end

end

