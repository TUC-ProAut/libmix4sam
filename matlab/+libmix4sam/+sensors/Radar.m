classdef Radar < matlab.mixin.Copyable
%RADAR Simple radar sensor class for handling and visualizing radar data.
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
        Time         = []; % Rosbag timestamp

    end
    
    properties (SetAccess = private)
        Phi          = []; % azimut angle
        Rho          = []; % distance
        Doppler      = []; % Doppler velocity
        StdPhi       = []; % angle standard deviation
        StdRho       = []; % distance standard deviation
        StdDoppler   = []; % Doppler velocities' standard deviation
        StdCycleTime = []; % How much do we trust the CycleTime (usually comming from the timestamps)
        Id           = []; 
        CycleTime    = []; % delta t between last and current Scan
        Cartesian    = []; % Cartesian coordinates as struct with x:Nx1 y:Nx1 P:2x2xN
        T_BS         = []; % Calibration information (Sensor position and orientation within Body Frame), struct with scalars x,y,alpha
    end
    
    properties (Dependent = true)
        CartesianCalibrated;   % Get method to this property returns the targets within the body frame, according to T_BS
    end
    
    methods
        function self = Radar(phi, rho, doppler, std_phi, std_rho, std_doppler, dt, id)
        %RADAR Construct an instance of this class.
        
            self.Phi = phi;
            self.Rho = rho;
            self.Doppler = doppler;
            self.StdPhi = std_phi;
            self.StdRho = std_rho;
            self.StdDoppler = std_doppler;
            self.StdCycleTime = 0;
            if nargin > 6, self.CycleTime = dt; end
            if nargin > 7, self.Id = id; end
        end
        
        function set.Time(self,t), self.Time = t; end
        
        function setStdCycleTime(self,v), self.StdCycleTime = v; end
        
        function setCalibration(self, x_B, y_B, alpha)
            self.T_BS = struct('x', x_B, 'y', y_B, 'alpha', alpha);
        end
        
        function v = hasCalibration(self), v = ~isempty(self.T_BS); end
        
        function s = getSize(self), s = length(self.Phi); end
               
        function T = getT_BS(self)
        %GETT_BS Return calibration information as Pose2 type.
            T = gtsam.Pose2(self.T_BS.x, self.T_BS.y, self.T_BS.alpha);
        end
        
        function cartesianOverride(self,x,y,P)
        %CARTESIANOVERRIDE Be carefull! With this mehtod, it is possible to
        %   manually calculate the cartesian coordinates of the targets.
        %   Instead of using the class' internal functionality.
            self.Cartesian = struct('x',x,'y',y,'P',P);
        end
        
        function [s] = get.Cartesian(self)
        %Retruns the cartesian coordinates, which are calculated from polar
        %   coordinates within the first request, if not present.
            if isempty(self.Cartesian)
                s = struct('x',[],'y',[],'P',[]);
                [s.x, s.y] = pol2cart(self.Phi, self.Rho);
                if ~isempty(self.StdPhi) && ~isempty(self.StdRho)
                    % Covariance given as variance for each dimension
                    % [sigma_theta^2 sigma_rho^2]xN
                    N = length(s.x); % Number of landmarks
                    cov = reshape([self.StdPhi.^2 self.StdRho.^2]',1,2,[]) .* repmat(eye(2),[1,1,N]); % Save as covariance matrices
                    % syms t r real; [x,y] = pol2cart(t,r); jacobian([x;y],[t r])
                    J_xy = arrayfun(@(t,r)[-r*sin(t), cos(t); r*cos(t), sin(t)],self.Phi,self.Rho,'UniformOutput',false);
                    cov_xy = cellfun(@(J,P) round(J*P*J',10), reshape(J_xy,1,1,[]), num2cell(cov,[1 2]), 'UniformOutput', false); % round is because of numerical errors, which could lead to unsymmetric covariance matrices
                    s.P = cell2mat(cov_xy); % Convert into Cartesian space
                end
                self.Cartesian = s;
            else
                s = self.Cartesian;
            end
        end
        
        function [s] = get.CartesianCalibrated(self)
        %Returns the cartesian coordinates, but includes also the sensor's
        %   calibration information. E.g. the the targets get transformed
        %   by the calibration information.
            s = self.Cartesian(); 
            if ~self.hasCalibration(), return; end
            su = s;                                % uncalibrated targets
            sa = sin(self.T_BS.alpha);             % Helper
            ca = cos(self.T_BS.alpha);             % Helper
            s.x = ca*su.x - sa*su.y + self.T_BS.x; % Calibrated x-targets
            s.y = sa*su.x + ca*su.y + self.T_BS.y; % Calibrated y-targets
            R = [ca -sa;sa ca];                    % Helper (Jacobian)
            s.P = cell2mat( cellfun(@(x)round(R*x*R',10),num2cell(su.P,[1 2]),'UniformOutput',false) ); % Rotated Covariance Matrix
        end
        
        function radar = getNoisy(self, s)
        %GETNOISY Returns a new radar sensor class with added noise.
        %   Meaning, we assume, that the targets are based on ground truth.
        %   We use the given standard deviations to sample new points and
        %   get a simulated sensor measurement.
            if nargin < 2, s = []; end
            if isempty(s), s = 'shuffle'; end
            r = RandStream('mt19937ar','Seed',s);
            
            selfType = mfilename('class');
            N = length(self.Phi);
            phi = self.Phi + randn(r,N,1) .* self.StdPhi;
            rho = self.Rho + randn(r,N,1) .* self.StdRho;
            if isempty(self.Doppler) || isempty(self.StdDoppler)
                doppler = []; 
            else
                doppler = self.Doppler + randn(r,N,1) .* self.StdDoppler;
            end
            radar = feval(selfType, phi, rho, doppler, self.StdPhi, self.StdRho, self.StdDoppler, self.CycleTime);
            radar.Id = self.Id;
        end
        
        function mix = getMixture(self,weightMethod)
        %GETMIXTURE Converts the targets into one GMM and returns this as
        %   Mixture class.
            if nargin < 2, weightMethod = 'equal'; end % equal / norm
            mix = libmix4sam.Mixture();
            cart = self.Cartesian;
            n = length(cart.x);
            if strcmp(weightMethod,'norm')
                w = cellfun(@(x)sqrt(det(x)),reshape(num2cell(cart.P,[1 2]),[],1));
                w = w/sum(w);
            else
                w = repmat(1/n,n,1);
            end
            for j=1:n
                % One target from reference measurement, modelled as GMM-component.
                refNoise = gtsam.noiseModel.Gaussian.Covariance(cart.P(:,:,j));
                mix.add(libmix4sam.MixComponent( refNoise, w(j), [cart.x(j) cart.y(j)]' ));
            end
        end
        
        function addOutliers(self, percentage, s, fov, distance)
        %ADDOUTLIERS Add outliers within the FoV of the sensor.
            if nargin < 3, s = []; end
            if nargin < 4, fov = [min(self.Phi) max(self.Phi)]; end
            if nargin < 5, distance = [min(self.Rho) max(self.Rho)]; end
            
            if isempty(s), s = 'shuffle'; end
            r = RandStream('mt19937ar','Seed',s);
            
            N = round(length(self.Phi) * percentage);
            phi = rand(r,N,1) * diff(fov)+min(fov);
            rho = rand(r,N,1) * diff(distance)+min(distance);
            idx = r.randperm(length(self.StdPhi),N);
            
            self.Phi = [self.Phi; phi];
            self.Rho = [self.Rho; rho];
            self.StdPhi = [self.StdPhi; self.StdPhi(idx(:))];
            self.StdRho = [self.StdRho; self.StdRho(idx(:))];
            if ~isempty(self.Doppler) && ~isempty(self.StdDoppler)
                self.Doppler = [self.Doppler; mean(self.Doppler) + std(self.Doppler)*randn(r,N,1)];
                self.StdDoppler = [self.StdDoppler; self.StdDoppler(idx(:))];
            end
            
            self.Cartesian = [];
        end
        
        function [h, hp, hg] = plot(self,hax,color,transform)
        %PLOT Minimalistic plot method for the sensor information.
        
            if nargin < 2 || isempty(hax)
                h=figure('Name','Radar'); 
                grid on; axis equal; hold on;
                hax = gca(h);
            else
                h = hax.Parent();
            end
            if nargin < 3 || isempty(color), color = 'r'; end
            
            xyP = self.Cartesian;
            if nargin > 3 && ~isempty(transform)
                xy = transform.rotation.matrix*[xyP.x';xyP.y'] + transform.translation.vector;
                xyP.x(:) = xy(1,:); xyP.y(:) = xy(2,:);
            end

            hp = plot(hax, xyP.x, xyP.y, 'Color', color, 'LineStyle', 'none', 'Marker', '+'); he = [];
            if exist('gtsam','dir') && ~isempty(xyP.P)
                for iLmk=size(xyP.P,3):-1:1
                    he(iLmk) = gtsam.covarianceEllipse([xyP.x(iLmk); xyP.y(iLmk)], xyP.P(:,:,iLmk), color);
                end
            end
            
            % Modify hggroup for legend information.
            hg = hggroup; for i=he, set(i,'Parent',hg); end
            set(get(get(hg,'Annotation'),'LegendInformation'), 'IconDisplayStyle','on'); 
            
            % Plot doppler
            if ~isempty( self.Doppler )
                Vdop = [xyP.x, xyP.y]./self.Rho.*self.Doppler;
                quiver(hax, xyP.x, xyP.y, Vdop(:,1), Vdop(:,2), 0, 'Color', color, 'ShowArrowHead', 'off', 'DisplayName','Doppler');
            end
            
            xl = get(hax,'xLim');
            set(hax,'xLim',[min([0 xl(1)]),xl(2)]);
        end
        
        function [ids] = getCorrespondenceIds(this, other)
        %GETCORRESPONDENCEIDS Return all IDs which are present in both,
        %   previous and current.
            [~,ia] = intersect(this.Id,other.Id);
            ia(this.Id(ia) == -1) = []; % delete outlier indexes
            ids = this.Id(ia);
        end
        
        function addIDs(self, ids), self.Id = ids; end
        
        function [xy,PP] = getTransformedCartesian(self, pose)
        %GETTRANSFORMEDCARTESIAN Transorm points and covariance.
            R=@(x)[cos(x) -sin(x);sin(x) cos(x)];
            xy = [self.Cartesian.x';self.Cartesian.y'];
            PP = self.Cartesian.P;
            xy = (R(pose.theta)*xy + [pose.x; pose.y])';
            J = R(pose.theta);
            cov = cellfun(@(P) J*P*J', num2cell(PP,[1 2]), 'UniformOutput', false);
            PP = cell2mat(cov);
        end
    end
    
    methods (Static)
        function self = generate(varargin)
        %GENERATE A radar sensor class based on given landmark information.
        %   Additional parameters for time and Doppler information can be
        %   provided.
            p = inputParser;
            addRequired(p, 'lms', @(x)isa(x,'libmix4sam.utils.Landmarks2D')); % Landmarks class
            addOptional(p, 'Noise', [],@(x)isempty(x) || isa(x,'gtsam.noiseModel.Unit') || isa(x,'gtsam.noiseModel.Diagonal')); % Nosie model for each dimension [phi r doppler]
            addParameter(p,'Motion',[],@isnumeric); % [x in m/s y in m/s omega in rad/s]
            addParameter(p,'dt',    [],@isnumeric); % delta t between last and current scan
            addParameter(p,'id',    [],@isnumeric); % delta t between last and current scan
            parse(p,varargin{:});
            p = p.Results;
            
            selfType = mfilename('class');
            
            [phi, r] = p.lms.getPolarCoordinates();
            if ~isempty(p.Motion)
                doppler = libmix4sam.sensors.genDopplerVelocity([phi(:),r(:)],p.Motion);
            else
                doppler = [];
            end
            if ~isempty(p.Noise)
                if length(p.Noise) == length(phi)
                    std_all = cell2mat(arrayfun(@(a)a.sigmas,p.Noise,'UniformOutput',false))';
                    std_phi = std_all(:,1);
                    std_rho = std_all(:,2);
                    std_doppler = std_all(:,3);
                else
                    s = p.Noise.sigmas();
                    std_phi = repmat(s(1),length(phi),1);
                    std_rho = repmat(s(2),length(phi),1);
                    std_doppler = repmat(s(3),length(phi),1);
                end
            else
                std_phi = [];
                std_rho = [];
                std_doppler = [];
            end
            self = feval(selfType, phi, r, doppler, std_phi, std_rho, std_doppler, p.dt, p.id);
            
        end
        
        function self = generateOutlierDistribution(alpha, beta, range, fov)
        %GENERATEOUTLIERDISTRIBUTION Generates a sensor class to reflect a
        %   uniform distribution over the sensor's fov and range.
        %
        %   Based on:
        %   Solà, J. (2007) Towards Visual Localization, Mapping and Moving
        %   Objects Tracking by a Mobile Robot: a Geometric and
        %   Probabilistic Approach. phdthesis. LAAS, Toulouse.
        %   Page: 107
            if nargin < 1, alpha = 0.4; end
            if nargin < 2, beta = 1.8; end
            if nargin < 3, range = [0.5 50]; end
            if nargin < 4, fov = 80*pi/180; end
            
            selfType = mfilename('class');
            rho = range(1) / (1-alpha);
            std = alpha * rho(1);
            Ng = 1+ceil(log((1-alpha)/(1+alpha)*range(2)/range(1))/log(beta));
            for i=2:Ng 
                rho(i) = beta * rho(i-1);
                std(i) = alpha * rho(i);
            end
            
            self = feval(selfType, zeros(length(rho),1), rho(:), [], repmat(fov/2,length(rho),1)*3, std(:)*2, []);
        end
        
        function h = plotRegistration(ax, previous, c1, current, c2, transform)
        %PLOTREGISTRATION Plots two radar sensor measurements with
        %   correspondence information, if available.
            if isempty(ax)
                h=figure('Name','Radar Registration'); 
                grid on; axis equal; hold on;
                xlabel('x in m'); ylabel('y in m');
                ax = gca(h);
            else
                h = ax.Parent();
            end
            if isempty(c1), c1 = 'k'; end
            if isempty(c2), c2 = 'r'; end
            [~, hp, he] = previous.plot(ax,c1);
            set(he.Children,'LineWidth',1.5,'Color',ones(1,3)*0.4,'LineStyle',':');
            set(he,{'DisplayName'},{'Previous 1\sigma-ellipse'});
            set(hp,'Marker','o','DisplayName','Previous mean');
            if ~isa(transform,'gtsam.Pose2')
                transform = gtsam.Pose2(transform(1),transform(2),transform(3));
            end
            [~, hc, he] = current.plot(ax, c2, transform);
            set(he.Children,'LineWidth',1.5);
            set(he,{'DisplayName'},{'Transformed current 1\sigma-ellipse'});
            set(hc,'DisplayName','Transformed current mean');
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
            end
            legend;
        end
        
        function [ids] = GetCorrespondenceIds(previous, current)
        %GETCORRESPONDENCEIDS Return all IDs which are present in both,
        %   previous and current. 
            [~,ia] = intersect(previous.Id,current.Id);
            ia(previous.Id(ia) == -1) = []; % delete outlier indexes
            ids = previous.Id(ia);
        end
        
        function [previousIDs, currentIDs] = EstimateCorrespondenceIds(varargin)
        %ESTIMATECORRESPONDENCEIDS Estimate the correspondence between two
        %   scans based on the Bhattacharyya distance.
            selfType = mfilename('class');
            p = inputParser;
            addRequired(p,'previous',@(x)isa(x,selfType));
            addRequired(p,'current',@(x)isa(x,selfType));
            addOptional(p,'transformation',[0 0 0]',@isnumeric); %[x,y,theta] 
            addParameter(p,'threshold',1,@isnumeric); 
            parse(p,varargin{:});
            p = p.Results; current = p.current; previous = p.previous;
            
            % TODO apply transformation first
            if ~isa(p.transformation, 'gtsam.Pose2')
                p.transformation = gtsam.Pose2(p.transformation(1),p.transformation(2),p.transformation(3));
            end
            
            [mu1,P1] = current.getTransformedCartesian(p.transformation);
            %mu1 = [current.Cartesian.x';current.Cartesian.y'];
            %P1 = current.Cartesian.P;
            
            mu2 = [previous.Cartesian.x';previous.Cartesian.y'];
            P2 = previous.Cartesian.P;

            dist = libmix4sam.utils.bhattacharyyaDist(mu1', P1, mu2, P2);
            % Find for each current the nearest previous
            dist = sortrows(dist,3);
            [~, corrInd] = unique(dist(:,1),'first');
            dist = dist(corrInd,:);
            % Apply threshold
            dist(dist(:,3) > p.threshold, :) = [];

            % Generate IDs (-1 is no correspondence)
            currentIDs = repmat(-1, current.getSize(), 1);
            previousIDs = repmat(-1, previous.getSize(), 1);
            currentIDs(dist(:,1)) = dist(:,1);
            previousIDs(dist(:,2)) = dist(:,1);
        end
    end
    
    methods (Static)
        
        function selfArray = ConvertNuScenesData(data)
        %CONVERTNUSCENESDATA Convert data from nuScenes into equivalent
        %   radar sensor data.
            selfType = mfilename('class');
            
            if all(isfield(data(1),{'rho','phi','doppler','rho_rms','phi_rms','doppler_rms'}))
                fcn = @convertSinglePolar; % Targets and it's uncertainties are given in polar coordinates
            else
                fcn = @convertSingleCart; % Targets are only given in cartesian coordinates
            end
            
            t = double([data(:).TimeStamp]')./1e6;
            dt = [0; diff(t)];
            selfArray = arrayfun(fcn, data(:), t, dt);
            
            for i=1:numel(selfArray)
                selfArray(i).setStdCycleTime(0.005); % How much do we trust the timestamps? TODO: Not completely evaluated, how big the value should be.
            end
                
            % Calibrierungsinformationen vorhanden
            if all(isfield(data(1),'T_BS'))
                for i=1:numel(selfArray)
                    e = quat2eul(quaternion(data(i).T_BS.qw,data(i).T_BS.qx,data(i).T_BS.qy,data(i).T_BS.qz)); % Sequence: ZYX
                    selfArray(i).setCalibration(data(i).T_BS.x, data(i).T_BS.y, e(1));
                end
            end
            
            function self = convertSingleCart(data, t, dt)
                [phi,rho] = cart2pol(data.X,data.Y);
                doppler = data.doppler;
                std_doppler = data.doppler_rms;
                %std_doppler = repmat(0.2, numel(doppler),1); % Evaluated over all measurements by using ground truth information
                std_phi = repmat(0.1,length(data.Vx),1); % Only dummy, will be overwritten later on.
                self = feval(selfType, phi, rho, doppler, std_phi, [], std_doppler, dt);
                self.Time = t;
                % Use nuScenes Cartesian covariance information.
                P = arrayfun(@(x,y)diag([x,y]),data.X_rms.^2,data.Y_rms.^2,'UniformOutput',false);
                self.cartesianOverride(data.X(:),data.Y(:),reshape(cell2mat(P)',2,2,[]));
            end

            function self = convertSinglePolar(data, t, dt)
                rho = data.rho;
                phi = data.phi;
                doppler = data.doppler;
                std_rho = data.rho_rms;
                std_phi = data.phi_rms;
                std_doppler = data.doppler_rms;
                self = feval(selfType, phi, rho, doppler, std_phi, std_rho, std_doppler, dt);
                self.Time = t;
            end
            
        end
        
    end
    
end

