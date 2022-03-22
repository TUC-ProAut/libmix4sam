classdef OutlierModelSquare < matlab.mixin.Copyable
% OutlierModelSquare Class for generating an approximated uniform
% distribution over a rectangular area.
%
% E.g. if we want to generate an outlier model for a point set registration
% problem, we want to have a uniform distribution for modeling the
% possibility to have an outlier anywhere in the world. To approximate this
% behaviour in gaussian based optimizations, we could use multiple
% gaussians (gaussian mixture) to approximate this uniform distribution
% within a given area.
%
% In the given example, we generate an appriximated uniform distribution
% within an area of 10 by 10. By giving the resolution parameter, we can
% finetune this approximation.
%
% EXAMPLE: 
%     test = psr2D.OutlierModelSquare([10 10],'Resolution',[3 4]); 
%     test.plot

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
        Size;
        Center;
        Resolution;
    end
    properties(Dependent=true, SetAccess=private)
        Mixture;
        Points;
        DistBetween;
    end
    
    methods
        function self = OutlierModelSquare(varargin)
            p = inputParser;
            addOptional(p, 'Size', [10 10], @(x)validateattributes(x,{'numeric'},{'numel',2}));
            addParameter(p, 'Center', [], @(x)validateattributes(x,{'numeric'},{'numel',2})||isempty(x));
            % Number of points [x, y]
            addParameter(p, 'Resolution', [3 3], @(x)validateattributes(x,{'numeric'},{'integer','numel',2}));
            parse(p,varargin{:});
            p = p.Results;
            
            self.Size = p.Size;
            if isempty(p.Center)
                self.Center = self.Size / 2;
            else
                self.Center = p.Center;
            end
            self.Resolution = p.Resolution;
        end
        
        function distance = get.DistBetween(self)
            % Distance between generated Points in x and y direction
            distance = self.Size./(self.Resolution);
        end
        
        function points = get.Points(self)
            [x,y] = meshgrid(1:self.Resolution(1),1:self.Resolution(2));
            x = self.DistBetween(1)*(x-1/2); y = self.DistBetween(2)*(y-1/2);
            x = x-self.Center(1); y = y-self.Center(2);
            points = [x(:),y(:)];
        end
        
        function mix = get.Mixture(self)
            % Number of distributions to generate
            n = prod(self.Resolution);
            % Weight for each distribution
            w = 1/n;
            % Generate mixture
            mix = libmix4sam.Mixture();
            points = self.Points;
            noise = gtsam.noiseModel.Diagonal.Sigmas(self.DistBetween(:)*2);
            for iPoint = 1:n
                % generate GMM components
                mix.add(libmix4sam.MixComponent( noise, w, points(iPoint,:)' ));
            end
        end
        

    end
end
