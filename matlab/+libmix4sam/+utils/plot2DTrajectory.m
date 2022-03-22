function h = plot2DTrajectory(values, varargin)
%PLOT2DTRAJECTORY Plots the Pose2's in a values, with optional covariances
%   Finds all the Pose2 objects in the given Values object and plots them
% in increasing key order, connecting consecutive poses with a line.  If
% a Marginals object is given, this function will also plot marginal
% covariance ellipses for each pose.
%
% INPUT:
%   values ...
%   linespec ... (optional)
%   marginals ... (optional)
%   name,value ... 
%
% MODIFIED VERSION FROM gtsam.plot2DTrajectory with legend and others

% @author GTSAM
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

p = inputParser;
addRequired(p,'values');
addOptional(p,'linespec','k*-',@ischar);
addOptional(p,'marginals',[]);
addParameter(p,'name','trajectory',@ischar); % Name for legend
parse(p,values, varargin{:});
p = p.Results;


haveMarginals = ~isempty(p.marginals);

holdstate = ishold;
hold on

% Collect everything which is plotted, to add into hggroup!
hax = gca();
h = hggroup;
hax_obj = findobj(hax);

% Do something very efficient to draw trajectory
if isa(p.values,'gtsam.Pose2')
    poses = libmix4sam.utils.gtClass2num(p.values);
else
    poses = gtsam.utilities.extractPose2(p.values);
end
X = poses(:,1);
Y = poses(:,2);
theta = poses(:,3);
plot(X,Y,p.linespec);

% Quiver can also be vectorized if no marginals asked
if ~haveMarginals
    C = cos(theta);
    S = sin(theta);
    quiver(X,Y,C,S,0.1,p.linespec);
else
    % plotPose2 does both quiver and covariance matrix
    keys = gtsam.KeyVector(p.values.keys);
    for i = 0:keys.size-1
        key = keys.at(i);
        try
            x = p.values.atPose2(key);
            P = p.marginals.marginalCovariance(key);
            gtsam.plotPose2(x,p.linespec(1), P);
        catch err
            % I guess it's not a Pose2
        end
    end
end

% Collext handles to every new plot object and add to hggroup
hax_new = setdiff(findobj(hax),hax_obj);
for i=hax_new, set(i,'Parent',h); end

% Modify hggroup for legend information.
set(get(get(h,'Annotation'),'LegendInformation'), 'IconDisplayStyle','on'); 
set(h,{'DisplayName'},{p.name});


if ~holdstate
    hold off
end

end

