function [ret] = bhattacharyyaDist(mu1, P1, mu2, P2)
%BHATTACHARYYADIST Calculate the Bhattacharyya distance between two normal
%   distributions.

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

    N1 = size(mu1,2);
    N2 = size(mu2,2);
    [i1, i2] = meshgrid(1:N1,1:N2);
    combinations = [i1(:),i2(:)];

    covMean = (P1(:,:,combinations(:,1)) + P2(:,:,combinations(:,2)))./2;
    meanDiff = mu1(:,combinations(:,1)) - mu2(:,combinations(:,2));

    dists = zeros(size(combinations,1),1);
    for i=1:size(combinations,1)
        mahala = 0.125 * meanDiff(:,i)' * inv(covMean(:,:,i)) * meanDiff(:,i);
        det1 = det(P1(:,:,combinations(i,1)));
        det2 = det(P2(:,:,combinations(i,2)));
        dists(i) = mahala + 0.5 * log( det(covMean(:,:,i)) / sqrt( det1 * det2 ));
    end

    ret = [combinations dists];

end

