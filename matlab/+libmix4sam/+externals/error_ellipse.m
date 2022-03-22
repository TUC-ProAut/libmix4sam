function h = error_ellipse(varargin)
% ERROR_ELLIPSE - plot an error ellipse, or ellipsoid, defining confidence region
%    ERROR_ELLIPSE(C22) - Given a 2x2 covariance matrix, plot the
%    associated error ellipse, at the origin. It returns a graphics handle
%    of the ellipse that was drawn.
%
%    ERROR_ELLIPSE(C33) - Given a 3x3 covariance matrix, plot the
%    associated error ellipsoid, at the origin, as well as its projections
%    onto the three axes. Returns a vector of 4 graphics handles, for the
%    three ellipses (in the X-Y, Y-Z, and Z-X planes, respectively) and for
%    the ellipsoid.
%
%    ERROR_ELLIPSE(C,MU) - Plot the ellipse, or ellipsoid, centered at MU,
%    a vector whose length should match that of C (which is 2x2 or 3x3).
%
%    ERROR_ELLIPSE(...,'Property1',Value1,'Name2',Value2,...) sets the
%    values of specified properties, including:
%      'C' - Alternate method of specifying the covariance matrix
%      'mu' - Alternate method of specifying the ellipse (-oid) center
%      'conf' - A value betwen 0 and 1 specifying the confidence interval.
%        the default is 0.5 which is the 50% error ellipse.
%      'scale' - Allow the plot the be scaled to difference units.
%      'style' - A plotting style used to format ellipses.
%      'clip' - specifies a clipping radius. Portions of the ellipse, -oid,
%        outside the radius will not be shown.
%      'gca' - specifies the axes handle, if none given, gca() is used.
%
%    NOTES: C must be positive definite for this function to work properly.
%    NOTES: THIS IS A SLIGHTLY MODIFIED VERSION COMPARED TO THE ORIGINAL,
%    AVAILABLE IN MATLAB FILE EXCHANGE.

p = inputParser;

% The covaraince matrix (required)
addRequired(p,'C', @isnumeric);
% Center of ellipse (optional)
addParameter(p,'mu', [], @isnumeric);
% Percent confidence/100
validFct = @(x)validateattributes(x,{'numeric'},{'>',0,'<',1});
addParameter(p,'conf', 0.5, validFct);
% Scale factor, e.g. 1e-3 to plot m as km
addParameter(p,'scale', 1, @isnumeric);
% Plot style
validFct = @(x)assert(iscell(x) || ischar(x),...
    'Plot style must eighter be a simple character array for plot or of name-value cell-type!');
addParameter(p,'style', '', validFct);
% Clipping radius
addParameter(p,'clip', inf, @isnumeric);
% Axis handle
addParameter(p,'gca', [], @ishandle);

parse(p,varargin{:});

C = p.Results.C;
if isempty(p.Results.mu), mu = zeros(length(C),1); else, mu = p.Results.mu; end
conf = p.Results.conf;
scale = p.Results.scale;
style = p.Results.style;
% Um auch aufrufe mit mehreren Style-Angaben zuzulassen, wie: 
%     plot_errorEllipse(...,'style',{'color',[0.9 0.9 0.9]})
if ~iscell(style), style = {style}; end
if isempty(p.Results.gca), hax = gca(); else, hax = p.Results.gca; end

[r,c] = size(C);
if r ~= c || (r ~= 2 && r ~= 3)
  error(['Don''t know what to do with ',num2str(r),'x',num2str(c),' matrix'])
end

x0=mu(1);
y0=mu(2);

% Compute quantile for the desired percentile
k = sqrt(chi2inv(conf,r)); % r is the number of dimensions (degrees of freedom)

hold_state = get(gca,'nextplot');

if r==3 && c==3
  z0=mu(3);
  
  % Make the matrix has positive eigenvalues - else it's not a valid covariance matrix!
  if any(eig(C) <=0)
    warning('The covariance matrix must be positive definite (it has non-positive eigenvalues)')
    h = [];
    return;
  end

  % C is 3x3; extract the 2x2 matricies, and plot the associated error
  % ellipses. They are drawn in space, around the ellipsoid; it may be
  % preferable to draw them on the axes.
  Cxy = C(1:2,1:2);
  Cyz = C(2:3,2:3);
  Czx = C([3 1],[3 1]);

  [x,y,z] = getpoints(Cxy,p.Results.clip);
  h1=plot3(hax,x0+k*x,y0+k*y,z0+k*z,style{:});hold on
  [y,z,x] = getpoints(Cyz,p.Results.clip);
  h2=plot3(hax,x0+k*x,y0+k*y,z0+k*z,style{:});hold on
  [z,x,y] = getpoints(Czx,p.Results.clip);
  h3=plot3(hax,x0+k*x,y0+k*y,z0+k*z,style{:});hold on

  
  [eigvec,eigval] = eig(C);

  [X,Y,Z] = ellipsoid(0,0,0,1,1,1);
  XYZ = [X(:),Y(:),Z(:)]*sqrt(eigval)*eigvec';
  
  X(:) = scale*(k*XYZ(:,1)+x0);
  Y(:) = scale*(k*XYZ(:,2)+y0);
  Z(:) = scale*(k*XYZ(:,3)+z0);
  h4=surf(X,Y,Z);
  colormap gray
  alpha(0.3)
  %camlight
  if nargout
      % Modify hggroup for legend information.
      h = hggroup; 
      set(h1,'Parent',h);
      set(h2,'Parent',h);
      set(h3,'Parent',h);
      set(h4,'Parent',h);
      set(get(get(h,'Annotation'),'LegendInformation'), 'IconDisplayStyle','on'); 
  end
elseif r==2 && c==2
  % Make the matrix has positive eigenvalues - else it's not a valid covariance matrix!
  if any(eig(C) <=0)
    error('The covariance matrix must be positive definite (it has non-positive eigenvalues)')
  end

  [x,y,z] = getpoints(C,p.Results.clip);
  h1=plot(hax,scale*(x0+k*x),scale*(y0+k*y),style{:});
  set(h1,'zdata',z+1)
  if nargout
    h=h1;
  end
else
  error('C (covaraince matrix) must be specified as a 2x2 or 3x3 matrix)')
end
%axis equal

set(gca,'nextplot',hold_state);

%---------------------------------------------------------------
% getpoints - Generate x and y points that define an ellipse, given a 2x2
%   covariance matrix, C. z, if requested, is all zeros with same shape as
%   x and y.
function [x,y,z] = getpoints(C,clipping_radius)

n=100; % Number of points around ellipse
p=0:pi/n:2*pi; % angles around a circle

[eigvec,eigval] = eig(C); % Compute eigen-stuff
xy = [cos(p'),sin(p')] * sqrt(eigval) * eigvec'; % Transformation
x = xy(:,1);
y = xy(:,2);
z = zeros(size(x));

% Clip data to a bounding radius
if nargin >= 2
  r = sqrt(sum(xy.^2,2)); % Euclidian distance (distance from center)
  x(r > clipping_radius) = nan;
  y(r > clipping_radius) = nan;
  z(r > clipping_radius) = nan;
end

