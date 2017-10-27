function [w, D, c] = train_dmp(name, n_rfs, T, dt)
% name       : filename for trained dmp to be stored into
% n_rfs      : number of basis functions to train dmp with
% T          : training trajectory formatted: [Y; dY; ddY];
% dt         : timestep between each sequential trajectory point
% .........................................................................

% Extract trajectory into pos, vel, accl vectors
Tdd     = T(:,3);
Td      = T(:,2);
T       = T(:,1);

% the time constants for chosen for critical damping
alpha_z = 25;
beta_z  = alpha_z/4;
alpha_g = alpha_z/2;
alpha_v = alpha_z;
beta_v  = beta_z;

% canonical function with x and xd solutions of:
% x(t) = 1-(1+alpha/2*t)*exp(-alpha/2*t)
% xd(t) = (-alpha/2)*x(t) + alpha/2*exp(-alpha/2*t)
t = (0:1/(n_rfs-1):1)';
c       = (1+alpha_z/2*t).*exp(-alpha_z/2*t);

% Misc. Memory Allocations
D       = (diff(c)*0.55).^2;
D       = 1./[D;D(end)];
    
% .........................................................................    
% the start state is the first state in the trajectory
y0 = T(1);
g  = y0;
goal = T(end);

% compute the hidden states
X = zeros(size(T));
V = zeros(size(T));
G = zeros(size(T));
x = 1; v = 0;
for i=1:length(T)
  X(i) = x;
  V(i) = v;
  G(i) = g;

  vd   = alpha_v*(beta_v*(0-x)-v);
  xd   = v;

  gd   = (goal - g) * alpha_g;

  x    = xd*dt+x;
  v    = vd*dt+v;
  g    = gd*dt+g;
end

Ft  = (Tdd-alpha_z*(beta_z*(G-T)-Td));

% compute the weights for each local model along the trajectory
PSI = exp(-0.5*((X*ones(1,length(c))-ones(length(T),1)*c').^2).*(ones(length(T),1)*D'));

% compute the regression
sx2  = sum(((V.^2)*ones(1,length(c))).*PSI,1)';
sxtd = sum(((V.*Ft)*ones(1,length(c))).*PSI,1)';
w    = sxtd./(sx2+1.e-10);