function [] = train_dmp(name, n_rfs, T, dt)
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
dG = goal - y0;
A  = max(T)-min(T);
s  = 1;  % for fitting a new primitive, the scale factor is always equal to one

Ft  = (Tdd-alpha_z*(beta_z*(G-T)-Td));

% compute the weights for each local model along the trajectory
PSI = exp(-0.5*((X*ones(1,length(c))-ones(length(T),1)*c').^2).*(ones(length(T),1)*D'));

% compute the regression
sx2  = sum(((V.^2)*ones(1,length(c))).*PSI,1)';
sxtd = sum(((V.*Ft)*ones(1,length(c))).*PSI,1)';
w    = sxtd./(sx2+1.e-10);


%% save DMP instance to xml
xml = com.mathworks.xml.XMLUtils.createDocument('DMP');
% 1st level nodes
root = xml.getDocumentElement();

% 2nd level nodes
weights = xml.createElement('weights');
inv_sq_var = xml.createElement('inv_sq_var');
gauss_means = xml.createElement('gauss_means');
dG_xml = xml.createElement('dG');
A_xml = xml.createElement('A');
s_xml = xml.createElement('s');
y0_xml = xml.createElement('y0');

dG_xml.appendChild(xml.createTextNode(num2str(dG,'%f')));
A_xml.appendChild(xml.createTextNode(num2str(A,'%f')));
s_xml.appendChild(xml.createTextNode(num2str(s,'%f')));
y0_xml.appendChild(xml.createTextNode(num2str(y0,'%f')));

%3rd level nodes
for i = 1:length(w)
    wt = xml.createElement('w');
    dt = xml.createElement('d');
    ct = xml.createElement('c');

    wt.appendChild(xml.createTextNode(num2str(w(i),'%f')));
    dt.appendChild(xml.createTextNode(num2str(D(i),'%f')));
    ct.appendChild(xml.createTextNode(num2str(c(i),'%f')));

    weights.appendChild(wt); 
    inv_sq_var.appendChild(dt);
    gauss_means.appendChild(ct);
end

% append 2nd level nodes to root
root.appendChild(weights);
root.appendChild(inv_sq_var);
root.appendChild(gauss_means);
root.appendChild(dG_xml);
root.appendChild(A_xml);
root.appendChild(s_xml);
root.appendChild(y0_xml);

% write to file
xmlwrite(name, xml);