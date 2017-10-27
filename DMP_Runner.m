classdef DMP_Runner  
   properties
      w;    % canonical function weights (weights of gaussians)
      D;    % 1/sigma^2 (inverse variance squared of gaussians)
      c;    % canonical function spacing (means of gaussians)
      
      vd = 0;
      xd = 0;
      zd = 0;
      gd = 0;
      
      % think about these
      g = 0;
      G = 1;
      
      x = 1;
      v = 0;
      z = 0;
      % think about these
      
      y = 0;
      yd = 0;
      ydd = 0;
   end
   methods
       function obj = DMP_Runner(w, D, c)
          obj.w = w;
          obj.D = D;
          obj.c = c;
       end
       function [y yd ydd] = step(obj, tau, dt)
          tau = 1/tau; %because ???
          % the time constants for chosen for critical damping
          alpha_z = 25;
          beta_z  = alpha_z/4;
          alpha_g = alpha_z/2;
          alpha_v = alpha_z;
          beta_v  = beta_z;
           
          psi = exp(-0.5*((obj.x-obj.c).^2).*obj.D);
          in = obj.v;
          f  = sum(in*(obj.w).*psi)/sum(psi+1.e-10);
          
          obj.vd = (alpha_v*(beta_v*(0-obj.x)-obj.v))*tau;
          obj.xd = obj.v*tau;
          
          obj.zd = (alpha_z*(beta_z*(obj.g-obj.y)-obj.z)+f)*tau;
          obj.yd = obj.z*tau;
          obj.ydd= obj.zd*tau;
          
          obj.gd = alpha_g*(obj.G-obj.g);

          obj.x  = obj.xd*dt+obj.x;
          obj.v  = obj.vd*dt+obj.v;

          obj.z  = obj.zd*dt+obj.z;
          obj.y  = obj.yd*dt+obj.y;

          obj.g  = obj.gd*dt+obj.g;
          
          % return stuff here
          y = obj.y;
          yd = obj.yd;
          ydd = obj.ydd;
       end
   end
end
% add back in functional scaling if you want it
% add back in 0th order canonical function if you want it
