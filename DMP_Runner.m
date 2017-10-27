classdef DMP_Runner < handle 
   properties
      w;    % canonical function weights (weights of gaussians)
      D;    % 1/sigma^2 (inverse variance squared of gaussians)
      c;    % canonical function spacing (means of gaussians)
      
      g  = 0;       % progress towards goal (current goal state)
      gd = 0;       % change in goal position
      G  = 1;       % Goal position of DMP
      
      % state variables
      x = 1;
      v = 0;
      z = 0;      
      y = 0;
      vd = 0;
      xd = 0;
      zd = 0;
      
   end
   methods
       % initialize the DMP_Runner object
       % Inputs:
       %    w: vector of weights
       %    D: vector of 1/sigma^2
       %    c: vector of canonical function values
       function obj = DMP_Runner(w, D, c)
          obj.w = w;
          obj.D = D;
          obj.c = c;
       end
       
       % Changes the goal position of the DMP
       % Inputs:
       %    G: goal position
       function setGoal(obj, G)
          obj.G = G;
       end
       
       % Generates one time-step of a DMP trajectory
       % Inputs:
       %    tau: time constant
       %    dt : integration constant
       % Outputs:
       %    [y yd ydd]: current DMP trajectory
       function [y yd ydd] = step(obj, tau, dt)
          tau = 1/tau; % ????????
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
          yd = obj.z*tau;
          ydd= obj.zd*tau;
          
          obj.gd = alpha_g*(obj.G-obj.g);

          obj.x  = obj.xd*dt+obj.x;
          obj.v  = obj.vd*dt+obj.v;

          obj.z  = obj.zd*dt+obj.z;
          obj.y  = yd*dt+obj.y;

          obj.g  = obj.gd*dt+obj.g;
          
          % return stuff here
          y = obj.y;
       end
   end
end
