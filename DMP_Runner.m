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
      
      dG;
      A;
      s;
      y0;
      
   end
   methods
       % initialize the DMP_Runner object
       % Inputs:
       %    w: vector of weights
       %    D: vector of 1/sigma^2
       %    c: vector of canonical function values
       function obj = DMP_Runner(fileName)
          [obj.w, obj.D, obj.c, obj.dG, obj.A, obj.s, obj.y0] = readInXml(fileName);
       end
       
       % Changes the goal position of the DMP
       % Inputs:
       %    G: goal position
       function setGoal(obj, G, flag)
          obj.G = G;
          if (flag == 1),
            obj.x     = 1;
            obj.y0    = obj.y;
          end
          if (obj.A ~= 0)  % check whether dcp has been fit
            if (obj.A/(abs(obj.dG)+1.e-10) > 2.0) 
        % amplitude-based scaling needs to be set explicity
            else
        % dG based scaling cab work automatically
            obj.s       = (obj.G-obj.y0)/obj.dG;
            end
          end
       end
       
       function setStart(obj, y)
          obj.y = y;
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
          amp = obj.s;
          in = obj.v;
          f  = sum(in*(obj.w).*psi)/sum(psi+1.e-10)*amp;
          
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

function [w, D, c, dG, A, s, y0] = readInXml(fileName);
          xml = xmlread(fileName);
          
          weights = xml.getElementsByTagName('weights').item(0).getChildNodes();
          weight = weights.getFirstChild();
          w = [];
          while ~isempty(weight)
              if strcmp(weight.getNodeName(), 'w')
                  w = [w, str2double(weight.getTextContent)];
              end
              weight = weight.getNextSibling();
          end
      
          inv_sq_vars = xml.getElementsByTagName('inv_sq_var').item(0).getChildNodes();
          inv_sq_var = inv_sq_vars.getFirstChild();
          D = [];
          while ~isempty(inv_sq_var)
              if strcmp(inv_sq_var.getNodeName(), 'd')
                  D = [D, str2double(inv_sq_var.getTextContent)];
              end
              inv_sq_var = inv_sq_var.getNextSibling();
          end
         
          means = xml.getElementsByTagName('gauss_means').item(0).getChildNodes();
          mean = means.getFirstChild();
          c = [];
          while ~isempty(mean)
              if strcmp(mean.getNodeName(), 'c')
                  c = [c, str2double(mean.getTextContent)];
              end
              mean = mean.getNextSibling();
          end
          
          dG = str2double(xml.getElementsByTagName('dG').item(0).getTextContent());
          A = str2double(xml.getElementsByTagName('A').item(0).getTextContent());
          s = str2double(xml.getElementsByTagName('s').item(0).getTextContent());
          y0 = str2double(xml.getElementsByTagName('y0').item(0).getTextContent());
end
