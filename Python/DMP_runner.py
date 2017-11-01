from xml.etree import ElementTree
import numpy as np
import re

class DMP_runner():
       #     filename: full path to .xml file containing trained DMP
       #    start   : desired starting coordinate of DMP
       #    goal    : desired ending coordinate of DMP
    def __init__(self,file_name,start,goal):
        readInXML(self, file_name)
        self.g = 0 #progress towards goal (current goal state)
        self.gd = 0 #change in goal position
        self.G = 1 #Goal position of DMP
        
        #state variables
        self.x = 1
        self.v = 0
        self.z = 0      
        self.y = 0
        self.vd = 0
        self.xd = 0
        self.zd = 0
        #self.start = start
        #self.goal = goal
        self.flag = 0
        
        setStart(self,start)
        setGoal(self,goal,1)

       # Generates one time-step of a DMP trajectory
       # Inputs:
       #    tau: time constant
       #    dt : integration constant
       # Outputs:
       #    [y yd ydd]: current DMP trajectory
    
    def step(self,tau_old,dt):
        
        tau = np.divide(float(1),tau_old)
        alpha_z = 25
        beta_z  = np.divide(alpha_z,float(4))
        alpha_g = np.divide(alpha_z,float(2))
        alpha_v = alpha_z
        beta_v  = beta_z      
        
        psi = np.exp(np.multiply(-0.5,np.multiply(np.power((np.array(self.c)-self.x),2),np.array(self.D))))
        
        print 
        amp = self.s
        In = self.v

        f  = np.divide(np.sum(np.dot(np.dot(In,(self.w)),psi)), np.multiply(np.sum(psi+1.e-10),amp))

        self.vd = np.multiply(np.multiply(alpha_v,(np.multiply(beta_v,(0-self.x))-self.v)),tau)
        
        self.xd = np.multiply(self.v,tau)
        
        #PROBLEM HERE
        #obj.zd = (alpha_z*(beta_z*A-obj.z)+f)*tau;
        self.zd = np.multiply((np.multiply(alpha_z,(np.multiply(beta_z,self.g-self.y)-self.z))+f),tau)
        yd = np.multiply(self.z,tau)
        ydd= np.multiply(self.zd,tau)
        
        self.gd = np.multiply(alpha_g,(self.G-self.g))
        
        self.x  = np.multiply(self.xd,dt)+self.x
        self.v  = np.multiply(self.vd,dt)+self.v
        
        self.z  = np.multiply(self.zd,dt)+self.z
        self.y  = np.multiply(yd,dt)+self.y
        
        self.g  = np.multiply(self.gd,dt)+self.g
        
        #Return o/p trajectory
        y = self.y
        return y,yd,ydd
        
    
#Reads in the XML file
        
def readInXML(runner, filename):
    runner.w = []
    runner.D = []
    runner.c= []
    runner.A = 0
    runner.dG = 0
    runner.s = 0
    runner.y0 = 0
    
    tree = ElementTree.parse(filename).getroot()	
    for weights in tree.findall("Weights"):
		     for wt in weights.findall("w"):
			     runner.w.append(float(wt.text))
        
    for inv_sq in tree.findall("inv_sq_var"):
		     for inv_sq in inv_sq.findall("D"):
			     runner.D.append(float(inv_sq.text))
        
    for mean in tree.findall("gauss_means"):
		     for m in mean.findall("c"):
			     runner.c.append(float(m.text))
    
    for dG in tree.iter('dG'):
        runner.dG = ElementTree.tostring(dG)
        runner.dG = np.int(''.join(i for i in runner.dG if i.isdigit()))
        
    for A in tree.findall("A"):
        runner.A = ElementTree.tostring(A)
        runner.A = np.int(''.join(i for i in runner.A if i.isdigit()))

    for s in tree.findall("s"):
        runner.s = ElementTree.tostring(s)
        runner.s = np.int(''.join(i for i in runner.s if i.isdigit()))
        
    for y0 in tree.findall("y0"):
        runner.y0 = ElementTree.tostring(y0)
        runner.y0 = np.int(''.join(i for i in runner.y0 if i.isdigit()))
        
    
def setStart(self,start_val):
    self.y = start_val
    
    
    
    
       # Changes the goal position of the DMP
       # Inputs:
       #    G   : goal position
       #    flag: 1 initial setting; 0 for mid-run update
    
def setGoal(self,goal_val,flag):
    self.G = goal_val
    
    if (flag == 1):
        self.x = 1
        self.y0 = self.y
        
    if (self.A != 0): # check whether dcp has been fit
        if (np.divide(float(self.A),(np.abs(self.dG)+(1.e-10))) > 2.0):
            # amplitude-based scaling needs to be set explicity
            pass
        else:
            #dG based scaling cab work automatically
            self.s = np.divide((self.G-self.y0),float(self.dG))


            
        
    