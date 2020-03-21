function copter_dynamics(block)

  
setup(block);
  
%endfunction


function setup(block)

  % Register the number of ports.
  block.NumInputPorts  = 4;
  block.NumOutputPorts = 12;
  
  % Set up the port properties to be inherited or dynamic.
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;

  % Override the input port properties.
  for i = 1:4;
  block.InputPort(i).DatatypeID  = 0; % double
  block.InputPort(i).Dimensions        = 1;
  block.InputPort(i).DirectFeedthrough = 0;
  block.InputPort(i).SamplingMode      = 'Sample';
  block.InputPort(1).Complexity  = 'Real';
  end
  % Override the output port properties.
  for i = 1:12;
  block.OutputPort(i).DatatypeID  = 0; % double
  block.OutputPort(i).Dimensions  = 1;
  block.OutputPort(i).SamplingMode  = 'Sample';
  block.OutputPort(1).Complexity  = 'Real';
  end

  
  

  % Register the parameters.
  block.NumDialogPrms     = 1;
  %block.DialogPrmsTunable = {'Tunable','Nontunable','SimOnlyTunable'};%%change
  
  % Set up the continuous states.
  block.NumContStates = 12;

  % Register the sample times.
  %  [0 offset]            : Continuous sample time
  %  [positive_num offset] : Discrete sample time
  %
  %  [-1, 0]               : Inherited sample time
  %  [-2, 0]               : Variable sample time
  block.SampleTimes = [-1 0];%% changed this to Inherited sample time
  
  % -----------------------------------------------------------------
  % Options
  % -----------------------------------------------------------------
  % Specify if Accelerator should use TLC or call back to the 
  % MATLAB file
  block.SetAccelRunOnTLC(false);
  
  % Specify the block simStateCompliance. The allowed values are:
  %    'UnknownSimState', < The default setting; warn and assume DefaultSimState
  %    'DefaultSimState', < Same SimState as a built-in block
  %    'HasNoSimState',   < No SimState
  %    'CustomSimState',  < Has GetSimState and SetSimState methods
  %    'DisallowSimState' < Errors out when saving or restoring the SimState
  block.SimStateCompliance = 'DefaultSimState';
  
  % -----------------------------------------------------------------
  % The MATLAB S-function uses an internal registry for all
  % block methods. You should register all relevant methods
  % (optional and required) as illustrated below. You may choose
  % any suitable name for the methods and implement these methods
  % as local functions within the same file.
  % -----------------------------------------------------------------
   
  % -----------------------------------------------------------------
  % Register the methods called during update diagram/compilation.
  % -----------------------------------------------------------------
  
  % 
  % CheckParameters:
  %   Functionality    : Called in order to allow validation of the
  %                      block dialog parameters. You are 
  %                      responsible for calling this method
  %                      explicitly at the start of the setup method.
  %   C-Mex counterpart: mdlCheckParameters
  %
  block.RegBlockMethod('CheckParameters', @CheckPrms);

  %
  % SetInputPortSamplingMode:
  %   Functionality    : Check and set input and output port 
  %                      attributes and specify whether the port is operating 
  %                      in sample-based or frame-based mode
  %   C-Mex counterpart: mdlSetInputPortFrameData.
  %   (The DSP System Toolbox is required to set a port as frame-based)
  %
  %block.RegBlockMethod('SetInputPortSamplingMode', @SetInpPortFrameData);
  
  %
  % SetInputPortDimensions:
  %   Functionality    : Check and set the input and optionally the output
  %                      port dimensions.
  %   C-Mex counterpart: mdlSetInputPortDimensionInfo
  %
%  block.RegBlockMethod('SetInputPortDimensions', @SetInpPortDims);

  %
  % SetOutputPortDimensions:
  %   Functionality    : Check and set the output and optionally the input
  %                      port dimensions.
  %   C-Mex counterpart: mdlSetOutputPortDimensionInfo
  %
 % block.RegBlockMethod('SetOutputPortDimensions', @SetOutPortDims);
  
  %
  % SetInputPortDatatype:
  %   Functionality    : Check and set the input and optionally the output
  %                      port datatypes.
  %   C-Mex counterpart: mdlSetInputPortDataType
  %
 % block.RegBlockMethod('SetInputPortDataType', @SetInpPortDataType);
  
  %
  % SetOutputPortDatatype:
  %   Functionality    : Check and set the output and optionally the input
  %                      port datatypes.
  %   C-Mex counterpart: mdlSetOutputPortDataType
  %
 % block.RegBlockMethod('SetOutputPortDataType', @SetOutPortDataType);
  
  %
  % SetInputPortComplexSignal:
  %   Functionality    : Check and set the input and optionally the output
  %                      port complexity attributes.
  %   C-Mex counterpart: mdlSetInputPortComplexSignal
  %
 % block.RegBlockMethod('SetInputPortComplexSignal', @SetInpPortComplexSig);
  
  %
  % SetOutputPortComplexSignal:
  %   Functionality    : Check and set the output and optionally the input
  %                      port complexity attributes.
  %   C-Mex counterpart: mdlSetOutputPortComplexSignal
  %
 % block.RegBlockMethod('SetOutputPortComplexSignal', @SetOutPortComplexSig);
  
  %
  % PostPropagationSetup:
  %   Functionality    : Set up the work areas and the state variables. You can
  %                      also register run-time methods here.
  %   C-Mex counterpart: mdlSetWorkWidths
  %
 % block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);

  % -----------------------------------------------------------------
  % Register methods called at run-time
  % -----------------------------------------------------------------
  
  % 
  % ProcessParameters:
  %   Functionality    : Call to allow an update of run-time parameters.
  %   C-Mex counterpart: mdlProcessParameters
  %  
  %block.RegBlockMethod('ProcessParameters', @ProcessPrms);

  % 
  % InitializeConditions:
  %   Functionality    : Call to initialize the state and the work
  %                      area values.
  %   C-Mex counterpart: mdlInitializeConditions
  % 
  block.RegBlockMethod('InitializeConditions', @InitializeConditions);
  
  % 
  % Start:
  %   Functionality    : Call to initialize the state and the work
  %                      area values.
  %   C-Mex counterpart: mdlStart
  %
 % block.RegBlockMethod('Start', @Start);

  % 
  % Outputs:
  %   Functionality    : Call to generate the block outputs during a
  %                      simulation step.
  %   C-Mex counterpart: mdlOutputs
  %
  block.RegBlockMethod('Outputs', @Outputs);

  % 
  % Update:
  %   Functionality    : Call to update the discrete states
  %                      during a simulation step.
  %   C-Mex counterpart: mdlUpdate
  %
 % block.RegBlockMethod('Update', @Update);

  % 
  % Derivatives:
  %   Functionality    : Call to update the derivatives of the
  %                      continuous states during a simulation step.
  %   C-Mex counterpart: mdlDerivatives
  %
  block.RegBlockMethod('Derivatives', @Derivatives);
  
  % 
  % Projection:
  %   Functionality    : Call to update the projections during a
  %                      simulation step.
  %   C-Mex counterpart: mdlProjections
  %
 % block.RegBlockMethod('Projection', @Projection);
  
  % 
  % SimStatusChange:
  %   Functionality    : Call when simulation enters pause mode
  %                      or leaves pause mode.
  %   C-Mex counterpart: mdlSimStatusChange
  %
  %block.RegBlockMethod('SimStatusChange', @SimStatusChange);
  
  % 
  % Terminate:
  %   Functionality    : Call at the end of a simulation for cleanup.
  %   C-Mex counterpart: mdlTerminate
  %
  block.RegBlockMethod('Terminate', @Terminate);

  %
  % GetSimState:
  %   Functionality    : Return the SimState of the block.
  %   C-Mex counterpart: mdlGetSimState
  %
 % block.RegBlockMethod('GetSimState', @GetSimState);
  
  %
  % SetSimState:
  %   Functionality    : Set the SimState of the block using a given value.
  %   C-Mex counterpart: mdlSetSimState
  %
 % block.RegBlockMethod('SetSimState', @SetSimState);

  % -----------------------------------------------------------------
  % Register the methods called during code generation.
  % -----------------------------------------------------------------
  
  %
  % WriteRTW:
  %   Functionality    : Write specific information to model.rtw file.
  %   C-Mex counterpart: mdlRTW
  %
 % block.RegBlockMethod('WriteRTW', @WriteRTW);
%endfunction

% -------------------------------------------------------------------
% The local functions below are provided to illustrate how you may implement
% the various block methods listed above.
% -------------------------------------------------------------------

function CheckPrms(block)
  
  model= block.DialogPrm(1).Data;
   if ~strcmp(class(model), 'struct') %#ok<STISA>
     me = MSLException(block.BlockHandle, message('Simulink:blocks:invalidParameter'));
     throw(me);
   end
  
%endfunction

%function ProcessPrms(block)

 % block.AutoUpdateRuntimePrms;
 
%endfunction


    
% function DoPostPropSetup(block)
%   block.NumDworks = 2;
%   
%   block.Dwork(1).Name            = 'x1';
%   block.Dwork(1).Dimensions      = 1;
%   block.Dwork(1).DatatypeID      = 0;      % double
%   block.Dwork(1).Complexity      = 'Real'; % real
%   block.Dwork(1).UsedAsDiscState = true;
%   
%   block.Dwork(2).Name            = 'numPause';
%   block.Dwork(2).Dimensions      = 1;
%   block.Dwork(2).DatatypeID      = 7;      % uint32
%   block.Dwork(2).Complexity      = 'Real'; % real
%   block.Dwork(2).UsedAsDiscState = true;
%   
%   % Register all tunable parameters as runtime parameters.
%   block.AutoRegRuntimePrms;
% 
% %endfunction
% 
 function InitializeConditions(block)
 %initial conditions for 12 states
 
 model= block.DialogPrm(1).Data;
 
 %convert to rads
 
 R = model.R*pi/180;
 Q = model.Q*pi/180; 
 P = model.P*pi/180;

Phi = model.Phi*pi/180;
Theta = model.Theta*pi/180;
Psi = model.Psi*pi/180;
 %linear velocity
dX = model.dX;         
dY =model.dY;
dZ =model.dZ; 

x = model.x;
y = model.y;
z = model.z;


init_cond = [P,Q,R,Phi,Theta,Psi,dX,dY,dZ,x,y,z];

for i=1:12
block.OutputPort(i).Data = init_cond(i);
block.ContStates.Data(i) = init_cond(i);

end
 
 %endfunction
% 
% function Start(block)
% 
%   block.Dwork(1).Data = 0;
%   block.Dwork(2).Data = uint32(1); 
%    
% %endfunction
function Outputs(block)
  
for i = 1:12;
  block.OutputPort(i).Data = block.ContStates.Data(i);
  
end


  
%endfunction

function Derivatives(block)

%% here the derivatives of the states would be calculated    
%block.Derivatives.Data = 2*block.ContStates.Data;

model=block.DialogPrm(1).Data;

%extracting the 12 states
% first the angular velocity in body frame
P = block.ContStates.Data(1);
Q=  block.ContStates.Data(2);
R=  block.ContStates.Data(3);

%the angles 
Phi=block.ContStates.Data(4);
Theta=block.ContStates.Data(5);
Psi=block.ContStates.Data(6);

%the linear velocity in body frame
dX=block.ContStates.Data(7);
dY=block.ContStates.Data(8);
dZ=block.ContStates.Data(9);

%the linear position

x=block.ContStates.Data(10);
y=block.ContStates.Data(11);
z=block.ContStates.Data(12);

% The rpm values from motors
w1 = block.InputPort(1).Data;
w2 = block.InputPort(2).Data;
w3 = block.InputPort(3).Data;
w4 = block.InputPort(4).Data;

w  = [w1; w2; w3; w4];
w=w/60*2*pi; %rpm to ang_vel
% disp(w);
T=model.bt*(w.^2); %the  force thrust
% disp(T);
H=model.bh*(w.^2); %the  hub tourque

l=model.l;    %copter arm length
T_tot=sum(T);               %the total thrust
H_tot=[l*(-T(2)+T(4));
       l*(-T(3)+T(1));
       (H(1)+H(3))-(H(2)+H(4))];
   
 Jx=model.Jx;
 Jy=model.Jy;
 Jz=model.Jz;
%calculate angular acceleration P,Q,R    
 dP=(H_tot(1)-(Jz-Jy)*Q*R)/Jx;
 dQ=(H_tot(2)-(Jx-Jz)*P*R)/Jy;
 dR=(H_tot(3)-(Jy-Jx)*P*Q)/Jz;
 
%  Wd=[dP;dQ;dR];
 
 
 %calculate euler angular velocities
  Ro=[1 ,sin(Phi)*tan(Theta) ,cos(Phi)*tan(Theta);
          0, cos(Phi)            ,-sin(Phi);
          0,sin(Phi)/cos(Theta)  , cos(Phi)/cos(Theta)];

Uler_W=Ro*[P;Q;R];

%Rotation matrix Rt=((Rx*Ry*Rz)^-1) displacement from body to inertia
Rt=[cos(Theta)*cos(Psi)-sin(Phi)*sin(Theta)*sin(Psi),-cos(Theta)*sin(Psi), cos(Psi)*sin(Phi)+cos(Theta)*sin(Phi)*sin(Psi);
     cos(Phi)*sin(Psi)+cos(Psi)*sin(Phi)*sin(Theta) ,   cos(Phi)*cos(Psi), sin(Theta)*sin(Psi)-sin(Phi)*cos(Theta)*cos(Psi);
      -cos(Phi)*sin(Theta) ,                           sin(Phi)  ,         cos(Phi)*cos(Theta)             ];

% Rt = [cos(Psi)*cos(Theta) cos(Psi)*sin(Theta)*sin(Phi)-sin(Psi)*cos(Phi) cos(Psi)*sin(Theta)*cos(Phi)+sin(Psi)*sin(Phi);
%        sin(Psi)*cos(Theta) sin(Psi)*sin(Theta)*sin(Phi)+cos(Psi)*cos(Phi) sin(Psi)*sin(Theta)*cos(Phi)-cos(Psi)*sin(Phi);
%        -sin(Theta)         cos(Theta)*sin(Phi)                            cos(Theta)*cos(Phi)];

Vb=[dX;dY;dZ];
VI=Rt*[dX;dY;dZ];



WB_cross = [ 0,-R, Q;
           R, 0,-P;
          -Q, P, 0];


%   Txyz=[0;0;-model.g]+Rt*[0;0;T_tot]/model.m;
% Vxyz=(Rt')*[0;0;-model.g]+[0;0;T_tot]/model.m;
drag=[model.Kd*dX^2;model.Kd*dY^2;model.Kd*dZ^2];

 Vxyz=(Rt')*[0;0;-model.g]+[0;0;T_tot]/model.m-WB_cross*Vb-drag/model.m;

  %velocity in inertial frame 
dX=VI(1);
dY=VI(2);
dZ=VI(3);


%acceleration in body frame
 ddX=Vxyz(1);
 ddY=Vxyz(2);
 ddZ=Vxyz(3);
 
  
dPhi=Uler_W(1);
dTheta=Uler_W(2);
dPsi=Uler_W(3);

%[P,Q,R,Phi,Theta,Psi,Xd,Yd,Zd,x,y,z];
% grounding condition
if ((z<=0) && (dZ<=0)) 
    dZ = 0;
    block.ContStates.Data(12) = 0;
end
dstates = [dP dQ dR dPhi dTheta dPsi ddX ddY ddZ dX dY dZ].';
  %This is the state derivative vector
block.Derivatives.Data = dstates;


  
  
  
  
%endfunction

% function Projection(block)
% 
% states = block.ContStates.Data;
% block.ContStates.Data = states+eps; 
% 
% endfunction

% function SimStatusChange(block, s)
%   
%   block.Dwork(2).Data = block.Dwork(2).Data+1;    
% 
%   if s == 0
%     disp('Pause in simulation.');
%   elseif s == 1
%     disp('Resume simulation.');
%   end
%   
%endfunction
    
function Terminate(block)

disp(['Terminating the block with handle ' num2str(block.BlockHandle) '.']);

%endfunction

%  where sfun is the name of the S-function.
% The S-Function Builder inserts a call to this wrapper function in the mdlDerivatives callback method that it generates for the S-function. 
% The Simulink engine calls the mdlDerivatives method at the end of each time step to obtain the derivatives of the continuous states (see Simulink Engine Interaction with C S-Functions).
% The Simulink solver numerically integrates the derivatives to determine the continuous states at the next time step. At the next time step, the engine passes the updated states back to the mdlOutputs method (see Outputs Pane).