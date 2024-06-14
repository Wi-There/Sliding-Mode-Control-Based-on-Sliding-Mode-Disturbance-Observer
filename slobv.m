function [sys,x0,str,ts,simStateCompliance] = slobv(t,x,u,flag,pa)
switch flag
  case 0
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;
  case 1
    sys=mdlDerivatives(t,x,u,pa);
  case 2
    sys=mdlUpdate(t,x,u);
  case 3,
    sys=mdlOutputs(t,x,u,pa);
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);
  case 9,
    sys=mdlTerminate(t,x,u);
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 2;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 2;
sizes.NumInputs      = 2;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;  
sys = simsizes(sizes);
x0  = [1 1 ];
str = [];
ts  = [0 0];
simStateCompliance = 'UnknownSimState';
function sys=mdlDerivatives(t,x,u,pa)
x1=x(1);%hdth
x2=x(2);%hTL
J=pa.J;
B=pa.B;
i=pa.i;
cw=pa.cw;
l=pa.l;
delta=pa.delta;
Epw=pa.Epw;
dth=u(2);%dth
Te=u(1);%Te
ew=dth-x1;%dth-hdth
sw=cw*ew;%+dew;%+cw*integral(fun,0,t);
dsw=-Epw*abs(sw)/(abs(sw)+delta)*sign(sw)/cw;
g=(-B/(J))*ew-dsw;
dx1=(Te-x2-B*x1)/(J*i)+g;
dx2=l*g;
sys = [dx1;dx2];
function sys=mdlUpdate(t,x,u)
sys = [];
function sys=mdlOutputs(t,x,u,pa)
sys =x;
function sys=mdlGetTimeOfNextVarHit(t,x,u)
sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;
function sys=mdlTerminate(t,x,u)
sys = [];
