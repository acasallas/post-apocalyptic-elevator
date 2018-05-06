z = tf([1 0], [1], -1); 

dt = 1;
constant = 5;
kp = 15;
kd = .04;
ki = 0.3;
m = 1;
gamma = 13.23;

sys = z^2/(constant*(z^2-2*(z)+1));
pole(sys)
step(sys);

controller = (kp + kd/(m*dt) - kd/(dt*m*z^m) + (dt*ki*z)/(z-1));
CL = feedback(controller*sys,1);

pole(CL)
step(CL)