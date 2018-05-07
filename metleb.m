z = tf([1 0], [1], -1); 

dt = 1;
constant = 5;
kp = 20;
kd = 70;
ki = 100;
m = 1;
gamma = 0.0299*2;

sys = z^2/(constant*(z^2-2*(z)+1));
pole(sys)
step(sys);

controller = (kp + kd/(m*dt) - kd/(dt*m*z^m) + (dt*ki*z)/(z-1));
CL = feedback(controller*sys,1);

pole(CL)
step(CL)