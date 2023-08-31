clc; clear; close all;


Invp = InvPendOnCart(0.256, 2.153, 0.5, 0.1);

[A ,B] = Invp.Linearization();


Q = diag([1 1 1 1]);R=0.1;

K=lqr(double(A),double(B),Q,R);
wr =    [0.5 0 pi 0]';
x0    = [0.2; 0; 140*(pi/180);0];
tspan = [0 30];

Q=eye(4);
R=0.001;
K=lqr(A,B,Q,R);
            
u = @(x) K*(wr-x);

f = @(t,x)Invp.computeDynamics(x,u(x));

L=0.4;
ttime = tspan(1):0.1:tspan(end);
[T ,X] = ode45(f,ttime,x0);
figure()
for i = 1:12:length(X)
   Invp.motionPlot(X(i,1),X(i,3))
   pause(0.12);
    % movieVector(i) =  getframe(hf);
   clf;
end


plot(T,X(:,1),'LineWidth',2)
hold on
plot(T,X(:,2),'LineWidth',2)
hold on
plot(T,X(:,3),'LineWidth',2)
hold on
plot(T,X(:,4),'LineWidth',2)
legend('x','v','\theta','\omega')

