x = traj_time(1):0.01:traj_time(2);
y = subs(p1x, t, x);
plot(x, y)
hold on;
x = traj_time(2):0.01:traj_time(3);
y = subs(p2x, t, x);
plot(x, y)
x = traj_time(3):0.01:traj_time(4);
y = subs(p3x, t, x);
plot(x, y)
x = traj_time(4):0.01:traj_time(5);
y = subs(p4x, t, x);
plot(x, y)

x = traj_time(1):0.01:traj_time(2);
y = subs(diff(p1x, t), t, x);
plot(x, y,'r')
hold on;
x = traj_time(2):0.01:traj_time(3);
y = subs(diff(p2x,t), t, x);
plot(x, y,'r')
x = traj_time(3):0.01:traj_time(4);
y = subs(diff(p3x,t), t, x);
plot(x, y,'r')
x = traj_time(4):0.01:traj_time(5);
y = subs(diff(p4x,t), t, x);
plot(x, y,'r')

x = traj_time(1):0.01:traj_time(2);
y = subs(diff(p1x, t,2), t, x);
plot(x, y,'g')
hold on;
x = traj_time(2):0.01:traj_time(3);
y = subs(diff(p2x,t,2), t, x);
plot(x, y,'g')
x = traj_time(3):0.01:traj_time(4);
y = subs(diff(p3x,t,2), t, x);
plot(x, y,'g')
x = traj_time(4):0.01:traj_time(5);
y = subs(diff(p4x,t,2), t, x);
plot(x, y,'g')
