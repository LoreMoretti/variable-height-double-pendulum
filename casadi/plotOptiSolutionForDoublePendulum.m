function plotOptiSolutionForDoublePendulum(optiSolution, X, U, T, phase_length)
xsol = optiSolution.value(X);
usol = optiSolution.value(U);
t_sol = optiSolution.value(T);

time = linspace(0.0, t_sol(1), phase_length+1);
time = [time, linspace(time(end) + t_sol(2)/phase_length, time(end) + t_sol(2), phase_length)];
time = [time, linspace(time(end) + t_sol(3)/phase_length, time(end) + t_sol(3), phase_length)];
time = [time, linspace(time(end) + t_sol(4)/phase_length, time(end) + t_sol(4), phase_length)];
time = [time, linspace(time(end) + t_sol(5)/phase_length, time(end) + t_sol(5), phase_length)];

figure
ax = axes;
plot(time, xsol(1:3,:));
t = 0;
for i = 1 : length(t_sol)
    t = t + t_sol(i);
    line([t t],get(ax,'YLim'),'Color','k', 'LineStyle','--')
end
title("CoM Position")
legend(["x", "y", "z"]);
ylabel("[m]");
xlabel("t [s]");

figure
ax = axes;
plot(time, xsol(4:6,:));
t = 0;
for i = 1 : length(t_sol)
    t = t + t_sol(i);
    line([t t],get(ax,'YLim'),'Color','k', 'LineStyle','--')
end
title("CoM Velocity")
legend(["x", "y", "z"]);
ylabel("[m/s]");
xlabel("t [s]");

figure
ax = axes;
t = 0;
plot(time(2:end), usol(3,:), time(2:end), usol(6,:));
for i = 1 : length(t_sol)
    t = t + t_sol(i);
    line([t t],get(ax,'YLim'),'Color','k', 'LineStyle','--')
end
legend(["uLeft", "uRight"]);
title("u")
ylabel("[1/s^2]");
xlabel("t [s]");

figure
ax = axes;
t = 0;
plot(time(2:end), usol(1:2,:));
for i = 1 : length(t_sol)
    t = t + t_sol(i);
    line([t t],get(ax,'YLim'),'Color','k', 'LineStyle','--')
end
title("CoP Left")
legend(["x", "y"]);
ylabel("[m]");
xlabel("t [s]");

figure
ax = axes;
t = 0;
plot(time(2:end), usol(4:5,:));
for i = 1 : length(t_sol)
    t = t + t_sol(i);
    line([t t],get(ax,'YLim'),'Color','k', 'LineStyle','--')
end
title("CoP Right")
legend(["x", "y"]);
ylabel("[m]");
xlabel("t [s]");
end