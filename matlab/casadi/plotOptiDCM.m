function plotOptiDCM(xsol, usol, t_sol, omega0)

time = 0.0;
numberOfPhases = length(t_sol);
phase_length = size(usol,2)/numberOfPhases;
for i = 1 : numberOfPhases
    time = [time, linspace(time(end) + t_sol(i)/phase_length, time(end) + t_sol(i), phase_length)];
end

DCM = xsol(1:3,:) + 1/omega0 * xsol(4:6,:);

figure
ax = axes;
plot(time, DCM);
t = 0;
for i = 1 : length(t_sol)
    t = t + t_sol(i);
    line([t t],get(ax,'YLim'),'Color','k', 'LineStyle','--')
end
title("DCM Position")
legend(["x", "y", "z"]);
ylabel("[m]");
xlabel("t [s]");


end