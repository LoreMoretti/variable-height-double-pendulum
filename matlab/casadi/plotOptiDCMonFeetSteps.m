function plotOptiDCMonFeetSteps(feetLocations, t_sol, footprint, xsol, omega0)

    x = footprint(1,:);
    y = footprint(2,:);
    vertices = [x(1), x(2), x(2), x(1);
                y(1), y(1), y(2), y(2);];

    figure
    ax = axes;
    axis equal
    color = [0.9290 0.6940 0.1250];
    t = 0;
    for step=1:numel(t_sol)
        leftfoot_xy = feetLocations{step,1}(1:2);
        patch(vertices(1,:)+leftfoot_xy(1), ...
              vertices(2,:)+leftfoot_xy(2), ...
              color,'facealpha',0.3);

        rightfoot_xy = feetLocations{step,2}(1:2);
        patch(vertices(1,:)+rightfoot_xy(1), ...
              vertices(2,:)+rightfoot_xy(2), ...
              color,'facealpha',0.3);
    end
    ylim([-0.1, 0.1])
    xlim([0, 0.4])
    title("feet steps and DCM")
    ylabel("y [m]");
    xlabel("x [m]");
    hold on

    DCM = xsol(1:3,:) + 1/omega0 * xsol(4:6,:);
    plot(DCM(1,:),DCM(2,:), 'k')


end