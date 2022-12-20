function exportResultsToCsv(xsol, t_sol, activeFeet, feetLocations)

mass = 1.5; %[kg]

time = 0.0;
numberOfPhases = length(t_sol);
phase_length = (size(xsol,2)-1)/numberOfPhases;
for i = 1 : numberOfPhases
    time = [time, linspace(time(end) + t_sol(i)/phase_length, time(end) + t_sol(i), phase_length)];
end

% resample
ts = 0.01; %[s]
t = (time(1):ts:round(time(end),2))';
xCOM = interp1(time, xsol(1,:), t);
yCOM = interp1(time, xsol(2,:), t);
zCOM = interp1(time, xsol(3,:), t);
vxCOM = interp1(time, xsol(4,:), t);
vyCOM = interp1(time, xsol(5,:), t);
vzCOM = interp1(time, xsol(6,:), t);
hxCOM = vxCOM * mass;
hyCOM = vyCOM * mass;
hzCOM = vzCOM * mass;

timings = [0; cumsum(round(t_sol,2))];
contact_flag = double(activeFeet);
contact_flag = [1,1;
    contact_flag];
contact_flag_left = interp1(timings, contact_flag(:,1), t, "next");
contact_flag_right = interp1(timings, contact_flag(:,2), t, "next");

foot_location_left = reshape(cell2mat(feetLocations(:,1)),3,[])';
foot_location_left = [foot_location_left(1,:);
    foot_location_left];
foot_location_right = reshape(cell2mat(feetLocations(:,2)),3,[])';
foot_location_right = [foot_location_right(1,:);
    foot_location_right];
x_foot_location_left = interp1(timings, foot_location_left(:,1), t, "next");
y_foot_location_left = interp1(timings, foot_location_left(:,2), t, "next");
z_foot_location_left = interp1(timings, foot_location_left(:,3), t, "next");
x_foot_location_right = interp1(timings, foot_location_right(:,1), t, "next");
y_foot_location_right = interp1(timings, foot_location_right(:,2), t, "next");
z_foot_location_right = interp1(timings, foot_location_right(:,3), t, "next");

% write header
filename = 'TestTrajectory.csv';
header_string = 'time, contact_status_left, contact_status_right, pc-x_left, pc-y_left, pc-z_left, pc-x_right, pc-y_right, pc-z_right, x-COM, y-COM, z-COM, h_x, h_y, h_z';
fid = fopen(filename,'w');
fprintf(fid,'%s\r\n',header_string);
fclose(fid);

%write data
m = [t, contact_flag_left, contact_flag_right, ...
    x_foot_location_left, y_foot_location_left, z_foot_location_left, ...
    x_foot_location_right, y_foot_location_right, z_foot_location_right, ...
    xCOM, yCOM, zCOM, hxCOM, hyCOM, hzCOM];

dlmwrite(filename, m,'-append','delimiter',',');
end