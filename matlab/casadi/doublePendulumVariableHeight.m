close all 
clear all

jump = false;

%steps poses
xL1 = [0.00; 0.03; 0.0];
xR1 = [0.00; -0.03; 0.0];
xL2 = [0.05; 0.03; 0.0];
xR2 = [0.10; -0.03; 0.0];
xL3 = [0.15; 0.03; 0.0];
xR3 = [0.20; -0.03; 0.0];
xL4 = [0.25; 0.03; 0.0];
xR4 = [0.30; -0.03; 0.0];
xL5 = [0.35; 0.03; 0.0];
xR5 = [0.35; -0.03; 0.0];

% xL1 = [0.0; 0.03; 0.0];
% xR1 = [0.0; -0.03; 0.0];
% xL2 = [0.05; 0.03; 0.0];
% xR2 = [0.05; -0.03; 0.0];

initialState.position = [0.0; 0.0; 0.15];
initialState.velocity = [0.0; 0.0; 0.0];

references.state.position = [0.4; 0.0; 0.15];
references.state.velocity = [0.0; 0.0; 0.0];
references.state.anticipation = 0.1;

%use it to define the foot print
constraints.cop = [-0.035, 0.035;  % x
                   -0.015, 0.015]; % y
constraints.legLength = 0.15;
constraints.staticFriction = 0.5;
constraints.torsionalFriction = 0.01;

references.control = [0.0;
                      0.0;
                      9.81/(2*(references.state.position(3) - xL5(3)));
                      0.0;
                      0.0;
                      9.81/(2*(references.state.position(3) - xR5(3)))];

references.legLength = 0.15;
                   

%references.timings = [0.8; 1; 0.2; 1; 0.2; 1; 0.2; 1; 0.2; 1; 0.2; 1; 0.2; 1; 0.2; 1; 0.8]; 
references.timings = 0.6*ones(17,1);


activeFeet = [true, true;
              false, true;
              true, true;
              true, false;
              true, true;
              false, true;
              true, true;
              true, false;
              true, true;
              false, true;
              true, true;
              true, false;
              true, true;
              false, true;
              true, true;
              true, false;
              true, true;];

% activeFeet = [true, true;
%               false, true;
%               true, true;
%               true, false;
%               true, true;];

if (jump)
    activeFeet(3,:) = [false, false];              
end
    
feetLocations = {xL1, xR1;     %true, true
                 xL2, xR1;     %false, true
                 xL2, xR1;     %true, true
                 xL2, xR1;     %true, false
                 xL2, xR2;     %true, true
                 xL3, xR2;     %false, true
                 xL3, xR2;     %true, true
                 xL3, xR2;     %true, false
                 xL3, xR3;     %true, true
                 xL4, xR3;     %false, true
                 xL4, xR3;     %true, true
                 xL4, xR3;     %true, false
                 xL4, xR4;     %true, true
                 xL5, xR4;     %false, true
                 xL5, xR4;     %true, true
                 xL5, xR4;     %true, false
                 xL5, xR5;};   %true, true

% feetLocations = {xL1, xR1;     %true, true
%                  xL2, xR1;     %false, true
%                  xL2, xR1;     %true, true
%                  xL2, xR1;     %true, false
%                  xL2, xR2;};   %true, true

             
phase_length = 20;

numberOfPhases = size(activeFeet, 1);
N = phase_length * numberOfPhases;

constraints.minimumTimings = 0.5 * ones(numberOfPhases,1);
constraints.maximumTimings = 2.0 * ones(numberOfPhases,1);

weights.time = 1;
weights.finalState = 10;
weights.u = 0.1/N;
weights.cop = 10/N;
weights.copVariation = 0./N;
weights.controlVariation = 40/N;
weights.finalControl = 1;
weights.torques = 1/N;

tic
[xsol, usol, t_sol] = solveVariableHeightDoublePendulum(initialState,...
                                                        references, ...
                                                        constraints, ...
                                                        activeFeet, ...
                                                        feetLocations, ...
                                                        phase_length, ...
                                                        weights);
toc

plotOptiSolutionForDoublePendulum(xsol, usol, t_sol);
plotOptiFeetStepsForDoublePendulum(feetLocations, t_sol, constraints.cop, xsol)
plotOptiDCM(xsol, usol, t_sol, sqrt(9.81/references.legLength))
plotOptiDCMonFeetSteps(feetLocations, t_sol, constraints.cop, xsol, sqrt(9.81/references.legLength))
