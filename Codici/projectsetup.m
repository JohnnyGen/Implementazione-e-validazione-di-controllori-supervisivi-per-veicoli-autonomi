%% General Model parameters
Ts = 0.1;               % Simulation sample time                (s)

%% Tracking and Sensor Fusion Parameters                        Units
clusterSize = 4;        % Distance for clustering               (m)
assigThresh = 50;       % Tracker assignment threshold          (N/A)
M           = 2;        % Tracker M value for M-out-of-N logic  (N/A)
N           = 3;        % Tracker M value for M-out-of-N logic  (N/A)
numCoasts   = 5;        % Number of track coasting steps        (N/A)
numTracks   = 20;       % Maximum number of tracks              (N/A)
numSensors  = 2;        % Maximum number of sensors             (N/A)

% Position and velocity selectors from track state
% The filter initialization function used in this example is initcvekf that 
% defines a state that is: [x;vx;y;vy;z;vz]. 
posSelector = [1,0,0,0,0,0; 0,0,1,0,0,0]; % Position selector   (N/A)
velSelector = [0,1,0,0,0,0; 0,0,0,1,0,0]; % Velocity selector   (N/A)

%% Ego Car 
% Dynamics modeling parameters
m       = 1521;     % Total mass of vehicle                          (kg)
Iz      = 2875;     % Yaw moment of inertia of vehicle               (m*N*s^2)
lf      = 1.2;      % Longitudinal distance from c.g. to front tires (m)
lr      = 1.6;      % Longitudinal distance from c.g. to rear tires  (m)
Cf      = 19000;    % Cornering stiffness of front tires             (N/rad)
Cr      = 33000;    % Cornering stiffness of rear tires              (N/rad)
tau     = 0.5;      % Longitudinal time constant                     (N/A)
R = 0.5;
n = 0.93;
T_gap=1; %Headway time

% Initial condition for the ego car
v0_ego = 15;         %  Initial speed of the ego car           (m/s) 
x0_ego = 0;            % Initial x position of ego car          (m)
y0_ego = 0;       % Initial y position of ego car          (m)

%% AEB controller parameters
AEB.PB1_decel = 6;            % 1st stage Partial Braking deceleration (m/s^2)
AEB.PB2_decel = 8;            % 2nd stage Partial Braking deceleration (m/s^2)
AEB.FB_decel  = 9.8;            % Full Braking deceleration              (m/s^2)

%Assign AEB struct in base workspace
assignin('base','AEB',AEB);
%% Bus Creation
% Create the bus of actors from the scenario reader
modelName = 'FINAL_MODEL';
wasModelLoaded = bdIsLoaded(modelName);
if ~wasModelLoaded
    load_system(modelName)
end
blk=find_system(modelName,'System','driving.scenario.internal.ScenarioReader');
s = get_param(blk{1},'PortHandles');
get(s.Outport(1),'SignalHierarchy');

% Create the bus of tracks 
refModel = 'FINAL_MODEL';
wasReModelLoaded = bdIsLoaded(refModel);
if ~wasReModelLoaded
    load_system(refModel)
    blk=find_system(refModel,'System','multiObjectTracker');
    multiObjectTracker.createBus(blk{1});
    close_system(refModel)
else
    blk=find_system(refModel,'System','multiObjectTracker');
    multiObjectTracker.createBus(blk{1});
end

if ~wasModelLoaded
    close_system(modelName)
end

%% Code generation
% Uncomment this if you would like to generate code.
 %slbuild(refModel);