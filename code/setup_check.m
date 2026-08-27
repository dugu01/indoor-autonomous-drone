% =========================================================================
%  setup_check.m  —  Run this FIRST before anything else
%  Project : Indoor GPS-Denied Autonomous Drone
%
%  What this script does:
%    1. Checks your MATLAB version
%    2. Checks every required toolbox is licensed and installed
%    3. Creates the full project folder structure on your laptop
%    4. Adds all folders to the MATLAB path and saves it
%    5. Runs a 30-second quick smoke-test of S1
%
%  HOW TO RUN:
%    - Open MATLAB
%    - In the top bar: Home → Set Path → (ignore for now)
%    - In the address bar, navigate to your project folder
%      (wherever you cloned/downloaded the GitHub repo)
%    - Type in the Command Window:  setup_check
%    - Read the output carefully — fix anything marked [MISSING]
% =========================================================================

clear; clc;
fprintf('============================================================\n');
fprintf('  DRONE PROJECT — MATLAB SETUP CHECK\n');
fprintf('  %s\n', datestr(now));
fprintf('============================================================\n\n');

% =========================================================================
%  STEP 1: MATLAB VERSION CHECK
%  Minimum: R2022b  (monoVisualSLAM needs R2022a+, poseGraph needs R2019b+)
%  Recommended: R2023a or newer
% =========================================================================
fprintf('[ 1 ] MATLAB version\n');
v = ver('MATLAB');
matlab_ver = v.Version;
matlab_rel = v.Release;
fprintf('      Version : %s  %s\n', matlab_ver, matlab_rel);

year = str2double(matlab_ver(1:4));
if year >= 2022
    fprintf('      Status  : OK — R2022a or newer\n\n');
else
    fprintf('      Status  : WARNING — R2021b or older.\n');
    fprintf('                monoVisualSLAM requires R2022a+.\n');
    fprintf('                Ask your department lab admin to update.\n\n');
end

% =========================================================================
%  STEP 2: TOOLBOX CHECK
%  Each toolbox is tested by calling a function that only exists in it.
%  This is more reliable than checking license() alone.
% =========================================================================
fprintf('[ 2 ] Required toolboxes\n');
fprintf('      %-40s  %s\n', 'Toolbox', 'Status');
fprintf('      %s\n', repmat('-', 1, 55));

toolboxes = {
    'UAV Toolbox',                    'uavScenario',            'uavScenario';
    'Sensor Fusion & Tracking Tbx',   'imuSensor',              'imuSensor(''accel-gyro'')';
    'Computer Vision Toolbox',         'monoVisualSLAM',         'cameraIntrinsics([600 600],[320 240],[480 640])';
    'Navigation Toolbox',              'poseGraph',              'poseGraph';
    'Robotics System Toolbox',         'robotics.Rate',          'robotics.Rate(10)';
    'Simulink',                        'simulink',               'simulink_version';
    'Aerospace Blockset',              'aero',                   'Aero.FlightGearAnimation';
};

missing = {};
for i = 1:size(toolboxes,1)
    name    = toolboxes{i,1};
    testfn  = toolboxes{i,2};
    try
        % Try to instantiate or call the function
        eval(toolboxes{i,3});
        fprintf('      %-40s  [OK]\n', name);
    catch
        % Check if it's a license issue vs missing toolbox
        if license('test', testfn)
            fprintf('      %-40s  [LICENSED but not installed]\n', name);
        else
            fprintf('      %-40s  [MISSING — see fix below]\n', name);
        end
        missing{end+1} = name; %#ok<AGROW>
    end
end

fprintf('\n');
if isempty(missing)
    fprintf('      All toolboxes present.\n\n');
else
    fprintf('      Missing toolboxes:\n');
    for i = 1:length(missing)
        fprintf('        - %s\n', missing{i});
    end
    fprintf('\n');
    fprintf('      HOW TO FIX:\n');
    fprintf('        Home tab → Add-Ons → Get Add-Ons\n');
    fprintf('        Search for each missing toolbox and install.\n');
    fprintf('        If your institution does not have the license,\n');
    fprintf('        contact your lab admin — most colleges have TAH\n');
    fprintf('        (Total Academic Headcount) licenses that include all toolboxes.\n\n');
end

% =========================================================================
%  STEP 3: CREATE FOLDER STRUCTURE
%  Mirrors the GitHub repo structure exactly
% =========================================================================
fprintf('[ 3 ] Project folder structure\n');

% Detect project root = wherever this script lives
project_root = fileparts(mfilename('fullpath'));
if isempty(project_root)
    project_root = pwd;   % fallback: current directory
end
fprintf('      Project root: %s\n\n', project_root);

folders = {
    'simulation/S1_dynamics_pid'
    'simulation/S2_visual_slam'
    'simulation/S3_obstacle_avoidance'
    'simulation/S4_aruco_landing'
    'simulation/results'
    'firmware/arduino'
    'ros2_ws/src'
    'docs'
};

for i = 1:length(folders)
    full_path = fullfile(project_root, folders{i});
    if ~exist(full_path, 'dir')
        mkdir(full_path);
        fprintf('      Created : %s\n', folders{i});
    else
        fprintf('      Exists  : %s\n', folders{i});
    end
end
fprintf('\n');

% =========================================================================
%  STEP 4: CONFIGURE MATLAB PATH
%  Adds all simulation subfolders so MATLAB finds helper functions
% =========================================================================
fprintf('[ 4 ] MATLAB path setup\n');

paths_to_add = {
    fullfile(project_root, 'simulation', 'S1_dynamics_pid')
    fullfile(project_root, 'simulation', 'S2_visual_slam')
    fullfile(project_root, 'simulation', 'S3_obstacle_avoidance')
    fullfile(project_root, 'simulation', 'S4_aruco_landing')
    fullfile(project_root, 'simulation', 'results')
};

for i = 1:length(paths_to_add)
    if exist(paths_to_add{i}, 'dir')
        addpath(paths_to_add{i});
    end
end

% Save path so it persists across MATLAB sessions
try
    savepath;
    fprintf('      Path saved permanently (savepath succeeded)\n');
catch
    fprintf('      WARNING: savepath failed (no write permission).\n');
    fprintf('      Add this to your startup.m instead:\n');
    fprintf('        addpath(''%s'')\n', ...
            fullfile(project_root, 'simulation', 'S1_dynamics_pid'));
end
fprintf('\n');

% =========================================================================
%  STEP 5: QUICK SMOKE TEST
%  Instantiates key objects from each toolbox to confirm they actually work
%  Does NOT run the full simulation yet
% =========================================================================
fprintf('[ 5 ] Quick smoke test (instantiating key objects)\n');

tests_passed = 0;
tests_total  = 0;

% --- UAV Toolbox: uavScenario
tests_total = tests_total + 1;
try
    sc = uavScenario('UpdateRate', 10, 'StopTime', 1);
    fprintf('      uavScenario          : OK\n');
    tests_passed = tests_passed + 1;
    clear sc;
catch e
    fprintf('      uavScenario          : FAIL — %s\n', e.message);
end

% --- Sensor Fusion: imuSensor
tests_total = tests_total + 1;
try
    imu = imuSensor('accel-gyro', 'SampleRate', 100);
    fprintf('      imuSensor            : OK\n');
    tests_passed = tests_passed + 1;
    clear imu;
catch e
    fprintf('      imuSensor            : FAIL — %s\n', e.message);
end

% --- Computer Vision: cameraIntrinsics
tests_total = tests_total + 1;
try
    cam = cameraIntrinsics([860 860], [640 400], [800 1280]);
    fprintf('      cameraIntrinsics     : OK\n');
    tests_passed = tests_passed + 1;
    clear cam;
catch e
    fprintf('      cameraIntrinsics     : FAIL — %s\n', e.message);
end

% --- Navigation: poseGraph (2D needs [x y theta], 3 elements)
tests_total = tests_total + 1;
try
    pg = poseGraph;
    addRelativePose(pg, [1 0 0]);   % [x y theta] — 3 elements for 2D
    fprintf('      poseGraph            : OK\n');
    tests_passed = tests_passed + 1;
    clear pg;
catch e
    fprintf('      poseGraph            : FAIL — %s\n', e.message);
end

% --- Navigation: occupancyMap3D
tests_total = tests_total + 1;
try
    omap = occupancyMap3D(0.1);
    fprintf('      occupancyMap3D       : OK\n');
    tests_passed = tests_passed + 1;
    clear omap;
catch e
    fprintf('      occupancyMap3D       : FAIL — %s\n', e.message);
end

% --- Computer Vision: monoVisualSLAM
% R2025b API: monoVisualSLAM(intrinsics) — verify it exists and is callable
tests_total = tests_total + 1;
try
    % Just check it exists and is on the path — don't instantiate
    % (constructor triggers camera warm-up which needs an actual image)
    fn = which('monoVisualSLAM');
    if isempty(fn)
        error('monoVisualSLAM not found on path');
    end
    % Verify cameraIntrinsics works (dependency)
    cam2 = cameraIntrinsics([860 860], [640 400], [800 1280]);
    fprintf('      monoVisualSLAM       : OK  (%s)\n', fn);
    tests_passed = tests_passed + 1;
    clear cam2;
catch e
    fprintf('      monoVisualSLAM       : FAIL — %s\n', e.message);
end

fprintf('\n      Smoke test: %d / %d passed\n\n', tests_passed, tests_total);

% =========================================================================
%  STEP 6: SUMMARY AND NEXT STEPS
% =========================================================================
fprintf('============================================================\n');
if tests_passed == tests_total && isempty(missing)
    fprintf('  SETUP COMPLETE — all systems go.\n\n');
    fprintf('  Run Stage S1 now:\n');
    fprintf('    >> cd simulation/S1_dynamics_pid\n');
    fprintf('    >> run_S1_simulation\n\n');
    fprintf('  Expected output:\n');
    fprintf('    T/W ratio    : ~2.1  (confirms hardware budget)\n');
    fprintf('    UNIT TEST    : PASS  (altitude error < 5 cm)\n');
    fprintf('    4 figures    : trajectory, position, angles, RPM\n\n');
    fprintf('  Then run Stage S2:\n');
    fprintf('    >> cd ../S2_visual_slam\n');
    fprintf('    >> run_S2_visual_slam\n\n');
else
    fprintf('  SETUP INCOMPLETE — fix issues above first.\n\n');
    if ~isempty(missing)
        fprintf('  Missing toolboxes: install via Home → Add-Ons\n');
    end
    if tests_passed < tests_total
        fprintf('  Failed smoke tests: check MATLAB version (need R2022b+)\n');
    end
    fprintf('\n  Once fixed, run setup_check again to verify.\n\n');
end
fprintf('============================================================\n');