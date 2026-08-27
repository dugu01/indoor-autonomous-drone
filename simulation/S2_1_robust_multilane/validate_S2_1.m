function report=validate_S2_1(runSimulation)
if nargin<1,runSimulation=false;end
files={'run_S2_lidar_slam.m','plot_S2_dashboard.m','animate_S2_flight.m'};report=struct();
for i=1:numel(files),report.files.(matlab.lang.makeValidName(files{i}))=exist(fullfile(fileparts(mfilename('fullpath')),files{i}),'file')==2;end
fprintf('Resolved functions:\n');which run_S2_lidar_slam -all;which plot_S2_dashboard -all;which animate_S2_flight -all;
try,report.checkcode=checkcode(fullfile(fileparts(mfilename('fullpath')),'run_S2_lidar_slam.m'),'-id');catch ME,report.checkcode={ME.message};end
if runSimulation
    r=run_S2_lidar_slam(0,true,false,false);report.nominalPass=r.nominal.pass;report.stressPass=r.stress.pass;
    assert(r.nominal.pass,'Nominal S2.1 trial failed.');assert(r.stress.pass,'Stress S2.1 trial failed.');
else,report.nominalPass=[];report.stressPass=[];end
end
