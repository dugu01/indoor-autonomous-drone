%% MAKE_S2_4_SHADOW_VIDEO
% Creates a poster/demo video from the actual S2.4 shadow replay.
% The video shows the evolving probabilistic map, persistent frontiers,
% candidate viewpoints and selected shadow recommendations.
%
% IMPORTANT: The recommendation is logged only; it is not sent to the drone.

clear; clc; close all force;

%% USER PATH
s24root = fileparts(mfilename('fullpath'));

% Leave empty to use the newest repeat_1 report automatically.
reportFile = '';

% Slower poster-loop version: 85 snapshots at 3 fps plus short opening and
% closing holds gives a video of about 32 seconds.
videoFrameRate = 3;
firstHoldSeconds = 2;
finalHoldSeconds = 2;

%% LOCATE REPORTS
if isempty(reportFile)
    reports = dir(fullfile(s24root, 'results', '**', 'repeat_1', ...
        'S2_4_AD_shadow_report.mat'));
    assert(~isempty(reports), 'No repeat_1 S2.4 shadow report was found.');
    [~, newest] = max([reports.datenum]);
    reportFile = fullfile(reports(newest).folder, reports(newest).name);
end
assert(isfile(reportFile), 'Report not found:\n%s', reportFile);

R = load(reportFile, 'report', 'shadowLog');
trialMat = R.report.trialMat;
if ~isfile(trialMat)
    [f,p] = uigetfile('*.mat', 'Select the S2.3 trial MAT used for this replay');
    assert(~isequal(f,0), 'A trial MAT file is required.');
    trialMat = fullfile(p,f);
end
T = load(trialMat, 'maps', 'log', 'scenario');

assert(numel(R.shadowLog) == numel(T.log.mapSnapshots), ...
    'Shadow-log and map-snapshot counts do not match.');

validationDir = fileparts(fileparts(reportFile));
outDir = fullfile(s24root, 'poster_figures');
if ~isfolder(outDir), mkdir(outDir); end
videoFile = fullfile(outDir, 'S2_4_shadow_replay_demo.mp4');

xs = double(T.maps.finalGrid.xs(:));
ys = double(T.maps.finalGrid.ys(:));
goalXY = double(T.scenario.goal(1:2));

% Precompute the recorded drone path at the stored map-snapshot times.
n = numel(R.shadowLog);
droneHistory = zeros(n,2);
for kk = 1:n
    snapTime = double(T.log.mapSnapshots{kk}.time);
    [~, ii] = min(abs(double(T.log.t(:)) - snapTime));
    if size(T.log.estP,2) == 3
        droneHistory(kk,:) = double(T.log.estP(ii,1:2));
    else
        droneHistory(kk,:) = double(T.log.estP(1:2,ii)).';
    end
end
startXY = droneHistory(1,:);
scenarioLabel = prettyScenarioName(string(R.report.scenario));

%% VIDEO SETUP
writer = VideoWriter(videoFile, 'MPEG-4');
writer.FrameRate = videoFrameRate;
writer.Quality = 95;
open(writer);
cleanupWriter = onCleanup(@() close(writer)); %#ok<NASGU>

fig = figure('Color','w', 'Position',[80 80 1700 920], ...
    'Name','S2.4 Shadow Replay Video');
ax = axes(fig,'Position',[0.06 0.09 0.68 0.82]);
for k = 1:n
    delete(findall(fig,'Tag','S24InfoPanel'));
    cla(ax);
    hold(ax,'on');

    E = R.shadowLog{k};
    snap = T.log.mapSnapshots{k};

    known = logical(snap.knownFree);
    unknown = logical(snap.unknown);
    staticOcc = logical(snap.staticOccupied);
    dynamicOcc = logical(snap.dynamicOccupied);

    classMap = zeros(size(known));
    classMap(known) = 1;
    classMap(unknown) = 2;
    classMap(staticOcc) = 3;
    classMap(dynamicOcc) = 4;

    imagesc(ax, xs, ys, classMap);
    axis(ax,'xy','equal','tight');
    colormap(ax, [1 1 1; 0.84 0.91 0.97; 0.72 0.75 0.80; ...
        0.18 0.20 0.24; 0.95 0.63 0.15]);
    caxis(ax,[0 4]);

    % Frontier cells.
    for i = 1:numel(E.frontiers)
        cells = double(E.frontiers(i).cells);
        if isempty(cells), continue; end
        valid = all(isfinite(cells(:,1:2)),2) & ...
            all(cells(:,1:2) == round(cells(:,1:2)),2) & ...
            cells(:,1) >= 1 & cells(:,1) <= numel(ys) & ...
            cells(:,2) >= 1 & cells(:,2) <= numel(xs);
        cells = cells(valid,:);
        if ~isempty(cells)
            plot(ax, xs(cells(:,2)), ys(cells(:,1)), '.', ...
                'MarkerSize',10,'Color',[0.05 0.45 0.85]);
        end
    end

    % Candidate viewpoints. Rejected proposals outside the map are omitted.
    if ~isempty(E.candidates)
        accepted = logical([E.candidates.accepted]).';
        cells = double(vertcat(E.candidates.cell));
        valid = size(cells,2) >= 2 & ...
            all(isfinite(cells(:,1:2)),2) & ...
            all(cells(:,1:2) == round(cells(:,1:2)),2) & ...
            cells(:,1) >= 1 & cells(:,1) <= numel(ys) & ...
            cells(:,2) >= 1 & cells(:,2) <= numel(xs);

        rejectedValid = ~accepted & valid;
        acceptedValid = accepted & valid;

        if any(rejectedValid)
            plot(ax, xs(cells(rejectedValid,2)), ys(cells(rejectedValid,1)), ...
                'x','MarkerSize',4,'LineWidth',0.8,'Color',[0.70 0.20 0.17]);
        end
        if any(acceptedValid)
            plot(ax, xs(cells(acceptedValid,2)), ys(cells(acceptedValid,1)), ...
                'o','MarkerSize',4,'LineWidth',0.8,'Color',[0.05 0.55 0.28]);
        end
    end

    % Selected recommendation.
    selectedId = double(E.viewpointDiagnostics.selectedCandidateId);
    if selectedId > 0 && ~isempty(E.candidates)
        j = find(double([E.candidates.candidateId]) == selectedId, 1);
        if ~isempty(j)
            q = E.candidates(j);
            if ~isempty(q.path)
                pathCells = double(q.path(:,1:2));
                validPath = all(isfinite(pathCells),2) & ...
                    all(pathCells == round(pathCells),2) & ...
                    pathCells(:,1) >= 1 & pathCells(:,1) <= numel(ys) & ...
                    pathCells(:,2) >= 1 & pathCells(:,2) <= numel(xs);
                pathCells = pathCells(validPath,:);
                if ~isempty(pathCells)
                    plot(ax, xs(pathCells(:,2)), ys(pathCells(:,1)), '-', ...
                        'LineWidth',3,'Color',[0.55 0.10 0.72]);
                end
            end

            selectedCell = double(q.cell(1:2));
            validSelected = all(isfinite(selectedCell)) && ...
                all(selectedCell == round(selectedCell)) && ...
                selectedCell(1) >= 1 && selectedCell(1) <= numel(ys) && ...
                selectedCell(2) >= 1 && selectedCell(2) <= numel(xs);
            if validSelected
                plot(ax, xs(selectedCell(2)), ys(selectedCell(1)), 'p', ...
                    'MarkerSize',17,'MarkerFaceColor',[0.55 0.10 0.72], ...
                    'MarkerEdgeColor','w','LineWidth',1.3);
            end
        end
    end

    % Recorded drone path up to the current map state. The blue tail is
    % the path actually flown in the recorded parent mission. The purple
    % route, when present, is a shadow recommendation and was not executed.
    droneXY = droneHistory(k,:);
    plot(ax, droneHistory(1:k,1), droneHistory(1:k,2), '-', ...
        'LineWidth',2.3,'Color',[0.05 0.30 0.55]);
    plot(ax, startXY(1), startXY(2), 's', 'MarkerSize',10, ...
        'MarkerFaceColor',[0.30 0.65 0.90], ...
        'MarkerEdgeColor','w','LineWidth',1.3);
    plot(ax, droneXY(1), droneXY(2), 'o', 'MarkerSize',11, ...
        'MarkerFaceColor',[0.05 0.30 0.55], ...
        'MarkerEdgeColor','w','LineWidth',1.4);
    plot(ax, goalXY(1), goalXY(2), 'p', 'MarkerSize',16, ...
        'MarkerFaceColor',[0.85 0.25 0.05], ...
        'MarkerEdgeColor','w','LineWidth',1.4);
    text(ax,startXY(1)+0.07,startXY(2)+0.07,'START', ...
        'FontSize',12,'FontWeight','bold','Color',[0.05 0.30 0.55], ...
        'BackgroundColor','w','Margin',1,'Interpreter','none');
    text(ax,goalXY(1)+0.07,goalXY(2)+0.07,'TARGET', ...
        'FontSize',12,'FontWeight','bold','Color',[0.75 0.22 0.04], ...
        'BackgroundColor','w','Margin',1,'Interpreter','none');
    text(ax,droneXY(1)+0.06,droneXY(2)-0.10,'DRONE', ...
        'FontSize',11,'FontWeight','bold','Color',[0.05 0.30 0.55], ...
        'BackgroundColor','w','Margin',1,'Interpreter','none');

    candidateCount = double(E.viewpointDiagnostics.candidateCount);
    acceptedCount = double(E.viewpointDiagnostics.acceptedCount);
    clusterCount = double(E.frontierDiagnostics.clusterCount);

    title(ax, sprintf('Recorded map and next-view decision at t = %.1f s',double(E.time)), ...
        'FontSize',19,'FontWeight','bold','Interpreter','none');
    xlabel(ax,'x [m]'); ylabel(ax,'y [m]');
    set(ax,'FontSize',15,'LineWidth',1.1,'Box','on');

    if selectedId > 0
        recommendationText = 'Next-view recommendation: available';
    else
        recommendationText = 'Next-view recommendation: none at this map state';
    end
    panelText = sprintf([ ...
        'RECORDED CASE\n%s\n\n' ...
        'HOW TO READ THIS FRAME\n' ...
        'Light blue: known free\n' ...
        'Grey: unknown\n' ...
        'Dark: obstacle / inflation\n' ...
        'Orange: dynamic occupancy\n\n' ...
        'Blue line: recorded drone path\n' ...
        'Blue dots: frontiers\n' ...
        'Red x: rejected proposals\n' ...
        'Green o: proposals passing gates\n' ...
        'Purple line/star: recommended route/view\n' ...
        '(purple route was not flown)\n\n' ...
        'Frontier clusters: %d\n' ...
        'Proposals checked: %d\n' ...
        'Passed gates: %d\n' ...
        '%s\n\n' ...
        'SHADOW MODE\nRecommendation logged, not sent to controller'], ...
        scenarioLabel,clusterCount,candidateCount,acceptedCount,recommendationText);
    annotation(fig,'textbox',[0.765 0.11 0.22 0.78], 'Tag','S24InfoPanel', ...
        'String',panelText,'Interpreter','none','FontSize',12.5, ...
        'FitBoxToText','off','BackgroundColor','white', ...
        'EdgeColor',[0.55 0.65 0.75],'LineWidth',1.2,'Margin',8, ...
        'VerticalAlignment','top');

    drawnow;
    frame = getframe(fig);
    repeatCount = 1;
    if k == 1
        repeatCount = max(1,round(firstHoldSeconds*videoFrameRate));
    elseif k == n
        repeatCount = max(1,round(finalHoldSeconds*videoFrameRate));
    end
    for rr = 1:repeatCount
        writeVideo(writer, frame);
    end
end

close(writer);
clear cleanupWriter;
close(fig);

fprintf('\nVideo created successfully:\n%s\n', videoFile);
totalWrittenFrames = n - 2 + ...
    max(1,round(firstHoldSeconds*videoFrameRate)) + ...
    max(1,round(finalHoldSeconds*videoFrameRate));
fprintf('Source snapshots: %d | Written frames: %d | Frame rate: %.1f fps | Approx. duration: %.1f s\n', ...
    n, totalWrittenFrames, videoFrameRate, totalWrittenFrames/videoFrameRate);


function out = prettyScenarioName(raw)
raw = upper(string(raw));
switch raw
    case "DYNAMIC_TO_STATIC_MAPPING"
        out = "Dynamic obstacle becomes static";
    case "UNKNOWN_ROOM_NOMINAL"
        out = "Unknown room - nominal mission";
    case "LATE_CORRIDOR_BLOCKAGE_REPLAN"
        out = "Corridor becomes blocked";
    case "OCCLUDED_OBSTACLE"
        out = "Occluded obstacle";
    otherwise
        out = strrep(lower(raw),"_"," ");
end
end
