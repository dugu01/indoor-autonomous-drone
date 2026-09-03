%% MAKE_S2_4_POSTER_FIGURES
% Generates poster-ready figures and contextual metrics from an S2.4
% A-D shadow validation output. Run after validate_S2_4_AD has passed.

clear; clc; close all force;

%% USER PATH
s24root = fileparts(mfilename('fullpath'));

% Leave empty to automatically use the newest repeat_1 report.
reportFile = '';

%% LOCATE THE NEWEST VALIDATION REPORT
if isempty(reportFile)
    reports = dir(fullfile(s24root, 'results', '**', 'repeat_1', ...
        'S2_4_AD_shadow_report.mat'));
    assert(~isempty(reports), 'No repeat_1 S2.4 shadow report was found.');
    [~, newest] = max([reports.datenum]);
    reportFile = fullfile(reports(newest).folder, reports(newest).name);
end
assert(isfile(reportFile), 'Report not found:\n%s', reportFile);

repeat1Dir = fileparts(reportFile);
validationDir = fileparts(repeat1Dir);
repeat2File = fullfile(validationDir, 'repeat_2', 'S2_4_AD_shadow_report.mat');
assert(isfile(repeat2File), 'Repeat-2 report not found:\n%s', repeat2File);

R1 = load(reportFile, 'report', 'shadowLog', 'u', 'c');
R2 = load(repeat2File, 'report', 'shadowLog');

trialMat = R1.report.trialMat;
if ~isfile(trialMat)
    [f,p] = uigetfile('*.mat', 'Select the S2.3 trial MAT used for this replay');
    assert(~isequal(f,0), 'A trial MAT file is required.');
    trialMat = fullfile(p,f);
end
T = load(trialMat, 'maps', 'log', 'scenario', 'cfg', 'summary');

assert(numel(R1.shadowLog) == numel(T.log.mapSnapshots), ...
    'Shadow-log and map-snapshot counts do not match.');
assert(numel(R1.shadowLog) == numel(R2.shadowLog), ...
    'Repeat-1 and repeat-2 snapshot counts do not match.');

outDir = fullfile(s24root, 'poster_figures');
if ~isfolder(outDir), mkdir(outDir); end

%% EXTRACT CONTEXTUAL METRICS
n = numel(R1.shadowLog);
time = zeros(n,1);
frontierCells = zeros(n,1);
clusters = zeros(n,1);
frontierTracksActive = zeros(n,1);
candidateCount = zeros(n,1);
acceptedCount = zeros(n,1);
selectedId = zeros(n,1);
selectedUtility = nan(n,1);
selectedTier = nan(n,1);
selectedTargetRelevance = nan(n,1);

allReasons = strings(0,1);
totalCandidates = 0;
totalAccepted = 0;
selectedRecommendations = 0;

for k = 1:n
    e = R1.shadowLog{k};
    time(k) = double(e.time);
    frontierCells(k) = double(e.frontierDiagnostics.frontierCellCount);
    clusters(k) = double(e.frontierDiagnostics.clusterCount);
    frontierTracksActive(k) = numel(e.frontiers);
    candidateCount(k) = double(e.viewpointDiagnostics.candidateCount);
    acceptedCount(k) = double(e.viewpointDiagnostics.acceptedCount);
    selectedId(k) = double(e.viewpointDiagnostics.selectedCandidateId);

    totalCandidates = totalCandidates + numel(e.candidates);
    if ~isempty(e.candidates)
        totalAccepted = totalAccepted + nnz([e.candidates.accepted]);
        for j = 1:numel(e.candidates)
            rr = string(e.candidates(j).rejectionReasons);
            allReasons = [allReasons; rr(:)]; %#ok<AGROW>
        end
    end

    if selectedId(k) > 0
        j = find(double([e.candidates.candidateId]) == selectedId(k), 1);
        if ~isempty(j)
            selectedRecommendations = selectedRecommendations + 1;
            selectedUtility(k) = double(e.candidates(j).utility);
            selectedTier(k) = double(e.candidates(j).tier);
            selectedTargetRelevance(k) = double(e.candidates(j).targetRelevance);
        end
    end
end

% Repeat determinism using interpretable arrays, in addition to the saved digest.
accepted2 = cellfun(@(e) double(e.viewpointDiagnostics.acceptedCount), R2.shadowLog(:));
frontier2 = cellfun(@(e) double(e.frontierDiagnostics.frontierCellCount), R2.shadowLog(:));
selected2 = cellfun(@(e) double(e.viewpointDiagnostics.selectedCandidateId), R2.shadowLog(:));
interpretableReplayExact = isequal(acceptedCount, accepted2) && ...
    isequal(frontierCells, frontier2) && isequal(selectedId, selected2);

fprintf('\n============================================================\n');
fprintf('POSTER METRICS FROM ACTUAL S2.4 SHADOW OUTPUT\n');
fprintf('Scenario                         : %s\n', string(R1.report.scenario));
recordCount = numel(T.maps.perceptionReplay);
sourcePackets = double(R1.report.sourcePackets(:)).';
fprintf('Recorded perception updates      : %d\n', recordCount);
if numel(sourcePackets) >= 2
    fprintf('LiDAR / depth source packets      : %d / %d\n', sourcePackets(1), sourcePackets(2));
end
fprintf('Map snapshots evaluated          : %d\n', n);
fprintf('Persistent frontier tracks created: %d\n', R1.report.frontierTrackCount);
fprintf('Total candidate evaluations      : %d\n', totalCandidates);
fprintf('Candidates passing all hard gates: %d (%.1f%%)\n', ...
    totalAccepted, 100*totalAccepted/max(totalCandidates,1));
fprintf('Selected shadow recommendations  : %d\n', selectedRecommendations);
fprintf('Unsafe accepted candidates       : %d\n', R1.report.unsafeAcceptedCandidates);
fprintf('Truth-map accesses               : %d\n', R1.report.truthAccessCount);
fprintf('Commands issued                  : %d\n', R1.report.commandIssued);
fprintf('Interpretable repeat arrays exact: %d\n', interpretableReplayExact);
fprintf('Saved deterministic digests exact: %d\n', ...
    strcmp(R1.report.frontierViewpointDigest, R2.report.frontierViewpointDigest));
fprintf('============================================================\n\n');

metricsFile = fullfile(outDir, 'poster_metrics.txt');
fid = fopen(metricsFile, 'w');
assert(fid >= 0, 'Could not create %s', metricsFile);
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, 'Scenario: %s\n', string(R1.report.scenario));
fprintf(fid, 'Recorded perception updates: %d\n', recordCount);
if numel(sourcePackets) >= 2
    fprintf(fid, 'LiDAR / depth source packets: %d / %d\n', sourcePackets(1), sourcePackets(2));
end
fprintf(fid, 'Map snapshots evaluated: %d\n', n);
fprintf(fid, 'Persistent frontier tracks created: %d\n', R1.report.frontierTrackCount);
fprintf(fid, 'Total candidate evaluations: %d\n', totalCandidates);
fprintf(fid, 'Candidates passing all hard gates: %d (%.2f%%)\n', ...
    totalAccepted, 100*totalAccepted/max(totalCandidates,1));
fprintf(fid, 'Selected shadow recommendations: %d\n', selectedRecommendations);
fprintf(fid, 'Unsafe accepted candidates: %d\n', R1.report.unsafeAcceptedCandidates);
fprintf(fid, 'Truth-map accesses: %d\n', R1.report.truthAccessCount);
fprintf(fid, 'Commands issued: %d\n', R1.report.commandIssued);
fprintf(fid, 'Interpretable repeat arrays exact: %d\n', interpretableReplayExact);
clear cleanup;

%% CHOOSE AN INFORMATIVE SNAPSHOT
% Prefer a snapshot with an accepted target-relevant selection. Otherwise use
% the snapshot with the largest number of accepted candidates.
score = -inf(n,1);
for k = 1:n
    e = R1.shadowLog{k};
    if selectedId(k) > 0
        j = find(double([e.candidates.candidateId]) == selectedId(k), 1);
        if ~isempty(j)
            score(k) = 1e6*double(e.candidates(j).targetRelevance) + ...
                1e3*double(e.candidates(j).informationGain) + acceptedCount(k);
        end
    else
        score(k) = acceptedCount(k);
    end
end
[~, kBest] = max(score);
E = R1.shadowLog{kBest};
snap = T.log.mapSnapshots{kBest};

xs = double(T.maps.finalGrid.xs(:));
ys = double(T.maps.finalGrid.ys(:));

% Find estimated vehicle position at the snapshot time.
[~, it] = min(abs(double(T.log.t(:)) - double(snap.time)));
if size(T.log.estP,2) == 3
    startXY = double(T.log.estP(it,1:2));
else
    startXY = double(T.log.estP(1:2,it)).';
end
goalXY = double(T.scenario.goal(1:2));

%% FIGURE 1 — ACTUAL MAP, FRONTIERS AND VIEWPOINT DECISIONS
fig = figure('Color','w','Position',[100 100 2200 1300]);
ax = axes(fig); hold(ax,'on');

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
colormap(ax, [1 1 1; 0.84 0.91 0.97; 0.72 0.75 0.80; 0.18 0.20 0.24; 0.95 0.63 0.15]);
caxis(ax,[0 4]);

% Frontier cells.
for i = 1:numel(E.frontiers)
    cells = E.frontiers(i).cells;
    if isempty(cells), continue; end
    plot(ax, xs(cells(:,2)), ys(cells(:,1)), '.', ...
        'MarkerSize',13,'Color',[0.05 0.45 0.85]);
end

% Candidates: rejected, accepted, selected.
% Rejected proposals may intentionally lie outside the map or use [NaN NaN]
% as a sentinel. Those are valid rejection outcomes, but they cannot be used
% as MATLAB array indices. Plot only finite, integer, in-bounds cells.
legendHandles = gobjects(0);
legendLabels = strings(0);

% Add one frontier handle for the legend when frontiers exist.
if ~isempty(E.frontiers)
    hFrontier = plot(ax, nan, nan, '.', 'MarkerSize',13, ...
        'Color',[0.05 0.45 0.85]);
    legendHandles(end+1) = hFrontier; %#ok<SAGROW>
    legendLabels(end+1) = "Frontier cells"; %#ok<SAGROW>
end

invalidRejectedCount = 0;
if ~isempty(E.candidates)
    accepted = logical([E.candidates.accepted]).';
    cells = double(vertcat(E.candidates.cell));

    validCell = size(cells,2) >= 2 & ...
        all(isfinite(cells(:,1:2)),2) & ...
        all(cells(:,1:2) == round(cells(:,1:2)),2) & ...
        cells(:,1) >= 1 & cells(:,1) <= numel(ys) & ...
        cells(:,2) >= 1 & cells(:,2) <= numel(xs);

    % An accepted candidate must always have a valid map cell.
    assert(~any(accepted & ~validCell), ...
        'An accepted viewpoint contains an invalid or out-of-bounds grid cell.');

    rejectedValid = ~accepted & validCell;
    acceptedValid = accepted & validCell;
    invalidRejectedCount = nnz(~accepted & ~validCell);

    if any(rejectedValid)
        hRejected = plot(ax, xs(cells(rejectedValid,2)), ...
            ys(cells(rejectedValid,1)), 'x', ...
            'MarkerSize',4,'LineWidth',0.75,'Color',[0.74 0.28 0.23]);
        legendHandles(end+1) = hRejected; %#ok<SAGROW>
        legendLabels(end+1) = "Rejected proposal"; %#ok<SAGROW>
    end

    if any(acceptedValid)
        hAccepted = plot(ax, xs(cells(acceptedValid,2)), ...
            ys(cells(acceptedValid,1)), 'o', ...
            'MarkerSize',5,'LineWidth',1,'Color',[0.05 0.55 0.28]);
        legendHandles(end+1) = hAccepted; %#ok<SAGROW>
        legendLabels(end+1) = "Candidate passing hard gates"; %#ok<SAGROW>
    end
end

% Selected candidate and its known-free route.
if selectedId(kBest) > 0
    j = find(double([E.candidates.candidateId]) == selectedId(kBest), 1);
    assert(~isempty(j), 'Selected candidate ID was not found in the snapshot.');
    q = E.candidates(j);

    if ~isempty(q.path)
        pathCells = double(q.path(:,1:2));
        validPath = all(isfinite(pathCells),2) & ...
            all(pathCells == round(pathCells),2) & ...
            pathCells(:,1) >= 1 & pathCells(:,1) <= numel(ys) & ...
            pathCells(:,2) >= 1 & pathCells(:,2) <= numel(xs);
        assert(all(validPath), ...
            'Selected candidate path contains an invalid grid cell.');
        hRoute = plot(ax, xs(pathCells(:,2)), ys(pathCells(:,1)), '-', ...
            'LineWidth',3,'Color',[0.55 0.10 0.72]);
        legendHandles(end+1) = hRoute; %#ok<SAGROW>
        legendLabels(end+1) = "Recommended route (not executed)"; %#ok<SAGROW>
    end

    selectedCell = double(q.cell(1:2));
    validSelected = all(isfinite(selectedCell)) && ...
        all(selectedCell == round(selectedCell)) && ...
        selectedCell(1) >= 1 && selectedCell(1) <= numel(ys) && ...
        selectedCell(2) >= 1 && selectedCell(2) <= numel(xs);
    assert(validSelected, 'Selected candidate has an invalid grid cell.');

    hSelected = plot(ax, xs(selectedCell(2)), ys(selectedCell(1)), 'p', ...
        'MarkerSize',18,'MarkerFaceColor',[0.55 0.10 0.72], ...
        'MarkerEdgeColor','w','LineWidth',1.5);
    legendHandles(end+1) = hSelected; %#ok<SAGROW>
    legendLabels(end+1) = "Recommended next view"; %#ok<SAGROW>
end

hDrone = plot(ax, startXY(1), startXY(2), 'o', 'MarkerSize',12, ...
    'MarkerFaceColor',[0.05 0.30 0.55],'MarkerEdgeColor','w','LineWidth',1.5);
legendHandles(end+1) = hDrone; legendLabels(end+1) = "Current drone position";

hTarget = plot(ax, goalXY(1), goalXY(2), 'p', 'MarkerSize',17, ...
    'MarkerFaceColor',[0.85 0.25 0.05],'MarkerEdgeColor','w','LineWidth',1.5);
legendHandles(end+1) = hTarget; legendLabels(end+1) = "Target";

% Mark the mission start separately from the current replay position.
if size(T.log.estP,2) == 3
    missionStartXY = double(T.log.estP(1,1:2));
else
    missionStartXY = double(T.log.estP(1:2,1)).';
end
hStart = plot(ax, missionStartXY(1), missionStartXY(2), 's', 'MarkerSize',10, ...
    'MarkerFaceColor',[0.30 0.65 0.90],'MarkerEdgeColor','w','LineWidth',1.3);
legendHandles(end+1) = hStart; legendLabels(end+1) = "Mission start";
text(ax, missionStartXY(1)+0.07, missionStartXY(2)+0.07, 'START', ...
    'FontSize',13,'FontWeight','bold','Color',[0.05 0.30 0.55], ...
    'BackgroundColor','w','Margin',1,'Interpreter','none');
text(ax, goalXY(1)+0.07, goalXY(2)+0.07, 'TARGET', ...
    'FontSize',13,'FontWeight','bold','Color',[0.75 0.22 0.04], ...
    'BackgroundColor','w','Margin',1,'Interpreter','none');
text(ax,0.01,0.99, ...
    'Map: light blue = known free | grey = unknown | dark = obstacle/inflation | orange = dynamic', ...
    'Units','normalized','VerticalAlignment','top','FontSize',12.5, ...
    'FontWeight','bold','BackgroundColor','w','Margin',3,'Interpreter','none');

xlabel(ax,'x [m]'); ylabel(ax,'y [m]');
title(ax, sprintf('Recorded replay at t = %.1f s', double(E.time)), ...
    'FontWeight','bold','Interpreter','none');
set(ax,'FontSize',18,'LineWidth',1.2,'Box','on');
legend(ax, legendHandles, cellstr(legendLabels), ...
    'Location','eastoutside','FontSize',14);

% Outside-map and undefined rejected proposals remain counted in the
% metrics, but are intentionally omitted from this spatial display.
exportgraphics(fig, fullfile(outDir,'01_actual_frontier_viewpoint_map.png'), ...
    'Resolution',400,'BackgroundColor','white');
close(fig);

%% FIGURE 2 — FINAL UNCERTAINTY SIDECAR LAYERS
map = T.maps.probabilisticMap;
zs = double(map.zs(:));
[~, iz] = min(abs(zs - double(R1.c.nominalAltitude_m)));
H = double(R1.u.entropy(:,:,iz));
Q = double(R1.u.sourceQuality(:,:,iz));
A = double(R1.u.observationAge(:,:,iz));
finiteAge = A(isfinite(A));
if isempty(finiteAge), ageCap = 1; else, ageCap = max(1, prctile(finiteAge,95)); end
A(~isfinite(A)) = ageCap;
A = min(A,ageCap);

fig = figure('Color','w','Position',[100 100 2400 800]);
tl = tiledlayout(fig,1,3,'Padding','compact','TileSpacing','compact');
axH = nexttile; imagesc(axH,xs,ys,H); axis(axH,'xy','equal','tight'); caxis(axH,[0 1]);
cb = colorbar(axH); cb.Label.String = 'Normalized entropy';
title(axH,'Occupancy entropy'); xlabel(axH,'x [m]'); ylabel(axH,'y [m]'); set(axH,'FontSize',18);
axQ = nexttile; imagesc(axQ,xs,ys,Q); axis(axQ,'xy','equal','tight'); caxis(axQ,[0 1]);
cb = colorbar(axQ); cb.Label.String = 'Accumulated quality';
title(axQ,'Accumulated sensor-source quality'); xlabel(axQ,'x [m]'); ylabel(axQ,'y [m]'); set(axQ,'FontSize',18);
axA = nexttile; imagesc(axA,xs,ys,A); axis(axA,'xy','equal','tight');
cb = colorbar(axA); cb.Label.String = 'Age [s]';
title(axA,sprintf('Observation age, capped at %.1f s',ageCap)); xlabel(axA,'x [m]'); ylabel(axA,'y [m]'); set(axA,'FontSize',18);
title(tl, sprintf('Final read-only uncertainty layer at z = %.2f m',zs(iz)), ...
    'FontSize',24,'FontWeight','bold','Interpreter','none');
exportgraphics(fig, fullfile(outDir,'02_uncertainty_sidecar_layers.png'), ...
    'Resolution',400,'BackgroundColor','white');
close(fig);

%% FIGURE 3 — REPLAY TIMELINE AND DETERMINISM
fig = figure('Color','w','Position',[100 100 2200 1150]);
tl = tiledlayout(fig,2,1,'Padding','compact','TileSpacing','compact');

ax1 = nexttile; hold(ax1,'on');
yyaxis(ax1,'left');
plot(ax1,time,frontierCells,'LineWidth',2.3,'DisplayName','Frontier cells — run 1');
plot(ax1,time,frontier2,'--','LineWidth',1.6,'DisplayName','Frontier cells — run 2');
ylabel(ax1,'Frontier cells');
yyaxis(ax1,'right');
plot(ax1,time,clusters,'LineWidth',2.2,'DisplayName','Frontier clusters');
ylabel(ax1,'Frontier clusters');
grid(ax1,'on'); legend(ax1,'Location','northeast');
set(ax1,'FontSize',18,'LineWidth',1.1); title(ax1,'Persistent frontier evolution');

ax2 = nexttile; hold(ax2,'on');
yyaxis(ax2,'left');
plot(ax2,time,candidateCount,'LineWidth',2.2,'DisplayName','Evaluated proposals');
ylabel(ax2,'Evaluated proposals');
yyaxis(ax2,'right');
plot(ax2,time,acceptedCount,'LineWidth',2.5,'DisplayName','Passed hard gates — run 1');
plot(ax2,time,accepted2,'--','LineWidth',1.7,'DisplayName','Passed hard gates — run 2');
ylabel(ax2,'Passed hard gates');
xlabel(ax2,'Replay time [s]'); grid(ax2,'on'); legend(ax2,'Location','northeast');
set(ax2,'FontSize',18,'LineWidth',1.1); title(ax2,'Independent repeated replay');

maxFrontierDiff = max(abs(frontierCells-frontier2));
maxAcceptedDiff = max(abs(acceptedCount-accepted2));
annotation(fig,'textbox',[0.59 0.445 0.36 0.055], ...
    'String',sprintf('Run 1 = Run 2 exactly | max count difference = %.0f | digest match = %d', ...
    max(maxFrontierDiff,maxAcceptedDiff), ...
    strcmp(R1.report.frontierViewpointDigest,R2.report.frontierViewpointDigest)), ...
    'FitBoxToText','on','BackgroundColor','white','EdgeColor',[0.15 0.45 0.28], ...
    'Color',[0.10 0.35 0.22],'FontWeight','bold','FontSize',15);

title(tl,'Replay evolution and repeatability','FontSize',25,'FontWeight','bold','Interpreter','none');
exportgraphics(fig, fullfile(outDir,'03_replay_timeline_determinism.png'), ...
    'Resolution',400,'BackgroundColor','white');
close(fig);

%% FIGURE 4 — WHY CANDIDATES WERE REJECTED
allReasons(allReasons == "") = [];
if isempty(allReasons)
    reasonNames = "No rejections recorded";
    reasonCounts = 0;
else
    [reasonNames,~,ic] = unique(allReasons);
    reasonCounts = accumarray(ic,1);
end

% Replace internal enumeration codes with professor-readable labels.
prettyNames = strings(size(reasonNames));
for i = 1:numel(reasonNames)
    switch char(reasonNames(i))
        case 'IRRELEVANT_EXPLORATION'
            prettyNames(i) = "Irrelevant to target corridor";
        case 'POSITION_OCCUPIED_OR_UNKNOWN_INFLATED'
            prettyNames(i) = "Position occupied, unknown or inflated";
        case 'OUTSIDE_MAP'
            prettyNames(i) = "Outside mapped region";
        case 'STOPPING_SUPPORT_INVALID'
            prettyNames(i) = "Insufficient stopping support";
        case 'INSUFFICIENT_VISIBLE_UNKNOWN'
            prettyNames(i) = "Insufficient visible unknown space";
        case 'UNREACHABLE_KNOWN_FREE'
            prettyNames(i) = "No known-free route";
        case 'RETREAT_ROUTE_INVALID'
            prettyNames(i) = "No valid retreat route";
        case 'STALE_ROUTE_REQUIRES_RESCAN'
            prettyNames(i) = "Stale route requires rescan";
        otherwise
            prettyNames(i) = strrep(reasonNames(i),'_',' ');
    end
end

% Show the most frequently activated reasons, ordered for a readable bar chart.
[reasonCounts,ord] = sort(reasonCounts,'descend');
prettyNames = prettyNames(ord);
maxShown = min(8,numel(reasonCounts));
reasonCounts = reasonCounts(1:maxShown);
prettyNames = prettyNames(1:maxShown);
[reasonCounts,ord] = sort(reasonCounts,'ascend');
prettyNames = prettyNames(ord);

fig = figure('Color','w','Position',[100 100 1900 1050]);
ax = axes(fig);
barh(ax,categorical(prettyNames,prettyNames),reasonCounts);
xlabel(ax,'Number of check failures (one proposal may appear in several bars)');
title(ax,{'Why viewpoint proposals were filtered out', ...
    'One proposal can fail more than one check'}, ...
    'FontWeight','bold','Interpreter','none');
grid(ax,'on'); set(ax,'FontSize',19,'LineWidth',1.1,'TickLabelInterpreter','none');
xpad = max(reasonCounts)*0.015;
for i = 1:numel(reasonCounts)
    text(ax,reasonCounts(i)+xpad,i,sprintf('%d',reasonCounts(i)), ...
        'VerticalAlignment','middle','FontSize',16);
end
xlim(ax,[0 max(reasonCounts)*1.13]);
exportgraphics(fig, fullfile(outDir,'04_candidate_rejection_reasons.png'), ...
    'Resolution',400,'BackgroundColor','white');
close(fig);

%% FIGURE 5 — CANDIDATE SCREENING FUNNEL
rejectedTotal = totalCandidates-totalAccepted;
scenarioLabel = prettyScenarioName(string(R1.report.scenario));
fig = figure('Color','w','Position',[100 100 1800 1050]);
ax = axes(fig); axis(ax,[0 1 0 1]); axis(ax,'off'); hold(ax,'on');

% Header and context. No raw enum strings are shown, so underscores cannot
% be interpreted as subscripts and long names cannot overflow.
title(ax,'How viewpoint proposals were filtered','FontSize',27, ...
    'FontWeight','bold','Interpreter','none');
text(ax,0.5,0.91,"Recorded case: " + scenarioLabel, ...
    'HorizontalAlignment','center','FontSize',17,'Color',[0.28 0.38 0.48], ...
    'Interpreter','none');

levels = [0.73 0.50 0.29 0.10];
widths = [0.82 0.62 0.44 0.30];
labels = { ...
    sprintf('%d proposals checked\nacross %d recorded map states',totalCandidates,n), ...
    sprintf('%d passed every hard gate\n(%.2f%% of proposals)', ...
        totalAccepted,100*totalAccepted/max(totalCandidates,1)), ...
    sprintf('%d next-view recommendations\nselected in shadow mode',selectedRecommendations), ...
    sprintf('%d unsafe accepted viewpoints',R1.report.unsafeAcceptedCandidates)};

for i = 1:4
    x0 = 0.5-widths(i)/2;
    face = [0.90-0.045*i 0.95-0.035*i 0.98-0.02*i];
    rectangle(ax,'Position',[x0 levels(i) widths(i) 0.13], ...
        'Curvature',0.08,'FaceColor',face, ...
        'EdgeColor',[0.08 0.36 0.52],'LineWidth',1.8);
    text(ax,0.5,levels(i)+0.065,labels{i},'HorizontalAlignment','center', ...
        'VerticalAlignment','middle','FontSize',18,'FontWeight','bold', ...
        'Interpreter','none');
    if i < 4
        annotation(fig,'arrow',[0.5 0.5],[levels(i)-0.01 levels(i+1)+0.145], ...
            'Color',[0.08 0.36 0.52],'LineWidth',1.8);
    end
end
text(ax,0.5,0.015,sprintf('%d proposals were filtered out before a recommendation was chosen.',rejectedTotal), ...
    'HorizontalAlignment','center','FontSize',16,'Color',[0.35 0.42 0.50], ...
    'Interpreter','none');
exportgraphics(fig, fullfile(outDir,'05_candidate_outcome_summary.png'), ...
    'Resolution',400,'BackgroundColor','white');
close(fig);

fprintf('Poster figures saved in:\n%s\n', outDir);
fprintf('Use the actual metric values in poster_metrics.txt; do not copy assumed counts.\n');


function out = prettyScenarioName(raw)
% Convert internal enum text into a short human-readable label.
raw = upper(string(raw));
switch raw
    case "DYNAMIC_TO_STATIC_MAPPING"
        out = "dynamic obstacle becomes static";
    case "UNKNOWN_ROOM_NOMINAL"
        out = "unknown room - nominal mission";
    case "LATE_CORRIDOR_BLOCKAGE_REPLAN"
        out = "corridor becomes blocked";
    case "OCCLUDED_OBSTACLE"
        out = "occluded obstacle";
    otherwise
        out = lower(strrep(raw,"_"," "));
end
end
