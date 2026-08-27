function u = init_uncertainty_sidecar_S2_4(map)
% INIT_UNCERTAINTY_SIDECAR_S2_4 Allocate non-authoritative S2.4 map layers.
arguments
    map (1,1) struct
end
sz = size(map.logOdds);
u = struct();
u.schema = 'S2_4_UNCERTAINTY_SIDECAR_V1';
u.entropy = ones(sz,'single');
u.observationCount = zeros(sz,'uint16');
u.lastObservedTime = -inf(sz,'single');
u.observationAge = inf(sz,'single');
u.lidarHitCount = zeros(sz,'uint16');
u.lidarFreeCount = zeros(sz,'uint16');
u.depthHitCount = zeros(sz,'uint16');
u.depthFreeCount = zeros(sz,'uint16');
u.sourceMask = zeros(sz,'uint8');
u.sourceQuality = zeros(sz,'single');
u.staticConfidence = zeros(sz,'single');
u.dynamicConfidence = zeros(sz,'single');
u.staleFree = false(map.ny,map.nx);u.authoritativeMapVersion = uint32(0);u.sidecarVersion = uint32(0);u.lastTimestamp = -inf;u.acceptedSourcePackets = uint32([0 0]);u.rejectedSourcePackets = uint32(0);u.truthAccessCount = uint32(0);
end
