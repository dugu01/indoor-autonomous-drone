function model = init_perception_model_S2_3()
% INIT_PERCEPTION_MODEL_S2_3 State for raw obstacle-perception simulation.
model.sequence=uint64(0);
model.lastLidarTime=-inf;
model.lastDepthTime=-inf;
model.lastAnyTime=-inf;
end
