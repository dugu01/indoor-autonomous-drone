function radius = uncertainty_inflation_S2_2(cfg,xySigma_m)
% UNCERTAINTY_INFLATION_S2_2 Covariance-aware centre-point safety radius.
if ~isfinite(xySigma_m),xySigma_m=cfg.maxInflationRadius;end
radius=cfg.baseInflationRadius+cfg.uncertaintySigmaGain*max(0,xySigma_m);
radius=min(cfg.maxInflationRadius,max(cfg.baseInflationRadius,radius));
end
