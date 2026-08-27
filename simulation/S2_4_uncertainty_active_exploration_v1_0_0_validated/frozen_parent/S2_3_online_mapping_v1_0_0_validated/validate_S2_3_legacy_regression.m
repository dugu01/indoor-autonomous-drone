function report = validate_S2_3_legacy_regression()
% VALIDATE_S2_3_LEGACY_REGRESSION Run frozen S2.2 validators unchanged.
% This function verifies that inherited S2.2 files still reproduce the
% frozen baseline. It does not route S2.2 scenarios through the S2.3 mapper.
report=struct();
report.deterministic=validate_S2_2(false);
report.focused=validate_S2_2_v0_5_3_focus();
report.robustness=validate_S2_2_multiseed_robustness(0:9);
end
