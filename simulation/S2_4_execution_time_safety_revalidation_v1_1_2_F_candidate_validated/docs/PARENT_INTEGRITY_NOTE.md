# Uploaded S2.3 parent integrity note

The package freezes the exact bytes from the user-supplied archive:

```text
S2_3_online_mapping_v1_0_0_validated.zip
SHA-256: 7b898ce312821958cddb6d87152c8728f7f7614b46c3c56920dfe069839c2226
```

The latest inherited `FINAL_CLOSURE_PACKAGE_MANIFEST.sha256` checks 203 paths.
All 203 paths match exactly, including every MATLAB and Python source file and
the final release evidence files. The inherited manifest is not edited.

S2.4 also stores an external `FROZEN_PARENT_SHA256SUMS.txt` covering all 208
files extracted from the uploaded archive. `audit_parent_immutability.py` must
pass before and after every S2.4 validation run. Validation is performed on a
disposable working copy; the frozen parent is read-only by contract.
