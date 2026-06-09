# Migration: lcls-twincat-motion-abstraction old surface -> ND framework

## Status

In progress. Tracking the migration of every motion-library consumer in this
library from the pre-ND public surface to the new `lcls-twincat-motion`
master surface (`FB_MotionStageNC`, `FB_PositionStateND`, `FB_PositionStatePMPSND`,
`I_MotionStage`, etc.).

## Hard constraints (carried over from `docs/memory/`)

- No `VAR_IN_OUT` for FB dependencies in new code; inject via `FB_Init` /
  `CallAfterInit`. Existing consumers that still take `ST_MotionStage` via
  `VAR_IN_OUT` are migrated to take an injected `I_MotionStage` (or owned
  `FB_MotionStageNC` instance).
- No short-circuit eval. Always nest interface null-checks and array
  bound-checks before dereference.
- Every new `.TcPOU` / `.TcDUT` / `.TcIO` / `.TcGVL` must be added to
  `Library/Library.plcproj` `<Compile Include="...">` with Windows backslashes,
  alphabetical within folder section.
- Branches: `feature/<desc>` only; never commit to `master`. No `git push`
  from this VM (handled by the human).

## Rules pulled from motion-lib README (must be honored)

These were not captured in the original plan and reshape Phases 2-4. Source:
`lcls-twincat-motion-abstraction/README.md` sections "State Movers (ND Framework)"
and "PMPS Integration".

### R1. Motion params are NOT on `ST_PositionState`

> "Motion parameters are not part of the state. Velocity, acceleration,
> deceleration, and jerk are owned by the drive layer (`FB_MotionStageNC`,
> exposed via `MotionCmd`), not by `ST_PositionState`."

The legacy `FB_PositionState_Defaults` defaults five fields (`sNameDefault`,
`fVeloDefault`, `fDeltaDefault`, `fAccelDefault`, `fDecelDefault`). Only
`sName` and `fDelta` survive as state-resident fields in the new framework.
The other three (`fVelocity`, `fAccel`, `fDecel`) must move to per-motor
`afbMotorsNC[i].SetMotionParam(Velocity := ..., Acceleration := ...,
Deceleration := ...)` calls inside each device FB's first-scan block.

This changes every device FB body, not just signatures.

### R2. EPICS enum: index 0 must mean `Unknown`

> "Index 0 must mean `Unknown`. State indices `1..N` must match the slots in
> `astPositionState[motor][state]` one-to-one."

Audit every `E_<Device>_States` DUT in `Library/Devices/*/E_*_States.TcDUT`
during Phase 2. If any device used index 0 for a real state, the enum must
be renumbered and any consumer (incl. external projects) flagged.

### R3. 2-D jagged arrays (jagged, not multi-dim)

`ARRAY[1..MAX_STATE_MOTORS] OF ARRAY[1..MAX_STATES] OF ST_PositionState`
with `[motor][state]` access. **Not** `ARRAY[1..M, 1..N]` with `[m, n]`.
Already applied to `FB_CheckPositionStateWrite` and its test.

### R4. `FB_StateConfigurator` semantic is NOT `FB_PositionState_Defaults`

The replacement is real but the semantic differs:
- Legacy `FB_PositionState_Defaults`: "if field empty/zero, fill with
  default" : per-field, runtime-overridable.
- New `FB_StateConfigurator` / `FB_StateSetup`: first cycle writes all
  fields from (template + call-site overrides); subsequent cycles are
  no-ops because `FB_PositionStateInternalND` sets `bUpdated := TRUE`.
  Operator runtime overrides survive via the `bUpdated` latch.
  `bForceUpdate := TRUE` is the explicit "re-apply" knob.

Practical impact: device FBs need **one `FB_StateConfigurator` per (motor,
state) pair**, not one `FB_PositionState_Defaults` called per state. See
README "Setup Example" for the `afbStateConfig` array pattern sized
`MAX_STATE_MOTORS * MAX_STATES`.

### R5. `FB_PositionStatePMPSND` public surface differs from `FB_PositionStatePMPS1D`

| Was on 1D | Status on ND |
|---|---|
| `nCurrGoal` | **NOT public** : goal flow via `stEpicsToPlc.nSetValue` / `eEnumSet` |
| `bAtState` | **NOT public** : `AT_STATE` exposed via `eStatePMPSStatus` |
| `iMotorCount` as `FB_Init` input | **NOT an input** : detected internally; exposed via `GetStateMotorCountND()` |

New `FB_Init` inputs on `FB_PositionStatePMPSND`: `fbFFHWO`, `fbArbiter`,
`eMotionRequestPolicy`, `bEnableBeamParams`, `bEnablePositionLimits`,
`bReadDBNow`, `sDeviceName`, `sTransitionKey`, `bFileReaderBusy`,
`bFileReaderError`. Any test or wrapping FB that previously read
`fbPMPS.nCurrGoal` or `fbPMPS.bAtState` must rewire to the new surface.

### R6. PMPS singleton + sequential `bBeamParamsLoaded` chaining

`MOTION_GVL.fbStandardPMPSDB` is a pre-instantiated singleton; it must be
called **once per PLC in the main PRG** of the integration project. **Not
the library's responsibility** : common-components device FBs only consume
the `fbFFHWO` and `fbArbiter` references the integration project injects.

For multi-PMPS-mover deployments, each mover's `bBeamParamsLoaded`
(one-cycle pulse on first successful DB read) gates the next mover's
`bReadDBNow`. Device FBs should expose `bBeamParamsLoaded` and accept
`bReadDBNow` so the integration project can chain them.

### R7. pytmc array pragma rules

- `expand-names: M1, M2, ...` length **must equal** outer array bound.
  For single-motor legacy devices reshaped into `ARRAY[1..MAX_STATE_MOTORS]`,
  unused slots either need `nc-optional: true` (skip NC-link validation) or
  the device FB must declare its arrays with a tighter bound where possible.
- `axis-link` either explicit list (one entry per element, declaration
  order) or `%d` template. Pytmc only checks Link resolution, not
  `AxisRef` vs `axis-link` consistency : keep them on adjacent lines.
- `ARRAY OF ARRAY OF X` recursively expands; outer uses `expand-names`,
  inner uses numeric default.

This affects every device FB declaration; pytmc PV-name diffs must be
generated and reviewed before each device PR merges.

### R8. `ST_PositionState` fields are read-only from EPICS

All pytmc-exposed `ST_PositionState` fields carry `io: i` (`sName`,
`fPosition`, `nEncoderCount`, `bMoveOk`); the rest (`fDelta`, `bLocked`,
`bValid`, `bUseRawCounts`, `bUpdated`, `stPMPS`) have no pytmc pragma.
**There is no path for an operator to write a position-state field from
EPICS.** State editing happens via `FB_StateConfigurator` (PLC code) or
the JSON DB (`stPMPS` only).

This collapses the entire premise of `FB_CheckPositionStateWrite`: the
legacy helper existed to detect concurrent PLC + EPICS writers and let
the operator edit win. Both halves are gone:
  - "From EPICS" is structurally unrepresentable.
  - "From PLC" is single-owner (one `FB_StateConfigurator` per slot,
    gated by the `bUpdated` latch).

The helper is therefore obsolete : kept on disk under `POUs/` and
`Tests/` for history with an obsolescence note at the top, removed
from `Library.plcproj` so it does not compile.

## Public-surface delta

| Old identifier (still in current Library.plcproj reference) | Status on motion master | New equivalent |
|---|---|---|
| `FB_MotionStage` | removed | `FB_MotionStageNC` (or `FB_MotionStageMCS2`); both EXTEND `FB_MotionStageCommon` which IMPLEMENTS `I_MotionStage` |
| `ST_MotionStage` (struct passed VAR_IN_OUT) | removed | `I_MotionStage` interface; status lives in `MotionStatus : REFERENCE TO ST_MotionStatus`, `ExMotionStatus`, `MotionCmd`, `MotionParamsCmd` |
| `FB_PositionStatePMPS1D` | removed | `FB_PositionStatePMPSND` (EXTENDS `FB_PositionStateND` EXTENDS `FB_PositionStateCoreND`) |
| `FB_PositionState1D` | removed | `FB_PositionStateND` |
| `FB_StatePTPMove` (used by SLITS) | added to motion lib under `02_State/02_Helpers/` | same name; new contract via `I_MotionStage` + `ST_PositionState` injected through `FB_Init` |
| `FB_PositionState_Defaults` (common-components-owned helper) | n/a | obsolete (R1, R4, R8): three of five default fields (fVelocity / fAccel / fDecel) are no longer state-resident; sName is `io:i` so the operator-override branch is unreachable; the apply-once intent is owned by `FB_StateConfigurator` (one per (motor, state) pair) gated by the per-slot `bUpdated` latch. File kept on disk with obsolescence note, removed from `Library.plcproj` build. |
| `FB_CheckPositionStateWrite` (common-components-owned helper) | n/a | obsolete (R8): `ST_PositionState` is read-only from EPICS in ND, the framework's per-slot `bUpdated` latch replaces it. File kept on disk with obsolescence note, removed from `Library.plcproj` build. |
| `ST_PositionState` | kept | same name, compatible field set |
| `DUT_PositionState` (SLITS only) | removed | use `ST_PositionState` |
| `ENUM_StageEnableMode` (member `DURING_MOTION`) | removed | `E_StageEnableMode` (same member names) |
| `ENUM_StageBrakeMode` (member `NO_BRAKE`) | removed | `E_StageBrakeMode` |
| `ENUM_EpicsHomeCmd` (members `NONE`, `LOW_LIMIT`) | removed | `E_EpicsHomeCmd` |
| `E_MotionFFType.DEVICE_MOVE` | kept | same |

## ND-mover wiring contract

```pascal
afbMotorsNC : ARRAY[1..MAX_STATE_MOTORS] OF FB_MotionStageNC[ ... ];
aiMotionStage : ARRAY[1..MAX_STATE_MOTORS] OF I_MotionStage := [ afbMotorsNC[1], ... ];
astPositionState : ARRAY[1..MAX_STATE_MOTORS] OF ARRAY[1..MAX_STATES] OF ST_PositionState;
eEnumSet, eEnumGet : E_AxisStates;

fbStates : FB_PositionStatePMPSND(             (* or FB_PositionStateND if non-PMPS *)
    aiMotionStage        := aiMotionStage,
    astPositionState     := astPositionState,
    nActiveMotorCount    := N,
    eEnumSet             := eEnumSet,
    eEnumGet             := eEnumGet,
    bEnableMotion        := TRUE,
    eMotionRequestPolicy := E_MotionRequestPolicy.WAIT
);
```

## Phases

### Phase 0 - Library reference swap (DONE)

Discovery: the new motion library is not a version bump of the old
`lcls-twincat-motion`; it is a renamed library. The `PlaceholderReference`
identifier, namespace, AND company string all change:

| | Old (pre-migration) | New (motion master) |
|---|---|---|
| Placeholder Include | `lcls-twincat-motion` | `LCLS_Tc_Motion` |
| Company | `SLAC` | `SLAC - LCLS` |
| Namespace | `lcls_twincat_motion` | `LCLS_Tc_Motion` |
| Version | `* (SLAC)` | `0.0.1` (master, `<Released>false</Released>`) |

- [x] `Library/Library.plcproj` `PlaceholderReference` swapped to
      `LCLS_Tc_Motion, * (SLAC - LCLS)` with namespace `LCLS_Tc_Motion`.
- [x] `_Config/PLC/Library.xti` checked: no motion-lib references; it
      is only a System Manager type cache. No edit needed.
- [x] ST source audited: no `lcls_twincat_motion.`-qualified references
      in the library or tests. Unqualified type names rely on the
      placeholder's namespace import, so the rename is transparent for
      identifiers that still exist in the new library. Identifiers that
      were removed or renamed (`ST_MotionStage`, `FB_PositionState1D`,
      `FB_PositionStatePMPS1D`, `FB_PositionState_Defaults`) will surface
      as compiler errors during Phases 1-4.

Pre-req for human: the `LCLS_Tc_Motion` library must be installed in the
TwinCAT library repository on the dev VM (Save as library and install
from the motion `.sln`) before the project will resolve.

### Phase 1 - In-library helpers

- [x] `Library/POUs/FB_CheckPositionStateWrite.TcPOU` : **excluded from
      build** in `Library.plcproj` (kept on disk with an obsolescence
      note). Per R8 the helper's premise (mediate concurrent PLC/EPICS
      writers on `ST_PositionState`) is structurally unrepresentable in
      the ND framework, and the framework provides a stricter mechanism
      via the per-slot `bUpdated` latch wired by `FB_StateConfigurator`
      and `FB_PositionStateInternalND`. Earlier 1-D-to-2-D resize work
      reverted.
- [x] `Library/Tests/FB_CheckPositionStateWriteTest.TcPOU` : same
      treatment. Excluded from build, obsolescence note at top.
- [x] `Library/POUs/FB_PositionState_Defaults.TcPOU` : **excluded from
      build** in `Library.plcproj` (kept on disk with an obsolescence
      note). Rationale (R1 + R4 + R8): three of the five default fields
      it touches (`fVeloDefault`, `fAccelDefault`, `fDecelDefault`) have
      no destination on `ST_PositionState` anymore (per-stage motion
      params live on the drive layer via `SetMotionParam()`); `sName` is
      pytmc `io:i`, so the empty-string fall-back branch guarding against
      an EPICS operator edit is unreachable; the remaining apply-once
      intent is owned by `FB_StateConfigurator` with explicit re-apply
      via `bForceUpdate := TRUE`. Each Phase 2 device PR drops its
      `fbStateDefaults` member and the corresponding body call sequence.
      No test POU exists for this helper.

### Phase 2 - Per-device migration

For each device FB: replace `VAR_IN_OUT stXxx : ST_MotionStage` with
`VAR_IN_OUT fbXxx : FB_MotionStageNC` (or `I_MotionStage`); remove
internal `fbXxx : FB_MotionStage;`; replace `stXxx.n{Enable,Brake,Homing}Mode`
writes with property assignments or one `SetModeEnableLimits(...)` call;
convert `astPositionState` from 1-D to 2-D; swap `FB_PositionStatePMPS1D` ->
`FB_PositionStatePMPSND` (and `FB_PositionState1D` -> `FB_PositionStateND`
for SATT).

Per-device checklist (applies to every entry below):
- R1: Move `fbStateDefaults(..., fVeloDefault, fAccelDefault, fDecelDefault, ...)`
  arguments to one `afbMotorsNC[i].SetMotionParam(Velocity := ...,
  Acceleration := ..., Deceleration := ...)` call in the first-scan block.
  Keep `fDelta` (and `sName`) on the per-state `FB_StateConfigurator`.
- R2: Audit the device's `E_<Device>_States.TcDUT`; reserve index 0 for
  `Unknown` (renumber if needed and tag the device PR with the API break).
- R4: Replace each `fbStateDefaults(stPositionState := stTargetN, ...)` call
  with one entry in the device-owned `afbStateConfig : ARRAY[..] OF
  FB_StateConfigurator[(...)]` inline-init, sized
  `nActiveMotors * <state-count>`. Call all configurators each cycle
  in a flat `FOR` loop.
- R5: If the device exposes any `fbPMPS.nCurrGoal` / `fbPMPS.bAtState`
  to its parent or tests, rewire through `stEpicsToPlc.nSetValue` /
  `eEnumSet` / `eStatePMPSStatus`.
- R7: Add `expand-names: M1[, M2, ...]` to the device's `afbMotorsNC` /
  `astPositionState` / `aiMotionStage` declarations, lengths matching the
  outer bound. For single-motor devices reshaped into
  `ARRAY[1..MAX_STATE_MOTORS]`, mark unused slots with `nc-optional: true`.
  Keep `axis-link` adjacent to `AxisRef` for visual review (pytmc cannot
  check the pair).
- R6: Expose `bBeamParamsLoaded` (output) and accept `bReadDBNow` (input)
  on PMPS-aware device FBs so integration projects can chain movers.

- [ ] `Library/Devices/ATM/FB_ATM.TcPOU` (2 stages)
- [ ] `Library/Devices/GBS/FB_GBS.TcPOU` (1 stage)
- [ ] `Library/Devices/IPM/FB_IPM.TcPOU` (2 stages)
- [ ] `Library/Devices/LIC/FB_LIC.TcPOU` (1 stage)
- [ ] `Library/Devices/PPM/FB_PPM.TcPOU` (1 stage)
- [ ] `Library/Devices/REF/FB_REF.TcPOU` (1 stage)
- [ ] `Library/Devices/WFS/FB_WFS.TcPOU` (2 stages)
- [ ] `Library/Devices/XPIM/FB_XPIM.TcPOU` (3 stages)
- [ ] `Library/Devices/SATT/FB_SXR_SATT_Stage.TcPOU` (1 stage, uses
      `FB_PositionState1D` not PMPS variant)

### Phase 3 - SLITS special case

- [ ] `Library/Devices/SLITS/FB_SLITS.TcPOU` (4 blade stages)
  - Drop `DUT_PositionState` -> use `ST_PositionState`.
  - Per-blade `FB_StatePTPMove` instances (Shape A: minimal delta).
  - `FB_StatePTPMove` now takes `iMotionStage` + `stPositionState`
    through `FB_Init`; remove old `stMotionStage` VAR_IN_OUT wiring.
- [ ] `Library/Devices/SLITS/FB_SLITS_POWER.TcPOU` (audit and migrate)

### Phase 4 - Tests

Update every test FB under `Library/Tests/` so it declares its own
`FB_MotionStageNC` instances and `aiMotionStage` arrays instead of
bare `ST_MotionStage` locals.

- [ ] `Library/Tests/FB_ATMTest.TcPOU`
- [ ] `Library/Tests/FB_LICTest.TcPOU`
- [ ] `Library/Tests/FB_PPMTest.TcPOU`
- [ ] `Library/Tests/FB_REFTest.TcPOU`
- [ ] `Library/Tests/FB_SATTTest.TcPOU`
- [ ] `Library/Tests/FB_SLITS_POWERTest.TcPOU`
- [ ] `Library/Tests/FB_WFSTest.TcPOU`
- [ ] `Library/Tests/FB_XPIMTest.TcPOU`
- [ ] `Library/Tests/PRG_TEST.TcPOU` (top-level test orchestrator)

Per-device TcUnit additions (extends existing test FBs; each test
wraps with `TEST(__POUNAME())` / `TEST_FINISHED()` and uses a
single `VAR_INST` SUT):

| ID | Test | Coverage |
|----|------|----------|
| T01 | Construction | Device instantiates with new VAR_IN_OUT stage(s); no fault on first cycle |
| T02 | First-scan mode/limits | After 1 cycle, `EnableMode`, `BrakeMode`, `HomeMode` on the stage match the device's intent |
| T03 | Position-state defaults | `FB_StateConfigurator` populates `astPositionState[m][s].fDelta/bMoveOk/bValid` |
| T04 | Check-write 2-D | Writing `astPositionState[1][1].fPosition` from "EPICS" flags exactly that slot |
| T05 | Move dispatch | `eEnumSet := <Target1>` -> `fbStates.bExecMove` rises, `bMoverReady` TRUE |
| T06 | Per-motor diagnostics | `aeMotorTransitStatus[m]` and `sErrorMsg` populate per upstream contract |
| T07 | Reset/Halt pass-through | `MotionCmd.bReset := TRUE` cascades to `ExMotionStatus.bLatchReset` |
| T08-SLITS | Per-blade state | Each blade reaches its named target independently |
| T09-SLITS | Aperture math | X/Y aperture computation unchanged numerically by the refactor |

### Phase 5 - Cleanup / release

- [ ] Remove any temporary shims left from Phase 1.
- [ ] Pin the motion-library version in `Library.plcproj`.
- [ ] Update `README.md` (top level) with the API break notice.
- [ ] Tag a release.

## Test gates

- TcUnit `PRG_TEST` finishes with 0 errors / 0 warnings.
- TwinCAT static analysis: 0 new findings vs. master baseline.
- `Library.plcproj` builds standalone and via the tsproj test project.

## Risk register

| Risk | Mitigation |
|---|---|
| Downstream section PLCs were passing one `ST_MotionStage` to multiple devices, sharing state | Compile-break flushes them out; document the new shape (each device owns its stage FB or accepts `I_MotionStage`). |
| 1-D state arrays were serialized to EPICS PVs by name | Diff pytmc-generated PV names before/after for one sample device; tweak pragmas to keep PV names stable. |
| `FB_StatePTPMove` behavior parity with the old 1-D mover | Use Shape A (one motor per mover) first; regression-test SLITS aperture moves before considering a multi-motor consolidation. |
| `RestoreMotionParams` / persistence defaults differ from old surface | Default to `RestoreMotionParams := FALSE` on first integration; document the toggle. |

## Execution order (recommended PR shape)

1. Phase 0 + Phase 1 (helper-only, no device touched; existing tests still compile via temporary shims).
2. One PR per device family in Phase 2 (ATM, then GBS/IPM/LIC, then PPM/REF/WFS/XPIM, then SATT). Each PR carries its T01-T07 tests.
3. SLITS PR (Phase 3).
4. Cleanup PR (Phase 5): remove shims, pin library version, README/CHANGELOG.
