# Migration: motion-abstraction (ND) device interface

The common-components device FBs (`FB_PPM`, `FB_ATM`, `FB_WFS`, `FB_GBS`,
`FB_IPM`, `FB_LIC`, `FB_REF`, `FB_XPIM`, ...) are now thin wrappers over the
`lcls-twincat-motion-abstraction` (ND) state-mover stack. This note describes
the interface and how to initialize a device in a production PRG.

## What changed

- **The integration PRG owns the motion primitives.** Axes, persistent
  storage, the motion stages, the position-state slots, the fast-fault
  output and the arbiter are all declared in the PRG, not inside the device.
- **Devices receive dependencies by `FB_Init REFERENCE TO`.** The device
  forwards them into its internal `FB_PositionStatePMPSND` and re-forwards the
  enum bridge and run-time toggles each cycle. No device owns an `AXIS_REF`.
- **Per-state config lives in the PRG** via an `FB_CC_StatePMPSConfig` array
  (position + PMPS state key + a `stDefault` baseline). The device layers its
  own per-state name/`fDelta` defaults on top.
- **Non-state motors moved to the integrator.** Legacy X/Z/Zoom/Focus axes are
  no longer owned by the device; declare and wire them in the PRG.

## Ownership contract

| Concern | Owner |
|---|---|
| `AXIS_REF`, `FB_MotionStageNC`, `FB_PersistentDataStorage` | PRG |
| `aiMotionStage`, `astPositionState` | PRG |
| `FB_HardwareFFOutput`, `FB_Arbiter`, file-reader status | PRG |
| Per-state `fPosition` / PMPS key (`FB_CC_StatePMPSConfig`) | PRG |
| Per-state `sName` / `fDelta` defaults | device FB |
| State-mover, PMPS gating, beam-param lookup | device FB (via `fbStates`) |

## Production init (single-axis device, e.g. `FB_PPM`)

```iecst
PROGRAM PRG_xxL0_PPM
VAR
    // --- caller-owned motion primitives ---
    axRef : AXIS_REF;                                // linked to the NC axis in the project
    fbPDS : LCLS_Tc_DevAbs.FB_PersistentDataStorage;

    {attribute 'pytmc' := '
        pv: TST:PPM
        io: io
        axis-link: GVL_Axes.Axes[1]
        expand-names: Y
    '}
    fbMotorY : FB_MotionStageNC(AxisRef := axRef, iPersistentDataStorage := fbPDS, sName := 'Y');

    aiMotionStage : ARRAY[1..MotionConstants.MAX_STATE_MOTORS] OF I_MotionStage := [fbMotorY];

    {attribute 'pytmc' := '
        pv: TST:PPM
        io: i
        array: 1..1 | 1..4
        expand-names: Y
    '}
    astPositionState : ARRAY[1..MotionConstants.MAX_STATE_MOTORS]
                         OF ARRAY[1..GeneralConstants.MAX_STATES] OF ST_PositionState;

    // --- PMPS / fast-fault plumbing ---
    fbFFHWO   : FB_HardwareFFOutput;
    fbArbiter : FB_Arbiter(1);

    // --- per-state config (position + PMPS key) ---
    stDefault : ST_PositionState := (bMoveOk := TRUE, bValid := TRUE, bUseRawCounts := FALSE);
    afbStatePMPS : ARRAY[1..4] OF FB_CC_StatePMPSConfig[
        (stPositionState := astPositionState[1][E_PPM_States.OUT],        stDefault := stDefault, fPosition := 10.0, sPmpsState := 'XPP:PPM-OUT'),
        (stPositionState := astPositionState[1][E_PPM_States.POWERMETER], stDefault := stDefault, fPosition := 20.0, sPmpsState := 'XPP:PPM-POWERMETER'),
        (stPositionState := astPositionState[1][E_PPM_States.YAG1],       stDefault := stDefault, fPosition := 30.0, sPmpsState := 'XPP:PPM-YAG1'),
        (stPositionState := astPositionState[1][E_PPM_States.YAG2],       stDefault := stDefault, fPosition := 40.0, sPmpsState := 'XPP:PPM-YAG2')
    ];

    // --- the device (deps injected via FB_Init; pv base is install-specific) ---
    {attribute 'pytmc' := 'pv: XPP:PPM'}
    fbPPM : FB_PPM(
        aiMotionStage         := aiMotionStage,
        astPositionState      := astPositionState,
        fbFFHWO               := fbFFHWO,
        fbArbiter             := fbArbiter,
        sDeviceName           := 'XPP:PPM',
        sTransitionKey        := 'XPP:PPM-TRANSITION',
        bEnableBeamParams     := TRUE,
        bEnablePositionLimits := TRUE,
        bReadDBNow            := TRUE,
        bFileReaderBusy       := MOTION_GVL.fbPmpsFileReader.bBusy,
        bFileReaderError      := MOTION_GVL.fbPmpsFileReader.bError
    );

    i          : UINT;
    bFirstScan : BOOL := TRUE;
END_VAR
```

```iecst
// --- cyclic body: order matters ---
IF bFirstScan THEN
    fbMotorY.SetModeEnableLimits(
        LimBackward := TRUE, LimForward := TRUE, HomeEnable := FALSE,
        BrakeMode := E_StageBrakeMode.IF_ENABLED, EnableMode := E_StageEnableMode.DURING_MOTION,
        HomeMode := E_EpicsHomeCmd.LOW_LIMIT, UserEnable := TRUE, HardwareEnable := TRUE);
    fbMotorY.SetVelocity(Velocity := 15.5);
    bFirstScan := FALSE;
END_IF

// 1) tick per-state config BEFORE the device
FOR i := 1 TO 4 DO
    afbStatePMPS[i]();
END_FOR

// 2) tick the device (drives fbStates + sub-devices)
fbPPM();

// 3) tick the motion primitives the device references
fbPDS();
fbMotorY();

// 4) service the arbiter / fast-fault output last
fbArbiter( ... );
fbFFHWO.Execute();
```

## Power slits (`FB_SLITS_POWER`)

Blade-aperture device, not a state mover: four blade motors indexed by
`E_SlitsBlade`, eight RTDs and a flow switch. `M_UpdatePMPS(index)` runs each
cycle to snap the requested gap into the PMPS envelope; `index` is the device's
slot in the aperture table. The `fApertureMargin` FB_Init input (mm, default
`0.01`) is the amount trimmed off the authorized width/height when snapping the
request into compliance. Terminal links for the RTDs sit in the instance
`TcLinkTo` pragma.

```iecst
PROGRAM PRG_SL1L0_POWER
VAR
    axRef : ARRAY[1..4] OF AXIS_REF;                 // indexed by E_SlitsBlade
    fbPDS : LCLS_Tc_DevAbs.FB_PersistentDataStorage;
    {attribute 'pytmc' := '
        pv: SL1L0:POWER
        io: io
        array: 1..4
        axis-link: GVL_Axes.Axes[1], GVL_Axes.Axes[2], GVL_Axes.Axes[3], GVL_Axes.Axes[4]
        expand-names: Top, Bottom, North, South
    '}
    afbBlade : ARRAY[1..4] OF FB_MotionStageNC[
        (AxisRef := axRef[E_SlitsBlade.TOP],    iPersistentDataStorage := fbPDS, sName := 'Top'),
        (AxisRef := axRef[E_SlitsBlade.BOTTOM], iPersistentDataStorage := fbPDS, sName := 'Bottom'),
        (AxisRef := axRef[E_SlitsBlade.NORTH],  iPersistentDataStorage := fbPDS, sName := 'North'),
        (AxisRef := axRef[E_SlitsBlade.SOUTH],  iPersistentDataStorage := fbPDS, sName := 'South')
    ];
    aiMotionStage : ARRAY[1..4] OF I_MotionStage := [
        afbBlade[E_SlitsBlade.TOP],   afbBlade[E_SlitsBlade.BOTTOM],
        afbBlade[E_SlitsBlade.NORTH], afbBlade[E_SlitsBlade.SOUTH]
    ];

    fbFFHWO : FB_HardwareFFOutput;

    // Device pv base + RTD terminal links live on the instance pragmas.
    {attribute 'pytmc' := '
        pv: SL1L0:POWER
        io: io
    '}
    {attribute 'TcLinkTo' := '.RTD_TOP_1.iRaw := TIIB[SL1L0-EL3202-E10]^RTD Inputs Channel 1^Value;
                              (* ... remaining 7 RTD channels ... *)'}
    fbSlits : FB_SLITS_POWER(
        aiMotionStage     := aiMotionStage,
        fbFFHWO           := fbFFHWO,
        sDeviceName       := 'SL1L0:POWER',
        sPmpsState        := 'SL1L0:POWER-RTD',
        bFileReaderBusy   := MOTION_GVL.fbPmpsFileReader.bBusy,
        fApertureMargin   := 0.01
    );

    i : UINT;
END_VAR
```

```iecst
// --- cyclic body ---
FOR i := 1 TO 4 DO
    afbBlade[i]();               // parent ticks the blade motors (composition)
END_FOR
fbPDS();

fbSlits.M_UpdatePMPS(index := GVL.nSlitsApertureIndex);  // integrator-driven, valid index
fbSlits(bMoveOk := TRUE);        // aperture math + RTDs + flow switch

fbFFHWO.Execute();
```

## Solid attenuator (`FB_SXR_SATT_Stage`)

Single-axis filter-stack mover (states `OUT` + `FILTER1..8`) built on
`FB_PositionStateND`, not a PMPS state mover. It uses `FB_CC_StateDefaultsConfig`
(position + name, no PMPS key) for the states and pulls only an RTD reactive-temp
setpoint from the DB via `RTDPMPSState`. Per-filter material/thickness is written
into the device's `arrFilters` table by an `FB_SATT_FilterConfig` array. Beam
energy `PMPS_GVL.stCurrentBeamParameters.neV` feeds the transmission calc; keep it
non-zero to avoid a divide-by-zero in the physics library.

```iecst
PROGRAM PRG_AT1K4_SATT
VAR
    axRef : AXIS_REF;
    fbPDS : LCLS_Tc_DevAbs.FB_PersistentDataStorage;

    {attribute 'pytmc' := 'pv: MMS'}
    fbMotorY : FB_MotionStageNC(AxisRef := axRef, iPersistentDataStorage := fbPDS, sName := 'Y');
    aiMotionStage : ARRAY[1..MotionConstants.MAX_STATE_MOTORS] OF I_MotionStage := [fbMotorY];

    {attribute 'pytmc' := '
        pv: TST:D1:STATE
        io: i
        array: 1..1 | 1..9
        expand-names: D1M1, D1M2, D1M3, D1M4
    '}
    astPositionState : ARRAY[1..MotionConstants.MAX_STATE_MOTORS]
                         OF ARRAY[1..GeneralConstants.MAX_STATES] OF ST_PositionState;

    fbFFHWO : FB_HardwareFFOutput;

    {attribute 'pytmc' := 'pv: AT1K4:L2SI'}
    fbSATT : FB_SXR_SATT_Stage(
        aiMotionStage    := aiMotionStage,
        astPositionState := astPositionState,
        fbFFHWO          := fbFFHWO,
        bFileReaderBusy  := MOTION_GVL.fbPmpsFileReader.bBusy,
        sDeviceName      := 'AT1K4:L2SI',
        RTDPMPSState     := 'AT1K4:L2SI-RTD'
    );

    // State positions/names (no PMPS key for SATT).
    stDefault : ST_PositionState := (fDelta := 0.2, bMoveOk := TRUE, bValid := TRUE, bUseRawCounts := FALSE);
    afbStateCfg : ARRAY[1..9] OF FB_CC_StateDefaultsConfig[
        (stPositionState := astPositionState[1][E_SXR_SATT_Position.OUT],     stDefault := stDefault, sName := 'Out',     fPosition := 10),
        (stPositionState := astPositionState[1][E_SXR_SATT_Position.FILTER1], stDefault := stDefault, sName := 'Filter1', fPosition := 20)
        // ... FILTER2..FILTER8
    ];

    // Per-filter material + thickness written into fbSATT.arrFilters.
    afbFilterCfg : ARRAY[1..8] OF FB_SATT_FilterConfig[
        (stFilter := fbSATT.arrFilters[1], sFilterMaterial := 'C',  fFilterThickness_um := 25),
        (stFilter := fbSATT.arrFilters[2], sFilterMaterial := 'Si', fFilterThickness_um := 200)
        // ... filters 3..8
    ];

    i : UINT;
END_VAR
```

```iecst
// --- cyclic body ---
FOR i := 1 TO 9 DO
    afbStateCfg[i]();
END_FOR
FOR i := 1 TO 8 DO
    afbFilterCfg[i]();
END_FOR

PMPS_GVL.stCurrentBeamParameters.neV := 250.0;   // guard the transmission calc

fbSATT();          // fbStates + transmission calc + RTDs
fbPDS();
fbMotorY();
fbFFHWO.Execute();

// Command a filter:  fbSATT.eEnumSet := E_SXR_SATT_Position.FILTER3;
```

## Notes

- **Config-before-device.** Tick the `FB_CC_StatePMPSConfig` array before the
  device each cycle; the device's per-state defaults overwrite the baseline
  `FB_StateSetup` just laid down.
- **Multi-axis devices** size the `afbMotorsNC` / `aiMotionStage` arrays to the
  active motor count and set `nActiveMotorCount` accordingly (see the
  motion-abstraction `PRG_PMPS` test program for a 4-axis example).
- **Commanding moves:** write `fbPPM.eEnumSet := E_PPM_States.YAG1;` and read
  back `fbPPM.eEnumGet`.
