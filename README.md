# lcls-twincat-common-components

This is the LCLS TwinCAT3 project lcls-twincat-common-components (formerly lcls2-cc-lib).

[Documentation](https://pcdshub.github.io/lcls-twincat-common-components)

## Purpose

This repository contains supporting PLC code for common components used in the LCLS.
Including:

* ATM (Arrival Time Monitor)
* LIC (Laser Incoupling Mirrors)
* PPM (Power and Profile Monitor)
* REF (Reference Laser)
* SLITS (Slits)
* WFS (Wavefront Sensor)
* XPIM (XTES imager)

## Declaring a device in production code

A device FB injects all its dependencies through `FB_Init` at instantiation. Declare the
supporting pieces first, then the device, then tick everything cyclically in the body. The
steps below walk through a REF device.

### 1. Declare

```iecst
VAR
    // Motion: persistent storage, one axis per motor, collected as interfaces.
    fbPDS    : LCLS_Tc_DevAbs.FB_PersistentDataStorage;
     {attribute 'pytmc' := '
        pv: TST:Y
        io: io
        axis-link: GVL_Axes.Axes[1]
    '}   
    fbMotorY : FB_MotionStageNC(
        AxisRef := GVL_Axes.Axes[1],
        iPersistentDataStorage := fbPDS,
        sName := 'Y'
    );
    aiMotionStage : ARRAY[1..MotionConstants.MAX_STATE_MOTORS] OF I_MotionStage := [fbMotorY];

    // Position-state storage (exposed to EPICS via pytmc).
    {attribute 'pytmc' := '
        pv: TST:Y:STATE
        io: io
        array: 1..1 | 1..2
    '}
    astPositionState : ARRAY[1..MotionConstants.MAX_STATE_MOTORS]
                         OF ARRAY[1..GeneralConstants.MAX_STATES] OF ST_PositionState;

    // PMPS plumbing: fast-fault output, arbiter, and the DB reader.
    fbFFHWO     : FB_HardwareFFOutput;
    fbArbiter   : FB_Arbiter(1);
    fbArbiterIO : FB_SubSysToArbiter_IO;

    sDeviceName    : STRING := 'TST:REF';
    sTransitionKey : STRING := 'TST:REF-TRANSITION';

    // One PMPS configurator per state slot. Each writes its slot once in
    // FB_CC_StatePMPSConfig.CallAfterInit; no cyclic tick is needed.
    stDefault : ST_PositionState := (bMoveOk := TRUE, bValid := TRUE, bUseRawCounts := FALSE);
    afbStatePMPS : ARRAY[1..2] OF FB_CC_StatePMPSConfig[
        (stPositionState := astPositionState[1][E_REF_States.OUT], stDefault := stDefault,
         fPosition := 10.0, sPmpsState := 'TST:REF-OUT'),
        (stPositionState := astPositionState[1][E_REF_States.IN],  stDefault := stDefault,
         fPosition := 20.0, sPmpsState := 'TST:REF-IN')
    ];

    // The device. All dependencies are passed through FB_Init.
    {attribute 'pytmc' := 'pv: TST:Y'}
    fbREF : FB_REF(
        aiMotionStage    := aiMotionStage,
        astPositionState := astPositionState,
        fbFFHWO          := fbFFHWO,
        fbArbiter        := fbArbiter,
        sDeviceName      := sDeviceName,
        sTransitionKey   := sTransitionKey,
        bEnableBeamParams     := TRUE,
        bEnablePositionLimits := TRUE,
        bReadDBNow            := FALSE
    );
END_VAR
```

### 2. Define (cyclic body)

Tick the persistent storage, each motor, the device, and the arbiter IO every scan. Do
one-time motor setup behind a first-scan guard.

```iecst
IF bFirstScan THEN
    fbMotorY.SetModeEnableLimits(
        LimBackward := TRUE, LimForward := TRUE, HomeEnable := FALSE,
        BrakeMode := E_StageBrakeMode.IF_ENABLED, EnableMode := E_StageEnableMode.DURING_MOTION,
        HomeMode := E_EpicsHomeCmd.LOW_LIMIT, UserEnable := TRUE, HardwareEnable := TRUE);
    fbMotorY.SetVelocity(Velocity := 15.5);
    bFirstScan := FALSE;
END_IF

fbPDS();
fbMotorY();
fbREF(
    bFileReaderBusy  := MOTION_GVL.fbPmpsFileReader.bBusy,
    bFileReaderError := MOTION_GVL.fbPmpsFileReader.bError
);
fbArbiterIO(i_bVeto := FALSE, Arbiter := fbArbiter, fbFFHWO := fbFFHWO);
```

Key points:

* Pass every dependency through `FB_Init` at declaration; the device owns no concrete
  instances internally.
* The PMPS configurators (`FB_CC_StatePMPSConfig`) seed their state slots in `CallAfterInit`,
  so they need no body call.
* The program ticks the supporting FBs (`fbPDS`, motors, arbiter IO) and the device FB itself;
  the device does not tick another FB's body internally.
