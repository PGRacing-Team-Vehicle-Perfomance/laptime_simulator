# Configuration & options reference

Every run is driven by a single config file. This document lists every option the
application reads, grouped by config section, with its unit, default, and effect.

## File format

Config files are plain CSV, one option per line:

```
Section,key,value
```

- The **first field is the section** (subsystem); the second is the key; the third is the value.
- Lines that are empty or start with `#` are ignored — use them for comments/grouping.
- **A key that is absent falls back to its default** (listed below). Keys marked
  *required* have no default; omitting them is an error or undefined behaviour.
- Angles: a key `X` may be paired with `X.unit` set to `deg` or `rad` (default `rad`).
  When `X.unit,deg` is present the value is read in degrees and converted internally.
- The first line of the shipped configs is a header (`Module,Parameter,Value`) — it is
  just an ignored non-matching line.

Example configs live in the repo root: `config_pacejka_v2.csv` (full featured),
`config_pacejka_v2_steering.csv` (adds a steering table + yaw-zero knobs),
`config_simple.csv` (quadratic tire), `config_pacejka_v1.csv`, `pgr09_*.csv`.

## Running

| Command | Output |
| --- | --- |
| `build/laptime_simulator <config.csv>` | `build/yaw_diagram.csv` + `build/metrics.csv` + `build/hull.csv` |
| `build/laptime_simulator <config.csv> tire` | `build/tire_model.csv` (tire force/moment sweeps) |

Make targets wrap these: `make run`, `make plot`, `make tire CONFIG=<cfg>`, `make setups`.
Python tools under `tools/` only visualize the CSVs (see `docs/index.md`).

---

## Environment

Air properties and gravity. Air density is **computed** from temperature, pressure and
humidity via the Magnus formula (`src/config/configHelper.cpp`) unless `airDensity` is
given directly.

| Key | Unit | Default | Effect |
| --- | --- | --- | --- |
| `airTemperature` | °C | 20 | Ambient temperature; feeds air-density calc. |
| `airPressure` | kPa | 100 | Ambient pressure; feeds air-density calc. |
| `airHumidity` | % | 50 | Relative humidity; feeds air-density calc. |
| `airDensity` | kg/m³ | *computed* | Overrides the computed density if present. Frozen at construction. |
| `earthAcc` | m/s² | *required* | Gravitational acceleration (e.g. 9.81). |
| `wind.x/y/z` | m/s | 0 | Wind vector components (present in configs; aero wind coupling is minimal). |

---

## Vehicle

The car: masses, geometry, suspension rates, wheel alignment, drive/brake bias and the
steering table. `frame` selects the reference frame the whole vehicle is expressed in.

### Frame & mass

| Key | Unit | Default | Effect |
| --- | --- | --- | --- |
| `frame` | ISO8855 \| SAE | *required* | Vehicle reference frame. |
| `suspendedMassAtWheels.{FL,FR,RL,RR}` | kg | *required* | Sprung mass carried at each corner. **The CG is derived from these** — changing them moves the CG. |
| `nonSuspendedMassAtWheels.{FL,FR,RL,RR}` | kg | *required* | Unsprung mass at each corner. |
| `suspendedMassHeight` | m | *required* | Height of the sprung-mass CG. |
| `nonSuspendedMassHeight` | m | *required* | Height of the unsprung-mass CG. |

### Geometry

| Key | Unit | Default | Effect |
| --- | --- | --- | --- |
| `frontTrackWidth` | m | *required* | Front track. |
| `rearTrackWidth` | m | *required* | Rear track. |
| `trackDistance` | m | *required* | Wheelbase (front-to-rear axle distance). |
| `rollCenterHeightFront` | m | *required* | Front roll-center height. |
| `rollCenterHeightBack` | m | *required* | Rear roll-center height. |

### Suspension stiffness

Spring and anti-roll-bar rates set lateral load transfer. Motion ratios map wheel to
spring/ARB travel; `*.unit` documents the ratio units.

| Key | Unit | Default | Effect |
| --- | --- | --- | --- |
| `frontKspring` / `rearKspring` | N/mm | *required* | Spring rate per axle. |
| `frontSpringMotionRatio` / `rearSpringMotionRatio` | – | *required* | Wheel↔spring motion ratio. |
| `frontKarb` / `rearKarb` | – | *required* | Anti-roll-bar rate. |
| `frontKarb.unit` / `rearKarb.unit` | – | N/mm | ARB rate unit. |
| `frontArbMotionRatio` / `rearArbMotionRatio` | – | *required* | Wheel↔ARB motion ratio. |
| `frontArbMotionRatio.unit` / `rearArbMotionRatio.unit` | – | mm/mm | ARB motion-ratio unit. |

### Wheel alignment

| Key | Unit | Default | Effect |
| --- | --- | --- | --- |
| `toeAngle.{FL,FR,RL,RR}` (+ `toeAngle.unit`) | deg/rad | *required* | Static toe per wheel (mirrored by side internally). |
| `camber.{FL,FR,RL,RR}` (+ `camber.unit`) | deg/rad | *required* | Static camber per wheel; consumed by both Pacejka models. Constant per run (no kinematic camber gain). |

### Drive / brake

| Key | Unit | Default | Effect |
| --- | --- | --- | --- |
| `driveBiasFront` | 0–1 | 0.0 | Fraction of drive torque to the front axle. |
| `brakeBiasFront` | 0–1 | 0.6 | Fraction of brake torque to the front axle. |

### Steering table

Maps steering input to per-wheel inner/outer road-wheel angles (Ackermann geometry) as
a lookup table with linear interpolation. Entries are indexed `left.N` / `right.N`.

| Key | Unit | Default | Effect |
| --- | --- | --- | --- |
| `steeringTable.frame` | ISO8855 \| SAE | ISO8855 | Frame of the tabulated angles. |
| `steeringTable.unit` | deg/rad | (per convention) | Unit of the input/inner/outer angles. |
| `steeringTable.symmetry` | 0 \| 1 | – | Whether left/right are mirror-symmetric (reuse one side). |
| `steeringTable.outOfRangeBehaviour` | – | – | How inputs beyond the last table row are handled (clamp/extrapolate). |
| `steeringTable.{left,right}.N.input` | deg/rad | – | Steering input for row N. |
| `steeringTable.{left,right}.N.inner` | deg/rad | – | Inner-wheel angle at row N. |
| `steeringTable.{left,right}.N.outer` | deg/rad | – | Outer-wheel angle at row N. |

---

## Differential

| Key | Values | Default | Effect |
| --- | --- | --- | --- |
| `implementation` | Open | Open | Differential model. Only the open diff (equal wheel torque, weaker-wheel limited) is implemented. |

---

## Aero

Simple downforce/drag model; forces act at `claPosition`.

| Key | Unit | Default | Effect |
| --- | --- | --- | --- |
| `frame` | ISO8855 \| SAE | *required* | Aero reference frame. |
| `implementation` | Simple | *required* | Aero model (only `Simple`). |
| `cla` | – | *required* | Lift/downforce coefficient × area (ρ/2 applied internally). |
| `cda` | – | 0.0 | Drag coefficient × area. |
| `claPosition.{x,y,z}` | m | *required* | Center-of-pressure position. |

---

## Tire

`frame`, `implementation` and a calibration slip select the model; then either the
quadratic **Simple** params or the **Pacejka** coefficient set is read.

| Key | Values/Unit | Default | Effect |
| --- | --- | --- | --- |
| `frame` | ISO8855 \| SAE | *required* | Tire reference frame (Pacejka accounts for left/right asymmetry). |
| `implementation` | Simple \| PacejkaV1 \| PacejkaV2 | *required* | Tire model. `Vehicle` always builds the configured model. |
| `calibrationSlipAngle` (+ `.unit`) | deg/rad | 90 | Reference slip angle for peak/calibration. |

### Simple model (`implementation,Simple`)

Quadratic force vs load: `F = load · (linFac + quadFac · load) · scalingFac` (ignores slip angle).

| Key | Default | Effect |
| --- | --- | --- |
| `linFac` | *required* | Linear grip term. |
| `quadFac` | *required* | Load-sensitivity (quadratic) term. |
| `scalingFac` | *required* | Overall grip scale. |

### Pacejka model (`PacejkaV1` / `PacejkaV2`)

The standard Magic-Formula (MF/Pacejka 2002) coefficient set, read by name from the
`Tire` section into a parameter map (`src/vehicle/tire/tirePacejka*.inl`). Not enumerated
individually here — they are the conventional MF coefficients:

- **Longitudinal Fx**: `PCX1, PDX1..3, PEX1..4, PKX1..3, PHX1..2, PVX1..2, RBX*, RCX1, REX*, RHX1`.
- **Lateral Fy**: `PCY1, PDY1..3, PEY1..4, PKY1..3, PHY1..3, PVY1..4`.
- **Aligning Mz**: `QBZ*, QCZ1, QDZ*, QEZ*, QHZ*`.
- **Scaling factors** `L*` (e.g. `LMUX, LMUY, LKY, LCY, LEY, LGAY, …`): per-effect multipliers to tune/scale the base curves.
- **Normalization**: `FNOMIN` (nominal load), `R0` (unloaded radius).

For the physics of how each family enters the force/moment equations see
`docs/codebase_analysis.md`.

---

## Sweep

The yaw-moment-diagram grid: operating speed and the steering/slip ranges & resolution.
Consumed in `getYawMomentDiagramPoints`.

| Key | Unit | Default | Effect |
| --- | --- | --- | --- |
| `speed` | m/s | *required* | Constant vehicle speed for the whole diagram. |
| `maxSteeringAngle` | deg | *required* | Steering swept over ±this range. |
| `steeringAngleStep` | deg | *required* | Steering grid resolution. |
| `maxSlipAngle` | deg | *required* | Chassis slip swept over ±this range. |
| `slipAngleStep` | deg | *required* | Slip grid resolution. |

---

## Solver

Per-point equilibrium solver settings.

| Key | Unit | Default | Effect |
| --- | --- | --- | --- |
| `tolerance` | – | *required* | Convergence tolerance for the lateral-acc/equilibrium solve. |
| `maxIterations` | – | *required* | Iteration cap per point. |
| `latAccBracketG` | g | 4 | Half-width (in g) of the lateral-acc search bracket. |

---

## Longitudinal

Optional coupled longitudinal/lateral solve. When enabled the solver finds the slip
ratios that meet `targetAcc` while balancing lateral equilibrium.

| Key | Values | Default | Effect |
| --- | --- | --- | --- |
| `equilibrium` | true \| false | false | Enable the coupled longitudinal solve. |
| `targetAcc` | m/s² | 0.0 | Target path longitudinal acceleration (reported as `longAcc`). |

---

## Refine

Adaptive subdivision of the base diagram: sparse regions (large gaps between adjacent
points) are bisected up to a depth cap, densifying the diagram where it changes fastest.

| Key | Values | Default | Effect |
| --- | --- | --- | --- |
| `enabled` | true \| false | true | Turn adaptive refinement on/off. |
| `factor` | – | 1.5 | Gap target = `factor × median gap`; smaller ⇒ denser. |
| `maxDepth` | – | 4 | Max bisection depth per interval. |

---

## YawZero

Trim-locus mode: instead of the full 2D diagram, find the curve where **yawMoment = 0**
(the trimmed operating locus) by a line-scan. Enable with `enabled,true`. Output is the
trim points in `yaw_diagram.csv`; plot with `tools/plot_yaw_zero_trim.py`
(or `make trim CONFIG=...`).

| Key | Unit | Default | Effect |
| --- | --- | --- | --- |
| `enabled` | true \| false | false | Switch from the full diagram to yaw-zero trim tracing. |
| `scanStep` | deg | 2 | Spacing of the steering lines and of the slip bracket scan along each. Coarser ⇒ fewer solves; too coarse can step over two close crossings near a fold. |
| `rootFinder` | anderson \| bisect \| illinois \| ridders \| brent \| itp | anderson | Bracketed root-finder that pins each crossing. |
| `slipTolerance` | deg | 0.01 | Root tolerance on the solved (slip) axis. |
| `maxBisect` | – | 40 | Max root-finder iterations per crossing. |
| `residualTolerance` | N·m | 50 | A crossing is a genuine trim only if `|yawMoment|` converges below this; larger residuals (front saturated) are rejected. |
| `mergeRadius` | – | 0.02 | Normalized distance under which points are treated as duplicates (dedup). |

For each steering line (fixed steering, spaced by `scanStep`), chassis slip is stepped
across its range and every yawMoment sign change is pinned by the root-finder. A steering
fold shows up as two slip crossings on the lines just inside it, so folds are captured
without walking the curve. The steering lines are independent, so the scan runs in
parallel across the core pool and the deduped output is thread-count independent.
