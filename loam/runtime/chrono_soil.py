"""Roller compaction from Project Chrono's SCM soil model.

`loam soil-calibrate` rolls a rigid drum of the roller's mass and size back
and forth over Chrono's SCM deformable terrain (Bekker-Wong pressure-sinkage,
Mohr-Coulomb and Janosi-Hanamoto shear, plastic memory). It records the
settlement after each pass and writes the curve as JSON. `loam simulate-site
--soil-table` hands the curve to the simulated world. There, a compaction order
of k passes settles loose fill by the curve's k-th value, capped at the lift's
full compaction. So a light roller, or too few passes, leaves the fill short
of grade, and the closed loop has to notice.

The calibration needs pychrono (BSD-3, from the projectchrono conda channel).
Running a site from a saved table does not.

Checks (`tests/test_chrono.py`, at the default sandy-loam parameters):

* A towed 10 t drum, 2.1 m wide and 1.5 m in diameter, sinks 1.25-1.31x the
  Bekker-Wong closed form for a rigid wheel. The ratio holds over 2x the load,
  half the width and twice the mesh resolution (mesh-converged within 1 %).
  The closed form is itself an approximation for towed wheels.
* Settlement per pass falls with each pass (76, 25, 17, 13, 10, 9 mm for 10 t),
  as field compaction curves do.

The default soil parameters are illustrative. For a real site, measure them
with a plate-sinkage or bevameter test.
"""
from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass


# Illustrative sandy loam, SI units. Bekker: Kphi [Pa/m^n], Kc [Pa/m^(n-1)], n; Mohr-Coulomb: cohesion [Pa],
# friction [deg]; Janosi shear modulus [m]; SCM elastic stiffness [Pa/m] and damping [Pa s/m].
@dataclass(frozen=True)
class SoilParams:
    """The soil's parameters for Chrono's SCM model (see the comment above for each one's meaning and unit)."""
    Kphi: float = 1.515e6
    Kc: float = 5.27e3
    n: float = 0.7
    cohesion: float = 1.72e3
    friction_deg: float = 29.0
    janosi: float = 0.01
    elastic_k: float = 2e8
    damping_r: float = 3e4


SANDY_LOAM = SoilParams()


def bekker_wheel_sinkage(load_n: float, width_m: float, diameter_m: float, soil: SoilParams) -> float:
    """Bekker-Wong closed-form static sinkage of a rigid wheel (m)."""
    s = soil
    return (3 * load_n / ((3 - s.n) * (s.Kc + width_m * s.Kphi) * math.sqrt(diameter_m))) ** (2 / (2 * s.n + 1))


def drum_passes(mass_kg: float, width_m: float, diameter_m: float, soil: SoilParams = SANDY_LOAM,
                passes: int = 6, length_m: float = 3.0, speed_ms: float = 1.0, delta_m: float = 0.04,
                dt: float = 5e-4) -> dict:
    """Roll a rigid drum `passes` times over the same strip of SCM soil. Returns the mean settlement of the
    strip after each pass (m) and the sinkage under the drum during the first pass (m)."""
    try:
        import pychrono as ch
        import pychrono.vehicle as veh
    except ImportError as e:
        raise ValueError("soil calibration needs pychrono: micromamba install -c projectchrono pychrono") from e
    s = ch.ChSystemSMC()
    s.SetCollisionSystemType(ch.ChCollisionSystem.Type_BULLET)
    r = diameter_m / 2

    def light(pos):                               # a massless-ish carrier for the joints
        """A light body (10 kg) at `pos`, used only to connect joints."""
        b = ch.ChBody()
        b.SetMass(10)
        b.SetInertiaXX(ch.ChVector3d(1, 1, 1))
        b.SetPos(pos)
        s.Add(b)
        return b
    # The fixed frame everything is measured from.
    gnd = ch.ChBody()
    gnd.SetFixed(True)
    s.Add(gnd)
    axle = ch.ChVector3d(0, 0, r)
    sled, hub = light(axle), light(axle)
    drum = ch.ChBodyEasyCylinder(ch.ChAxis_Y, r, width_m, mass_kg / (math.pi * r * r * width_m), False, True,
                                 ch.ChContactMaterialSMC())
    drum.SetPos(axle)
    s.Add(drum)
    # ground -(motor along x)- sled -(free vertical slide)- hub -(free axle)- drum: towed, free to sink
    motor = ch.ChLinkMotorLinearSpeed()
    motor.Initialize(sled, gnd, ch.ChFramed(axle, ch.QuatFromAngleY(math.pi / 2)))
    speed = ch.ChFunctionConst(0.0)
    motor.SetSpeedFunction(speed)
    s.Add(motor)
    slide = ch.ChLinkLockPrismatic()
    slide.Initialize(hub, sled, ch.ChFramed(axle, ch.QUNIT))
    s.Add(slide)
    rev = ch.ChLinkLockRevolute()
    rev.Initialize(drum, hub, ch.ChFramed(axle, ch.QuatFromAngleX(math.pi / 2)))
    s.Add(rev)
    terrain = veh.SCMTerrain(s)
    p = soil
    terrain.SetSoilParameters(p.Kphi, p.Kc, p.n, p.cohesion, p.friction_deg, p.janosi, p.elastic_k, p.damping_r)
    terrain.Initialize(length_m + 8, width_m + 2, delta_m)

    xs = [length_m * 0.3 + 0.1 * i for i in range(int(length_m * 0.4 / 0.1))]
    ys = (-width_m / 4, 0.0, width_m / 4)

    def strip() -> float:
        """Mean settlement (m, positive downward) over the middle of the rolled strip, sampled every 10 cm at three
        positions across the drum.
        """
        return -sum(terrain.GetHeight(ch.ChVector3d(x, y, 0)) for x in xs for y in ys) / (len(xs) * len(ys))

    ramp = 1.5                                    # load the soil quasi-statically: no drop, no overshoot
    while s.GetChTime() < ramp + 0.3:
        s.SetGravitationalAcceleration(ch.ChVector3d(0, 0, -9.81 * min(1.0, s.GetChTime() / ramp)))
        s.DoStepDynamics(dt)
    settle, rolling = [], []
    for k in range(passes):
        fwd = k % 2 == 0
        # Odd passes roll forward and even passes roll back over the same strip, as a roller works. After each pass
        # the drum stops for 0.3 s so the soil comes to rest before it is measured.
        speed.SetConstant(speed_ms if fwd else -speed_ms)
        while (drum.GetPos().x < length_m) if fwd else (drum.GetPos().x > 0.0):
            s.DoStepDynamics(dt)
            if k == 0 and length_m * 0.4 < drum.GetPos().x < length_m * 0.9:
                rolling.append(r - drum.GetPos().z)
        speed.SetConstant(0.0)
        for _ in range(int(0.3 / dt)):
            s.DoStepDynamics(dt)
        settle.append(strip())
    return {"settlement_m": settle, "first_pass_sinkage_m": sum(rolling) / len(rolling)}


def calibrate(mass_kg: float, width_m: float, diameter_m: float, soil: SoilParams = SANDY_LOAM,
              passes: int = 8) -> dict:
    """The table `simulate-site --soil-table` reads, for one roller."""
    run = drum_passes(mass_kg, width_m, diameter_m, soil, passes)
    load = mass_kg * 9.81
    return {"model": "Project Chrono SCM, towed rigid drum", "mass_kg": mass_kg, "width_m": width_m,
            "diameter_m": diameter_m, "soil": asdict(soil),
            "settlement_m": [round(v, 5) for v in run["settlement_m"]],
            "first_pass_sinkage_m": round(run["first_pass_sinkage_m"], 5),
            "bekker_wong_sinkage_m": round(bekker_wheel_sinkage(load, width_m, diameter_m, soil), 5)}


def settlement(table: dict, passes: int) -> float:
    """Settlement (m) after `passes` passes. Beyond the calibrated passes it stays at the last value: the
    table is not extrapolated, so calibrate at least as many passes as the orders use."""
    s = table["settlement_m"]
    return 0.0 if passes <= 0 else float(s[min(passes, len(s)) - 1])


def load_tables(path: str) -> dict[str, dict]:
    """Tables by machine name: `{"rl1": {...}, ...}`, or one table that applies to every roller."""
    with open(path, encoding="utf-8") as f:
        d = json.load(f)
    return {"*": d} if "settlement_m" in d else d
