import math

import pytest

from payload_bolts import InterfaceLoad, circular_pattern, distribute_loads


def test_pure_mz_tangential_directions():
    # For a bolt on the +x axis (x=R, y=0), a positive (CCW) Mz should
    # produce a tangential force in +y: Vx_torsion = -Mz*y/J = 0,
    # Vy_torsion = Mz*x/J > 0.
    p = circular_pattern(4, 0.5, angular_offset=0.0)  # bolt 0 at (R, 0)
    result = distribute_loads(p, InterfaceLoad(Mz=1000.0))
    b0 = result.bolts[0]
    assert b0.x == pytest.approx(0.5)
    assert b0.y == pytest.approx(0.0, abs=1e-9)
    assert b0.Vx_torsion == pytest.approx(0.0, abs=1e-9)
    assert b0.Vy_torsion > 0


def test_pure_mz_equal_magnitudes_for_circular_pattern():
    p = circular_pattern(8, 0.5)
    result = distribute_loads(p, InterfaceLoad(Mz=5000.0))
    mags = [b.shear_resultant for b in result.bolts]
    for m in mags:
        assert m == pytest.approx(mags[0], rel=1e-9)


def test_pure_mz_analytical_magnitude_Mz_over_NR():
    n, R, Mz = 6, 0.4, 3000.0
    p = circular_pattern(n, R)
    result = distribute_loads(p, InterfaceLoad(Mz=Mz))
    expected = abs(Mz) / (n * R)
    for b in result.bolts:
        assert b.shear_resultant == pytest.approx(expected, rel=1e-9)


def test_pure_mz_recovered_exact():
    p = circular_pattern(9, 0.35, angular_offset=0.2)
    load = InterfaceLoad(Mz=7654.0)
    result = distribute_loads(p, load)
    assert result.equilibrium.Mz_recovered == pytest.approx(load.Mz, rel=1e-9)
    assert result.equilibrium.Mz_residual == pytest.approx(0.0, abs=1e-6)


def test_pure_mz_zero_net_force():
    p = circular_pattern(10, 0.6, angular_offset=1.1)
    result = distribute_loads(p, InterfaceLoad(Mz=8888.0))
    assert result.equilibrium.Fx_recovered == pytest.approx(0.0, abs=1e-6)
    assert result.equilibrium.Fy_recovered == pytest.approx(0.0, abs=1e-6)


def test_combined_direct_shear_and_mz_equilibrium():
    p = circular_pattern(8, 0.5)
    load = InterfaceLoad(Fx=20000.0, Fy=-10000.0, Mz=12000.0)
    result = distribute_loads(p, load)
    eq = result.equilibrium
    assert eq.Fx_recovered == pytest.approx(load.Fx, abs=1e-6)
    assert eq.Fy_recovered == pytest.approx(load.Fy, abs=1e-6)
    assert eq.Mz_recovered == pytest.approx(load.Mz, abs=1e-6)
    assert eq.max_abs_residual < 1e-6
