import pytest

from payload_bolts import InterfaceLoad, circular_pattern, distribute_loads


def test_pure_fx_equal_split():
    p = circular_pattern(6, 0.5)
    result = distribute_loads(p, InterfaceLoad(Fx=6000.0))
    for b in result.bolts:
        assert b.Vx_direct == pytest.approx(1000.0)
        assert b.Vy_direct == pytest.approx(0.0)
        assert b.Vx_total == pytest.approx(1000.0)
        assert b.Vy_total == pytest.approx(0.0)


def test_pure_fy_equal_split():
    p = circular_pattern(4, 0.3)
    result = distribute_loads(p, InterfaceLoad(Fy=4000.0))
    for b in result.bolts:
        assert b.Vy_direct == pytest.approx(1000.0)
        assert b.Vx_direct == pytest.approx(0.0)


def test_combined_fx_fy_vector_split():
    p = circular_pattern(5, 0.4)
    result = distribute_loads(p, InterfaceLoad(Fx=1000.0, Fy=-2000.0))
    for b in result.bolts:
        assert b.Vx_direct == pytest.approx(200.0)
        assert b.Vy_direct == pytest.approx(-400.0)


def test_direct_shear_force_equilibrium_exact():
    p = circular_pattern(7, 0.55)
    load = InterfaceLoad(Fx=12345.0, Fy=-6789.0)
    result = distribute_loads(p, load)
    eq = result.equilibrium
    assert eq.Fx_recovered == pytest.approx(load.Fx, abs=1e-6)
    assert eq.Fy_recovered == pytest.approx(load.Fy, abs=1e-6)
    assert eq.max_abs_residual < 1e-6
