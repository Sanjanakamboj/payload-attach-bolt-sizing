import pytest

from payload_bolts import InterfaceLoad, circular_pattern, distribute_loads


def test_pure_fz_equal_split():
    p = circular_pattern(8, 0.5)
    result = distribute_loads(p, InterfaceLoad(Fz=8000.0))
    for b in result.bolts:
        assert b.axial_direct == pytest.approx(1000.0)
        assert b.axial_total == pytest.approx(1000.0)


def test_pure_mx_proportional_to_y():
    p = circular_pattern(8, 0.5, angular_offset=0.3)
    result = distribute_loads(p, InterfaceLoad(Mx=1500.0))
    ys = [b.y for b in result.bolts]
    Ts = [b.axial_total for b in result.bolts]
    # T_i = cy * y_i (centered coords == original here since centroid at origin)
    ratios = [T / y for T, y in zip(Ts, ys) if abs(y) > 1e-9]
    for r in ratios:
        assert r == pytest.approx(ratios[0], rel=1e-6)


def test_pure_my_proportional_to_minus_x():
    p = circular_pattern(8, 0.5, angular_offset=0.3)
    result = distribute_loads(p, InterfaceLoad(My=1500.0))
    xs = [b.x for b in result.bolts]
    Ts = [b.axial_total for b in result.bolts]
    ratios = [T / (-x) for T, x in zip(Ts, xs) if abs(x) > 1e-9]
    for r in ratios:
        assert r == pytest.approx(ratios[0], rel=1e-6)


def test_pure_moment_gives_zero_net_axial_force():
    p = circular_pattern(6, 0.4)
    for load in (InterfaceLoad(Mx=1000.0), InterfaceLoad(My=-800.0)):
        result = distribute_loads(p, load)
        assert sum(b.axial_total for b in result.bolts) == pytest.approx(0.0, abs=1e-6)


def test_recovered_mx_exact():
    p = circular_pattern(8, 0.5, angular_offset=0.15)
    load = InterfaceLoad(Mx=25000.0)
    result = distribute_loads(p, load)
    assert result.equilibrium.Mx_recovered == pytest.approx(load.Mx, rel=1e-9)


def test_recovered_my_exact():
    p = circular_pattern(8, 0.5, angular_offset=0.15)
    load = InterfaceLoad(My=-15000.0)
    result = distribute_loads(p, load)
    assert result.equilibrium.My_recovered == pytest.approx(load.My, rel=1e-9)


def test_combined_fz_mx_my_equilibrium():
    p = circular_pattern(8, 0.5)
    load = InterfaceLoad(Fz=80000.0, Mx=25000.0, My=-15000.0)
    result = distribute_loads(p, load)
    eq = result.equilibrium
    assert eq.Fz_recovered == pytest.approx(load.Fz, abs=1e-6)
    assert eq.Mx_recovered == pytest.approx(load.Mx, abs=1e-6)
    assert eq.My_recovered == pytest.approx(load.My, abs=1e-6)
    assert eq.max_abs_residual < 1e-6


def test_signed_compression_side_loads_retained_not_clipped():
    p = circular_pattern(8, 0.5)
    load = InterfaceLoad(Fz=1000.0, Mx=50000.0, My=0.0)
    result = distribute_loads(p, load)
    axial_values = [b.axial_total for b in result.bolts]
    assert any(v < 0 for v in axial_values), "expected some compression-side (negative) bolts"
    assert any(v > 0 for v in axial_values)


def test_deterministic_governing_tensile_bolt():
    p = circular_pattern(8, 0.5)
    load = InterfaceLoad(Fz=80000.0, Mx=25000.0, My=-15000.0)
    result = distribute_loads(p, load)
    winner = result.max_tensile_bolt()
    assert winner.axial_total == max(b.axial_total for b in result.bolts)

    # tie -> lowest index: two bolts with identical axial_total
    p2 = circular_pattern(4, 0.5)
    result2 = distribute_loads(p2, InterfaceLoad(Fz=4000.0))  # all four tied
    assert result2.max_tensile_bolt().index == 0


def test_deterministic_governing_absolute_axial_bolt():
    p = circular_pattern(4, 0.5)
    result = distribute_loads(p, InterfaceLoad(Fz=4000.0))  # all tied
    assert result.max_abs_axial_bolt().index == 0
