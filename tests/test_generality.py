import pytest

from payload_bolts import BoltPattern, InterfaceLoad, circular_pattern, distribute_loads
from payload_bolts.solver import check_equilibrium


def _translate(pattern: BoltPattern, dx: float, dy: float) -> BoltPattern:
    return BoltPattern([(x + dx, y + dy) for x, y in pattern.coordinates])


def test_translated_pattern_same_relative_bolt_forces():
    p1 = circular_pattern(6, 0.45, angular_offset=0.2)
    p2 = _translate(p1, 250.0, -600.0)

    load = InterfaceLoad(Fx=5000.0, Fy=-2500.0, Fz=30000.0, Mx=4000.0, My=-3000.0, Mz=6000.0)

    r1 = distribute_loads(p1, load)
    r2 = distribute_loads(p2, load)

    for b1, b2 in zip(r1.bolts, r2.bolts):
        assert b1.Vx_total == pytest.approx(b2.Vx_total, rel=1e-9, abs=1e-9)
        assert b1.Vy_total == pytest.approx(b2.Vy_total, rel=1e-9, abs=1e-9)
        assert b1.axial_total == pytest.approx(b2.axial_total, rel=1e-9, abs=1e-9)


def test_irregular_nondegenerate_pattern_satisfies_equilibrium():
    coords = [(0.1, 0.05), (0.6, -0.2), (0.3, 0.5), (-0.4, 0.35), (-0.55, -0.15), (0.05, -0.6)]
    p = BoltPattern(coords)
    load = InterfaceLoad(Fx=8000.0, Fy=-3000.0, Fz=45000.0, Mx=6000.0, My=-9000.0, Mz=4000.0)
    result = distribute_loads(p, load)
    assert result.equilibrium.max_abs_residual < 1e-6


def test_degenerate_axial_moment_geometry_rejected_cleanly():
    # All bolts collinear on the x-axis (through the centroid): the
    # coefficient system for cy is singular because Ix_group and Ixy
    # about the centroid vanish in a way that Mx cannot be resolved
    # independent of My given zero y-spread. Use a truly singular case:
    # all bolts at the same y (a line), so cy is unconstrained /
    # over-constrained depending on Mx.
    coords = [(-1.0, 0.0), (0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    p = BoltPattern(coords)
    with pytest.raises(ValueError):
        distribute_loads(p, InterfaceLoad(Mx=100.0))


def test_duplicate_coordinates_handled_per_policy():
    with pytest.raises(ValueError):
        BoltPattern([(0.0, 0.0), (0.0, 0.0), (1.0, 1.0), (1.0, -1.0)])

    p = BoltPattern([(0.0, 0.0), (0.0, 0.0), (1.0, 1.0), (1.0, -1.0)], allow_duplicates=True)
    assert p.n_bolts == 4


def test_non_finite_load_rejected():
    with pytest.raises(ValueError):
        InterfaceLoad(Fx=float("nan"))
    with pytest.raises(ValueError):
        InterfaceLoad(Mz=float("inf"))


def test_all_zero_load_gives_zero_bolt_loads():
    p = circular_pattern(6, 0.5)
    result = distribute_loads(p, InterfaceLoad())
    for b in result.bolts:
        assert b.Vx_total == 0.0
        assert b.Vy_total == 0.0
        assert b.axial_total == 0.0
    assert result.equilibrium.max_abs_residual == 0.0


def test_repeated_solve_deterministic():
    p = circular_pattern(8, 0.5)
    load = InterfaceLoad(Fx=1000.0, Fy=2000.0, Fz=3000.0, Mx=400.0, My=-500.0, Mz=600.0)
    r1 = distribute_loads(p, load)
    r2 = distribute_loads(p, load)
    for b1, b2 in zip(r1.bolts, r2.bolts):
        assert b1 == b2


def test_equilibrium_check_is_independent_recomputation():
    # check_equilibrium can be called directly on a bolt result tuple,
    # independent of distribute_loads' internal solve path.
    p = circular_pattern(5, 0.4)
    load = InterfaceLoad(Fx=100.0, Mz=50.0)
    result = distribute_loads(p, load)
    eq2 = check_equilibrium(p, load, result.bolts)
    assert eq2 == result.equilibrium
