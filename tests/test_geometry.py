import pytest

from payload_bolts import BoltPattern, circular_pattern, rectangular_pattern


def test_circular_pattern_coordinate_count():
    p = circular_pattern(6, 1.0)
    assert p.n_bolts == 6
    assert len(p.coordinates) == 6


def test_circular_pattern_centroid_at_origin():
    p = circular_pattern(8, 0.5)
    cx, cy = p.centroid
    assert cx == pytest.approx(0.0, abs=1e-9)
    assert cy == pytest.approx(0.0, abs=1e-9)


def test_circular_pattern_all_radii_equal_R():
    R = 0.65
    p = circular_pattern(5, R)
    for r in p.radii:
        assert r == pytest.approx(R, rel=1e-9)


def test_circular_pattern_J_equals_N_R_squared():
    n, R = 10, 0.4
    p = circular_pattern(n, R)
    assert p.J == pytest.approx(n * R * R, rel=1e-9)


def test_circular_pattern_Ix_Iy_analytical():
    # For N equally spaced bolts on a circle, symmetry gives
    # Ix_group == Iy_group == J/2 for N >= 3 (and exactly for N=2 as well
    # when the two bolts are diametrically opposed, since y^2 and x^2 mixing
    # still holds by the same identity for antipodal points -- verify generally).
    n, R = 12, 0.5
    p = circular_pattern(n, R)
    assert p.Ix_group == pytest.approx(p.J / 2.0, rel=1e-9)
    assert p.Iy_group == pytest.approx(p.J / 2.0, rel=1e-9)


def test_angular_offset_rotation_preserves_group_inertias():
    n, R = 8, 0.5
    p1 = circular_pattern(n, R, angular_offset=0.0)
    p2 = circular_pattern(n, R, angular_offset=0.37)
    assert p2.J == pytest.approx(p1.J, rel=1e-9)
    assert (p1.Ix_group + p1.Iy_group) == pytest.approx(p2.Ix_group + p2.Iy_group, rel=1e-9)


def test_invalid_pattern_too_few_bolts():
    with pytest.raises(ValueError):
        BoltPattern([(0.0, 0.0)])


def test_invalid_pattern_all_coincident():
    with pytest.raises(ValueError):
        BoltPattern([(1.0, 1.0), (1.0, 1.0), (1.0, 1.0)], allow_duplicates=True)


def test_invalid_pattern_non_finite():
    with pytest.raises(ValueError):
        BoltPattern([(0.0, 0.0), (float("nan"), 1.0)])


def test_invalid_pattern_duplicates_rejected_by_default():
    with pytest.raises(ValueError):
        BoltPattern([(0.0, 0.0), (0.0, 0.0), (1.0, 1.0)])


def test_invalid_pattern_duplicates_allowed_when_flagged():
    p = BoltPattern([(0.0, 0.0), (0.0, 0.0), (1.0, 1.0)], allow_duplicates=True)
    assert p.n_bolts == 3


def test_invalid_circular_pattern_bad_inputs():
    with pytest.raises(ValueError):
        circular_pattern(1, 1.0)
    with pytest.raises(ValueError):
        circular_pattern(4, -1.0)
    with pytest.raises(ValueError):
        circular_pattern(4, float("inf"))
    with pytest.raises(ValueError):
        circular_pattern(4.5, 1.0)  # not an int


def test_invalid_rectangular_pattern_bad_inputs():
    with pytest.raises(ValueError):
        rectangular_pattern(0.0, 1.0)
    with pytest.raises(ValueError):
        rectangular_pattern(1.0, -1.0)


def test_rectangular_pattern_corners():
    p = rectangular_pattern(2.0, 4.0)
    assert p.n_bolts == 4
    xs = sorted(x for x, _ in p.coordinates)
    ys = sorted(y for _, y in p.coordinates)
    assert xs == pytest.approx([-1.0, -1.0, 1.0, 1.0])
    assert ys == pytest.approx([-2.0, -2.0, 2.0, 2.0])


def test_arbitrary_coordinate_centroid():
    coords = [(1.0, 1.0), (3.0, 1.0), (3.0, 3.0), (1.0, 3.0)]
    p = BoltPattern(coords)
    cx, cy = p.centroid
    assert cx == pytest.approx(2.0)
    assert cy == pytest.approx(2.0)


def test_coordinate_translation_invariance_of_group_properties():
    coords = [(1.0, 1.0), (3.0, 1.0), (3.0, 3.0), (1.0, 3.0)]
    shift = (100.0, -50.0)
    p1 = BoltPattern(coords)
    p2 = BoltPattern([(x + shift[0], y + shift[1]) for x, y in coords])

    assert p1.J == pytest.approx(p2.J, rel=1e-9)
    assert p1.Ix_group == pytest.approx(p2.Ix_group, rel=1e-9)
    assert p1.Iy_group == pytest.approx(p2.Iy_group, rel=1e-9)
    assert p1.Ixy_group == pytest.approx(p2.Ixy_group, rel=1e-9)
    assert sorted(p1.radii) == pytest.approx(sorted(p2.radii), rel=1e-9)
