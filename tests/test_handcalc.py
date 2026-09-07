"""Independent hand-calculation check.

Four bolts at (+a,0), (0,+a), (-a,0), (0,-a) -- a "cross" pattern
centered at the origin. Expected per-bolt loads below are derived by
hand from first principles (not by calling any internal solver helper)
and only compared against payload_bolts.distribute_loads output.

Geometry:
    bolt 0: ( a,  0)
    bolt 1: ( 0,  a)
    bolt 2: (-a,  0)
    bolt 3: ( 0, -a)

    J = sum(r_i^2) = 4*a^2
    Ix_group = sum(y_i^2) = a^2 (only bolts 1,3 contribute a^2 each) = 2*a^2
    Iy_group = sum(x_i^2) = 2*a^2  (bolts 0,2 contribute a^2 each)

Case 1: pure Mz = M.
    Torsional shear: Vx_i = -M*y_i/J, Vy_i = M*x_i/J
    bolt 0 (a,0):  Vx=0,        Vy=M*a/J = M/(4a)
    bolt 1 (0,a):  Vx=-M*a/J = -M/(4a), Vy=0
    bolt 2 (-a,0): Vx=0,        Vy=-M/(4a)
    bolt 3 (0,-a): Vx=M/(4a),   Vy=0
    Each bolt shear magnitude = M/(4a) = M/(J/a) ... check against
    general formula |V_i| = |Mz|/(N R) with N=4, R=a: M/(4a). Matches.

Case 2: pure Mx = Mx0.
    T_i = c0 + cx*x_i + cy*y_i, with c0=0 (Fz=0).
    sum(T_i)=0, sum(y_i*T_i)=Mx0, sum(-x_i*T_i)=0 (My=0)
    By symmetry (Ixy_group = sum(x_i*y_i) = 0 for this cross pattern),
    the coefficient system decouples: cx*Iy_group = -My = 0 => cx=0
    cy*Ix_group = Mx0 => cy = Mx0 / (2*a^2)
    T_0 (a,0)  = cy*0 = 0
    T_1 (0,a)  = cy*a = Mx0/(2a)
    T_2 (-a,0) = 0
    T_3 (0,-a) = -Mx0/(2a)

Case 3: pure My = My0.
    cy=0 (Mx=0), cx*Iy_group = -My0 => cx = -My0/(2*a^2)
    T_0 (a,0)  = cx*a = -My0/(2a)
    T_1 (0,a)  = 0
    T_2 (-a,0) = -cx*a = My0/(2a)
    T_3 (0,-a) = 0
"""

import pytest

from payload_bolts import BoltPattern, InterfaceLoad, distribute_loads

A = 0.3  # arm length, meters


def _cross_pattern():
    return BoltPattern([(A, 0.0), (0.0, A), (-A, 0.0), (0.0, -A)])


def test_handcalc_pure_mz():
    M = 400.0
    p = _cross_pattern()
    result = distribute_loads(p, InterfaceLoad(Mz=M))
    expected_Vx = [0.0, -M / (4 * A), 0.0, M / (4 * A)]
    expected_Vy = [M / (4 * A), 0.0, -M / (4 * A), 0.0]
    for b, evx, evy in zip(result.bolts, expected_Vx, expected_Vy):
        assert b.Vx_total == pytest.approx(evx, abs=1e-9)
        assert b.Vy_total == pytest.approx(evy, abs=1e-9)


def test_handcalc_pure_mx():
    Mx0 = 250.0
    p = _cross_pattern()
    result = distribute_loads(p, InterfaceLoad(Mx=Mx0))
    expected_T = [0.0, Mx0 / (2 * A), 0.0, -Mx0 / (2 * A)]
    for b, eT in zip(result.bolts, expected_T):
        assert b.axial_total == pytest.approx(eT, abs=1e-9)


def test_handcalc_pure_my():
    My0 = -180.0
    p = _cross_pattern()
    result = distribute_loads(p, InterfaceLoad(My=My0))
    expected_T = [-My0 / (2 * A), 0.0, My0 / (2 * A), 0.0]
    for b, eT in zip(result.bolts, expected_T):
        assert b.axial_total == pytest.approx(eT, abs=1e-9)
