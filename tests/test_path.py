"""Acceptance tests for the camera-path module.

These tests exercise the public contract of ``campath.path``.  They are the
fixed specification for the bug-fix task and must not be modified.
"""

import copy
import math

import pytest

from campath.path import (
    CameraPathError,
    arc_length,
    catmull_rom,
    look_at,
    path_batch,
    resample_uniform,
    speed_profile,
    tangent_at,
    validate,
)


def _length(vec):
    return math.sqrt(sum(component * component for component in vec))


# ---------------------------------------------------------------------------
# One test per reported defect (10)
# ---------------------------------------------------------------------------


def test_endpoint_tangents_are_nonzero():
    """Defect 1: zero endpoint tangents make the camera stop at both ends."""
    points = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 2.0, 0.0)]
    samples = catmull_rom(points, 7)

    first = tangent_at(samples, 0)
    last = tangent_at(samples, len(samples) - 1)

    assert _length(first) == pytest.approx(1.0, abs=1e-6)
    assert _length(last) == pytest.approx(1.0, abs=1e-6)
    assert max(abs(component) for component in first) > 1e-3
    assert max(abs(component) for component in last) > 1e-3


def test_tangents_are_unit_length_on_uneven_samples():
    """Defect 2: unnormalized tangents cause orientation jitter."""
    samples = [
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (3.0, 0.0, 0.0),
        (3.0, 4.0, 0.0),
        (3.0, 4.0, 5.0),
    ]

    for index in range(len(samples)):
        tangent = tangent_at(samples, index)
        assert len(tangent) == 3
        assert _length(tangent) == pytest.approx(1.0, abs=1e-6)


def test_look_at_basis_is_not_flipped():
    """Defect 3: reversed cross-product order flips the camera orientation."""
    result = look_at((1.0, 2.0, 3.0), (1.0, 2.0, 4.0), (0.0, 1.0, 0.0))

    assert isinstance(result, dict)
    assert tuple(result["forward"]) == pytest.approx((0.0, 0.0, 1.0), abs=1e-6)
    assert tuple(result["right"]) == pytest.approx((1.0, 0.0, 0.0), abs=1e-6)
    assert tuple(result["up"]) == pytest.approx((0.0, 1.0, 0.0), abs=1e-6)

    expected_matrix = [
        [1.0, 0.0, 0.0, -1.0],
        [0.0, 1.0, 0.0, -2.0],
        [0.0, 0.0, 1.0, -3.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
    assert len(result["matrix"]) == 4
    for row, expected_row in zip(result["matrix"], expected_matrix):
        assert len(row) == 4
        assert tuple(row) == pytest.approx(expected_row, abs=1e-6)


def test_arc_length_of_curved_path_is_not_shortened():
    """Defect 4: straight-line accumulation underestimates a curved path."""
    segments = 512
    samples = [
        (
            math.cos(0.5 * math.pi * i / segments),
            math.sin(0.5 * math.pi * i / segments),
            0.0,
        )
        for i in range(segments + 1)
    ]

    lengths = arc_length(samples)

    assert lengths[0] == pytest.approx(0.0, abs=1e-12)
    # Quarter circle of radius 1: the arc length is pi / 2, not sqrt(2).
    assert lengths[-1] == pytest.approx(0.5 * math.pi, abs=1e-4)
    assert all(
        later > earlier for earlier, later in zip(lengths, lengths[1:])
    )


def test_resample_uniform_spaces_by_arc_length():
    """Defect 5: parameter-based splitting produces uneven spacing."""
    samples = [
        (0.0, 0.0, 0.0),
        (0.05, 0.0, 0.0),
        (0.10, 0.0, 0.0),
        (0.20, 0.0, 0.0),
        (1.00, 0.0, 0.0),
    ]

    output = resample_uniform(samples, 5)

    assert len(output) == 5
    assert tuple(output[0]) == pytest.approx(samples[0], abs=1e-9)
    assert tuple(output[-1]) == pytest.approx(samples[-1], abs=1e-9)
    for start, end in zip(output, output[1:]):
        assert math.dist(start, end) == pytest.approx(0.25, abs=1e-6)


def test_speed_profile_decelerates_at_endpoints():
    """Defect 6: no endpoint easing makes starts and stops abrupt."""
    samples = [(float(index), 0.0, 0.0) for index in range(9)]

    profile = speed_profile(samples, "smooth")

    assert len(profile) == len(samples)
    assert profile[0] == pytest.approx(0.0, abs=1e-6)
    assert profile[-1] == pytest.approx(0.0, abs=1e-6)
    assert all(0.0 <= value <= 1.0 for value in profile)
    assert max(profile) == pytest.approx(1.0, abs=1e-6)
    assert profile == pytest.approx(list(reversed(profile)), abs=1e-6)


def test_look_at_rejects_parallel_up():
    """Defect 7: a parallel up hint silently creates a singular matrix."""
    with pytest.raises(CameraPathError):
        look_at((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), (0.0, 0.0, 1.0))


def test_duplicate_control_points_are_rejected_not_nan():
    """Defect 8: duplicate points must be reported, not produce NaN tangents."""
    points = [
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (2.0, 0.0, 0.0),
    ]

    problems = validate(points, [])
    assert isinstance(problems, list)
    assert any("duplicate" in problem.lower() for problem in problems)

    with pytest.raises(CameraPathError):
        catmull_rom(points, 8)


def test_batch_buffers_are_isolated_and_rollback_is_atomic():
    """Defect 9: a shared batch buffer leaks data between paths."""
    paths = [
        {"name": "east", "points": [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0)]},
        {"name": "north", "points": [(0.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 2.0, 0.0)]},
    ]
    snapshot = copy.deepcopy(paths)

    result = path_batch(paths, {"samples": 9})
    assert paths == snapshot

    batch = result["paths"]
    assert [item["name"] for item in batch] == ["east", "north"]
    east_samples = batch[0]["samples"]
    north_samples = batch[1]["samples"]
    assert east_samples is not north_samples
    assert tuple(east_samples[-1]) == pytest.approx((2.0, 0.0, 0.0), abs=1e-6)
    assert tuple(north_samples[-1]) == pytest.approx((0.0, 2.0, 0.0), abs=1e-6)

    east_samples[0] = (99.0, 99.0, 99.0)
    assert tuple(north_samples[0]) == pytest.approx((0.0, 0.0, 0.0), abs=1e-6)

    bad_paths = [
        {"name": "good", "points": [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]},
        {"name": "bad", "points": [(0.0, 0.0, 0.0), (0.0, 0.0, 0.0)]},
    ]
    bad_snapshot = copy.deepcopy(bad_paths)
    with pytest.raises(CameraPathError):
        path_batch(bad_paths, {"samples": 9})
    assert bad_paths == bad_snapshot


def test_resample_count_below_two_is_rejected():
    """Defect 10: count < 2 must raise instead of returning an empty list."""
    samples = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]

    for bad_count in (-1, 0, 1):
        with pytest.raises(CameraPathError):
            resample_uniform(samples, bad_count)


# ---------------------------------------------------------------------------
# Combined-behaviour tests (5)
# ---------------------------------------------------------------------------


def test_two_point_resample_has_equal_spacing():
    """A straight two-point path resamples to equal spacings within 1e-6."""
    output = resample_uniform([(0.0, 0.0, 0.0), (10.0, 0.0, 0.0)], 6)

    assert len(output) == 6
    assert tuple(output[0]) == pytest.approx((0.0, 0.0, 0.0), abs=1e-9)
    assert tuple(output[-1]) == pytest.approx((10.0, 0.0, 0.0), abs=1e-9)
    for start, end in zip(output, output[1:]):
        assert math.dist(start, end) == pytest.approx(2.0, abs=1e-6)


def test_resample_count_two_returns_endpoints():
    """count == 2 returns exactly the first and last samples."""
    samples = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0)]

    output = resample_uniform(samples, 2)

    assert len(output) == 2
    assert tuple(output[0]) == pytest.approx(samples[0], abs=1e-9)
    assert tuple(output[1]) == pytest.approx(samples[-1], abs=1e-9)


def test_single_point_path_raises():
    """A path with one control point is invalid and raises CameraPathError."""
    with pytest.raises(CameraPathError):
        catmull_rom([(1.0, 2.0, 3.0)], 5)


def test_arc_length_is_strictly_increasing():
    """Every non-degenerate segment adds positive arc length."""
    samples = [
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (1.0, 3.0, 0.0),
        (1.0, 3.0, 4.0),
    ]

    lengths = arc_length(samples)

    assert lengths == pytest.approx([0.0, 1.0, 4.0, 8.0], abs=1e-6)
    assert all(later > earlier for earlier, later in zip(lengths, lengths[1:]))


def test_batch_preserves_order_on_success():
    """Successful batches keep the input path order."""
    paths = [
        {"name": "first", "points": [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]},
        {"name": "second", "points": [(0.0, 0.0, 0.0), (0.0, 1.0, 0.0)]},
        {"name": "third", "points": [(0.0, 0.0, 0.0), (0.0, 0.0, 1.0)]},
    ]

    result = path_batch(paths, {"samples": 5})

    assert result["count"] == 3
    assert [item["name"] for item in result["paths"]] == [
        "first",
        "second",
        "third",
    ]
    assert all(len(item["samples"]) == 5 for item in result["paths"])
