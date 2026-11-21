"""Spline camera-path helpers for cutscene cameras.

Conventions used by every function in this module:

* Positions are world-space ``(x, y, z)`` tuples measured in world units.
* Time-like values (for example the output of :func:`speed_profile`) are in
  seconds or are explicitly normalized to ``0..1``.
* A tangent is a unit vector pointing in the direction of travel.
* An orientation is a right-handed orthonormal camera basis
  ``(right, up, forward)``.
* Numeric comparisons are expected to use a tolerance of ``1e-6``.

Every public function raises :class:`CameraPathError` for invalid or
degenerate input instead of returning NaN, empty, or silently skewed data.
"""

from __future__ import annotations

import math

TOLERANCE = 1e-6


class CameraPathError(ValueError):
    """Raised when camera-path input is invalid or geometrically degenerate."""


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------


def _as_point(point, what="point"):
    """Coerce ``point`` to a finite ``(x, y, z)`` tuple of floats."""
    try:
        x, y, z = point
    except (TypeError, ValueError):
        raise CameraPathError(f"{what} must be a 3-component vector: {point!r}")
    x, y, z = float(x), float(y), float(z)
    if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
        raise CameraPathError(f"{what} has a non-finite coordinate: {point!r}")
    return (x, y, z)


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _scale(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _norm(a):
    return math.sqrt(_dot(a, a))


def _normalize(vec, what="vector"):
    length = _norm(vec)
    if length <= TOLERANCE:
        raise CameraPathError(f"{what} is zero or near-zero and cannot be normalized")
    return (vec[0] / length, vec[1] / length, vec[2] / length)


def _check_points(points, what="control points"):
    """Shared validation for control-point sequences. Returns coerced list."""
    coerced = [_as_point(p, what="control point") for p in points]
    if len(coerced) < 2:
        raise CameraPathError(
            f"{what}: fewer than two control points ({len(coerced)} given)"
        )
    for i in range(len(coerced)):
        for j in range(i + 1, len(coerced)):
            if _norm(_sub(coerced[i], coerced[j])) <= TOLERANCE:
                raise CameraPathError(
                    f"{what}: duplicate control points at indices {i} and {j}"
                )
    return coerced


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------


def catmull_rom(points, samples):
    """Sample a Catmull-Rom spline through ``points``.

    ``points`` is a sequence of at least two ``(x, y, z)`` control points in
    world units.  The returned list contains exactly ``samples`` positions,
    starts at ``points[0]``, ends at ``points[-1]``, and passes through every
    control point.  Phantom endpoints are mirrored so that the tangent at both
    ends is non-zero.

    Raises :class:`CameraPathError` when there are fewer than two control
    points, when any coordinate is not finite, or when two control points are
    duplicates within ``1e-6``.
    """
    pts = _check_points(points)
    if not isinstance(samples, int) or isinstance(samples, bool) or samples < 2:
        raise CameraPathError(f"samples must be an integer of at least 2, got {samples!r}")

    n = len(pts)
    # Mirrored phantom endpoints keep the endpoint tangents non-zero.
    extended = (
        [_sub(_scale(pts[0], 2.0), pts[1])]
        + list(pts)
        + [_sub(_scale(pts[-1], 2.0), pts[-2])]
    )

    segments = n - 1
    result = []
    for s in range(samples):
        t_global = s * segments / (samples - 1)
        seg = min(int(t_global), segments - 1)
        t = t_global - seg
        p0 = extended[seg]
        p1 = extended[seg + 1]
        p2 = extended[seg + 2]
        p3 = extended[seg + 3]
        t2 = t * t
        t3 = t2 * t
        position = tuple(
            0.5
            * (
                2.0 * p1[c]
                + (-p0[c] + p2[c]) * t
                + (2.0 * p0[c] - 5.0 * p1[c] + 4.0 * p2[c] - p3[c]) * t2
                + (-p0[c] + 3.0 * p1[c] - 3.0 * p2[c] + p3[c]) * t3
            )
            for c in range(3)
        )
        result.append(position)

    # Snap the ends to the exact control points.
    result[0] = pts[0]
    result[-1] = pts[-1]
    return result


def tangent_at(samples, index):
    """Return the unit tangent at ``samples[index]``.

    ``samples`` is a sequence of at least two world-space positions.  Interior
    samples use a central difference; the first and last samples use a
    one-sided forward/backward difference so the endpoint tangent is non-zero.
    The result is normalized to unit length and returned as a tuple.

    Raises :class:`CameraPathError` when ``index`` is out of range, when there
    are fewer than two samples, or when the difference vector is zero.
    """
    pts = [_as_point(p, what="sample") for p in samples]
    if len(pts) < 2:
        raise CameraPathError(
            f"tangent_at needs at least two samples, got {len(pts)}"
        )
    if not isinstance(index, int) or isinstance(index, bool):
        raise CameraPathError(f"index must be an integer, got {index!r}")
    if index < 0:
        index += len(pts)
    if index < 0 or index >= len(pts):
        raise CameraPathError(
            f"index {index} out of range for {len(pts)} samples"
        )

    if index == 0:
        diff = _sub(pts[1], pts[0])
    elif index == len(pts) - 1:
        diff = _sub(pts[-1], pts[-2])
    else:
        diff = _sub(pts[index + 1], pts[index - 1])

    return _normalize(diff, what="tangent difference")


def look_at(pos, target, up):
    """Build a right-handed camera orientation looking from ``pos`` at ``target``.

    ``pos``, ``target`` and ``up`` are world-space ``(x, y, z)`` vectors.
    ``up`` is a hint only; it must not be parallel to the view direction.
    The basis is computed as::

        forward = normalize(target - pos)
        right   = normalize(cross(up, forward))
        up      = cross(forward, right)

    Returns a dict with ``position``, ``forward``, ``right`` and ``up`` (unit
    tuples) plus ``matrix``, a row-major 4x4 view matrix whose first three rows
    are ``right``, ``up`` and ``forward`` with translation ``-dot(axis, pos)``.

    Raises :class:`CameraPathError` when ``pos`` and ``target`` coincide or
    when ``up`` is parallel to ``forward`` (singular basis).
    """
    pos = _as_point(pos, what="pos")
    target = _as_point(target, what="target")
    up_hint = _as_point(up, what="up")

    forward = _normalize(_sub(target, pos), what="view direction")
    right = _normalize(
        _cross(up_hint, forward),
        what="right vector (up hint may be parallel to the view direction)",
    )
    up_out = _normalize(_cross(forward, right), what="recomputed up")

    matrix = [
        [right[0], right[1], right[2], -_dot(right, pos)],
        [up_out[0], up_out[1], up_out[2], -_dot(up_out, pos)],
        [forward[0], forward[1], forward[2], -_dot(forward, pos)],
        [0.0, 0.0, 0.0, 1.0],
    ]
    return {
        "position": pos,
        "forward": forward,
        "right": right,
        "up": up_out,
        "matrix": matrix,
    }


def arc_length(samples):
    """Return the cumulative arc length along the sampled polyline.

    ``samples`` is a sequence of at least two world-space positions.  The
    returned list has one entry per sample, starts at ``0.0``, and accumulates
    Euclidean segment lengths in world units.  For a sampled curve this tracks
    the curved distance rather than the straight-line distance between the
    endpoints; the values are strictly increasing.

    Raises :class:`CameraPathError` when there are fewer than two samples, a
    coordinate is not finite, or a segment has zero length.
    """
    pts = [_as_point(p, what="sample") for p in samples]
    if len(pts) < 2:
        raise CameraPathError(
            f"arc_length needs at least two samples, got {len(pts)}"
        )

    lengths = [0.0]
    for i in range(1, len(pts)):
        segment = _norm(_sub(pts[i], pts[i - 1]))
        if segment <= TOLERANCE:
            raise CameraPathError(
                f"zero-length segment between samples {i - 1} and {i}"
            )
        lengths.append(lengths[-1] + segment)
    return lengths


def resample_uniform(samples, count):
    """Resample a polyline into ``count`` equally spaced arc-length positions.

    ``count`` must be an integer of at least two.  The first returned position
    is exactly ``samples[0]``, the last is exactly ``samples[-1]``, and every
    adjacent pair is separated by the same arc length (within ``1e-6``).
    Interpolation is linear along the input polyline.

    Raises :class:`CameraPathError` when ``count < 2``, when ``samples`` has
    fewer than two positions, or when the total arc length is zero.
    """
    if not isinstance(count, int) or isinstance(count, bool) or count < 2:
        raise CameraPathError(f"count must be an integer of at least 2, got {count!r}")
    pts = [_as_point(p, what="sample") for p in samples]
    if len(pts) < 2:
        raise CameraPathError(
            f"resample_uniform needs at least two samples, got {len(pts)}"
        )

    lengths = arc_length(pts)
    total = lengths[-1]
    if total <= TOLERANCE:
        raise CameraPathError("total arc length is zero; cannot resample")

    spacing = total / (count - 1)
    output = [pts[0]]
    seg = 0  # segment index such that target distance falls in (lengths[seg], lengths[seg+1]]
    for k in range(1, count - 1):
        target_dist = spacing * k
        while seg < len(pts) - 2 and lengths[seg + 1] < target_dist:
            seg += 1
        seg_start = lengths[seg]
        seg_len = lengths[seg + 1] - seg_start
        frac = (target_dist - seg_start) / seg_len
        output.append(_add(pts[seg], _scale(_sub(pts[seg + 1], pts[seg]), frac)))
    output.append(pts[-1])
    return output


def speed_profile(samples, ease):
    """Return a normalized speed curve for the sampled camera path.

    The result has one value per sample.  Every value is in ``[0, 1]``.  With
    ``ease="smooth"`` the profile is symmetric, starts and ends at speed
    ``0.0`` (soft acceleration and deceleration), and peaks at ``1.0`` in the
    middle.  With ``ease="linear"`` every value is ``1.0``.  Values represent
    normalized speed; multiplying by a duration gives travel time.

    Raises :class:`CameraPathError` when there are fewer than two samples or
    when ``ease`` is not one of the supported strings.
    """
    pts = list(samples)
    if len(pts) < 2:
        raise CameraPathError(
            f"speed_profile needs at least two samples, got {len(pts)}"
        )
    if ease not in ("smooth", "linear"):
        raise CameraPathError(
            f"unsupported ease {ease!r}; expected 'smooth' or 'linear'"
        )

    n = len(pts)
    if ease == "linear":
        return [1.0] * n
    # Smooth: sinusoidal ease-in/ease-out, 0 at both ends, 1.0 in the middle.
    return [math.sin(math.pi * i / (n - 1)) for i in range(n)]


def validate(points, samples):
    """Return a list of human-readable problems with a camera path.

    ``points`` is the control-point sequence and ``samples`` is an optional
    sampled polyline (an empty sequence means "not sampled").  An empty result
    means the input is valid.  Conditions reported include fewer than two
    control points, duplicate control points, and non-finite (NaN/inf)
    coordinates in either input.  Messages contain stable keywords such as
    ``"fewer than two"``, ``"duplicate"`` and ``"non-finite"``.
    """
    problems = []

    pts = list(points) if points is not None else []
    if len(pts) < 2:
        problems.append(
            f"fewer than two control points ({len(pts)} given)"
        )

    coerced = []
    for i, p in enumerate(pts):
        try:
            coerced.append(_as_point(p, what="control point"))
        except CameraPathError:
            problems.append(f"non-finite coordinate in control point {i}: {p!r}")
    for i in range(len(coerced)):
        for j in range(i + 1, len(coerced)):
            if _norm(_sub(coerced[i], coerced[j])) <= TOLERANCE:
                problems.append(
                    f"duplicate control points at indices {i} and {j}"
                )

    for i, p in enumerate(samples or []):
        try:
            _as_point(p, what="sample")
        except CameraPathError:
            problems.append(f"non-finite coordinate in sample {i}: {p!r}")

    return problems


def path_batch(paths, config):
    """Sample several camera paths atomically.

    ``paths`` is a sequence of dicts, each with ``"points"`` (control points)
    and an optional ``"name"``.  ``config`` is a dict with ``"samples"`` (an
    integer of at least two) and optional ``"ease"`` (default ``"smooth"``).
    On success returns::

        {
            "paths": [
                {
                    "name": name,
                    "samples": [positions...],
                    "tangents": [unit tangents...],
                    "speed": [normalized speeds...],
                },
                ...
            ],
            "count": number_of_paths,
        }

    The order of ``paths`` is preserved.  Every path owns independent sample,
    tangent, and speed buffers; no buffer is shared between results.  All
    inputs are validated before any output is built, so if any path is invalid
    the whole call raises :class:`CameraPathError` and leaves the input
    structures untouched (atomic rollback).
    """
    if not isinstance(config, dict):
        raise CameraPathError("config must be a dict")
    samples_count = config.get("samples")
    if (
        not isinstance(samples_count, int)
        or isinstance(samples_count, bool)
        or samples_count < 2
    ):
        raise CameraPathError(
            f"config['samples'] must be an integer of at least 2, got {samples_count!r}"
        )
    ease = config.get("ease", "smooth")
    if ease not in ("smooth", "linear"):
        raise CameraPathError(
            f"unsupported ease {ease!r}; expected 'smooth' or 'linear'"
        )

    entries = list(paths)

    # Phase 1: validate everything before building any output (atomic).
    validated = []
    for i, entry in enumerate(entries):
        if not isinstance(entry, dict) or "points" not in entry:
            raise CameraPathError(f"path {i} must be a dict with a 'points' key")
        pts = _check_points(entry["points"], what=f"path {i} control points")
        validated.append((entry.get("name"), pts))

    # Phase 2: build independent buffers per path.
    out_paths = []
    for name, pts in validated:
        sampled = catmull_rom(pts, samples_count)
        tangents = [tangent_at(sampled, i) for i in range(len(sampled))]
        speed = speed_profile(sampled, ease)
        out_paths.append(
            {
                "name": name,
                "samples": list(sampled),
                "tangents": list(tangents),
                "speed": list(speed),
            }
        )

    return {"paths": out_paths, "count": len(out_paths)}
