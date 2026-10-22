"""Initialization-only terrain envelope checks; never produces model inputs."""
import math
import numpy as np


def validate_envelope(msg, now, cfg, geometry, watermark):
    if (msg.schema_version != 2 or msg.profile != cfg['profile'] or
            msg.header.frame_id != cfg['frames']['output'] or
            not msg.motion_compensated or not msg.operator_commissioned or
            msg.geometry_hash != geometry):
        raise ValueError('Terrain initialization provenance/ABI mismatch')
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    elapsed = now - stamp
    if (not math.isfinite(elapsed) or elapsed < -cfg['runtime']['future_tolerance_s'] or
            elapsed > cfg['runtime']['input_timeout_s']):
        raise ValueError('Terrain initialization transport timestamp stale/future')
    arrays = [np.asarray(getattr(msg, name), dtype=np.float64)
              for name in ('height', 'validity', 'confidence', 'variance', 'age')]
    if any(a.shape != (187,) or not np.isfinite(a).all() for a in arrays):
        raise ValueError('Terrain initialization malformed/nonfinite arrays')
    h, m, c, v, a = arrays
    if (np.any(np.abs(h) > 1) or not np.all((m == 0) | (m == 1)) or
            np.any((c < 0) | (c > 1)) or np.any(v < 0) or
            np.any(h[m == 0] != 0) or np.any(c[m == 0] != 0) or np.any(a[m == 1] < 0)):
        raise ValueError('Terrain initialization malformed ranges/unknown semantics')
    if (not isinstance(msg.session_id, int) or isinstance(msg.session_id, bool) or
            not 0 <= msg.session_id < 2**64):
        raise ValueError('Terrain initialization session invalid')
    if (not isinstance(msg.sequence, int) or isinstance(msg.sequence, bool) or not 0 <= msg.sequence < 2**64 or
            (watermark and (msg.session_id != watermark['session'] or msg.sequence <= watermark['sequence']))):
        raise ValueError('Terrain initialization session/sequence changed or replayed')
    watermark.update(session=msg.session_id, sequence=msg.sequence)
    return stamp


WAIVED_INIT_QUALITY_ERRORS = frozenset((
    'Insufficient valid terrain coverage',
    'Terrain/query pose stale or from future',
))
