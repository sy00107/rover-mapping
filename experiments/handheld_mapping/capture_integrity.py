"""Bounded recording continuity checks, separate from successful file transfer."""
from collections import OrderedDict


class CaptureIntegrity:
    def __init__(self, fps=15., depth_tolerance_ns=2_000_000):
        self.period_ns = 1e9/fps
        self.tolerance = depth_tolerance_ns
        self.streams = {s: dict(count=0, first_stamp_ns=None, last_stamp_ns=None,
            max_gap_ms=0., gap_events=0, sequence_gaps=0) for s in ('left', 'right', 'depth')}
        self.sequences = {}
        self.pending = {s: OrderedDict() for s in ('left', 'right', 'depth_left', 'depth')}
        self.stereo_pairs = self.depth_matches = 0

    def _offer(self, side, other, stamp, tolerance):
        candidates = self.pending[other]
        match = min(candidates, key=lambda t: abs(t-stamp), default=None)
        if match is not None and abs(match-stamp) <= tolerance:
            del candidates[match]
            return True
        own = self.pending[side]
        own[stamp] = None
        while len(own) > 128:
            own.popitem(last=False)
        return False

    def offer(self, header):
        side, stamp = header['stream'], header['stamp_ns']
        state = self.streams[side]
        if state['last_stamp_ns'] is not None:
            gap = stamp-state['last_stamp_ns']
            if gap <= 0:
                raise ValueError('non-increasing capture timestamp')
            state['max_gap_ms'] = max(state['max_gap_ms'], gap/1e6)
            if gap > 1.5*self.period_ns:
                state['gap_events'] += 1
        else:
            state['first_stamp_ns'] = stamp
        sequence = header.get('stream_seq')
        if sequence is not None:
            previous = self.sequences.get(side)
            if previous is not None and sequence > previous+1:
                state['sequence_gaps'] += sequence-previous-1
            self.sequences[side] = sequence
        state['count'] += 1
        state['last_stamp_ns'] = stamp
        if side in ('left', 'right'):
            self.stereo_pairs += self._offer(side, 'right' if side == 'left' else 'left', stamp, 0)
        if side in ('left', 'depth'):
            self.depth_matches += self._offer('depth_left' if side == 'left' else 'depth',
                'depth' if side == 'left' else 'depth_left', stamp, self.tolerance)

    def result(self):
        counts = {s: row['count'] for s, row in self.streams.items()}
        issues = []
        for side, row in self.streams.items():
            if not row['count']:
                issues.append(side+' 영상 없음')
            if row['gap_events'] or row['sequence_gaps']:
                issues.append(side+' 영상 시각/순번 공백')
        unmatched = counts['left']+counts['right']-2*self.stereo_pairs
        if unmatched:
            issues.append('좌우 시각 미대응 영상 '+str(unmatched)+'장')
        if self.depth_matches < counts['left']:
            issues.append('깊이 미대응 좌측 영상 '+str(counts['left']-self.depth_matches)+'장')
        return dict(partial=bool(issues), issues=issues, counts=counts,
            streams={s: dict(row) for s, row in self.streams.items()},
            stereo_pairs=self.stereo_pairs, unmatched_stereo_images=unmatched,
            left_depth_matches=self.depth_matches,
            stereo_pair_fraction=2*self.stereo_pairs/max(1, counts['left']+counts['right']),
            left_depth_fraction=self.depth_matches/max(1, counts['left']),
            gap_threshold_ms=1.5*self.period_ns/1e6,
            scope='Saved callbacks; timestamp gaps detect discontinuity but are not a verified hardware frame counter.')


def stale_streams(receipts, now, timeout_s=5.):
    return [side for side, times in receipts.items() if not times or now-times[-1] > timeout_s]
