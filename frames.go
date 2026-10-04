package mocap

import (
	"fmt"
	"sort"
)

// FrameCleaner removes duplicate frames, restores capture order,
// corrects timestamp drift, and interpolates missing frames.
type FrameCleaner struct {
	SampleRate float64 // nominal rate in Hz
	// MaxGapFrames is the largest gap (in frames) that will be
	// interpolated; larger gaps are reported as errors.
	MaxGapFrames int
}

// GapError reports a frame gap too large to interpolate.
type GapError struct {
	AfterSequence uint64
	Missing       int
}

func (e *GapError) Error() string {
	return fmt.Sprintf("frame gap of %d frames after sequence %d exceeds interpolation limit",
		e.Missing, e.AfterSequence)
}

// CleanFrames takes arbitrarily ordered, duplicated, drifting frames and
// returns a gap-free, uniformly sampled sequence ordered by capture
// sequence number. Missing frames are interpolated from neighbours,
// never treated as zero. Duplicate sequence numbers keep the first
// occurrence, so repeated or shuffled input yields the same output as
// complete ordered input.
func (c *FrameCleaner) CleanFrames(in []NormalizedFrame) ([]NormalizedFrame, error) {
	if c.SampleRate <= 0 {
		return nil, fmt.Errorf("frame cleaner: sample rate must be positive, got %v", c.SampleRate)
	}
	if len(in) == 0 {
		return nil, nil
	}

	// 1. Deduplicate by sequence number (first occurrence wins).
	seen := make(map[uint64]bool, len(in))
	uniq := make([]NormalizedFrame, 0, len(in))
	for _, f := range in {
		if seen[f.Sequence] {
			continue
		}
		seen[f.Sequence] = true
		uniq = append(uniq, f)
	}

	// 2. Restore capture order.
	sort.Slice(uniq, func(i, j int) bool { return uniq[i].Sequence < uniq[j].Sequence })

	// 3. Rebuild the full sequence, interpolating gaps.
	dt := 1.0 / c.SampleRate
	out := make([]NormalizedFrame, 0, len(uniq))
	out = append(out, uniq[0])
	for i := 1; i < len(uniq); i++ {
		prev, cur := uniq[i-1], uniq[i]
		gap := int(cur.Sequence - prev.Sequence)
		if gap <= 0 {
			continue // unreachable after dedup+sort, kept for safety
		}
		missing := gap - 1
		if missing > 0 {
			if c.MaxGapFrames > 0 && missing > c.MaxGapFrames {
				return nil, &GapError{AfterSequence: prev.Sequence, Missing: missing}
			}
			for k := 1; k <= missing; k++ {
				t := float64(k) / float64(gap)
				out = append(out, interpolateFrame(prev, cur, t, prev.Sequence+uint64(k)))
			}
		}
		out = append(out, cur)
	}

	// 4. Correct timestamp drift: re-anchor every frame to the nominal
	//    grid derived from sequence numbers, using the first frame's
	//    timestamp as the epoch.
	epoch := out[0].Time
	base := out[0].Sequence
	for i := range out {
		out[i].Time = epoch + float64(out[i].Sequence-base)*dt
	}
	return out, nil
}

// interpolateFrame blends two frames at parameter t in (0,1).
func interpolateFrame(a, b NormalizedFrame, t float64, seq uint64) NormalizedFrame {
	f := NormalizedFrame{
		Sequence:  seq,
		Time:      a.Time + (b.Time-a.Time)*t,
		Rotations: make(map[string]Quat, len(a.Rotations)),
	}
	for name, qa := range a.Rotations {
		if qb, ok := b.Rotations[name]; ok {
			f.Rotations[name] = qa.Slerp(qb, t)
		} else {
			// Joint absent in one neighbour: hold the known value
			// rather than inventing a zero rotation.
			f.Rotations[name] = qa
		}
	}
	for name, qb := range b.Rotations {
		if _, ok := a.Rotations[name]; !ok {
			f.Rotations[name] = qb
		}
	}
	switch {
	case a.HasRoot && b.HasRoot:
		f.RootPos = a.RootPos.Lerp(b.RootPos, t)
		f.HasRoot = true
	case a.HasRoot:
		f.RootPos, f.HasRoot = a.RootPos, true
	case b.HasRoot:
		f.RootPos, f.HasRoot = b.RootPos, true
	}
	return f
}
