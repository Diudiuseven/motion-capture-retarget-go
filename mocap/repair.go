package mocap

import (
	"math"
	"sort"
)

// RepairConfig controls fault-tolerance behaviour.
type RepairConfig struct {
	MaxGapFrames int     // max consecutive missing frames to interpolate; larger gaps are an error
	DriftTol     float64 // timestamp drift tolerance in seconds
}

// Repair sorts frames by timestamp, removes duplicates, corrects
// timestamp drift onto the nominal sample grid and interpolates
// missing frames. Missing data is never treated as zero.
func Repair(clip *Clip, cfg RepairConfig) (*Clip, error) {
	if clip.SampleRate <= 0 {
		return nil, errf("repair", -1, "", "sample rate must be > 0")
	}
	if len(clip.Frames) == 0 {
		return nil, errf("repair", -1, "", "clip has no frames")
	}
	if cfg.DriftTol <= 0 {
		cfg.DriftTol = 0.5 / clip.SampleRate
	}
	dt := 1 / clip.SampleRate

	// 1. Sort by timestamp (stable: keeps input order for equal keys).
	sorted := make([]Frame, len(clip.Frames))
	for i, f := range clip.Frames {
		sorted[i] = f.Copy()
	}
	sort.SliceStable(sorted, func(i, j int) bool { return sorted[i].Time < sorted[j].Time })

	// 2. Drop duplicates: frames landing on the same grid slot.
	type slotFrame struct {
		slot int64
		f    Frame
	}
	t0 := sorted[0].Time
	slotOf := func(t float64) int64 { return int64(math.Round((t - t0) / dt)) }
	uniq := make([]slotFrame, 0, len(sorted))
	for _, f := range sorted {
		s := slotOf(f.Time)
		if n := len(uniq); n > 0 && uniq[n-1].slot == s {
			continue // duplicate sample for the same grid slot: keep first
		}
		uniq = append(uniq, slotFrame{s, f})
	}

	// 3. Rebuild every grid slot, interpolating across gaps.
	lastSlot := uniq[len(uniq)-1].slot
	if gap := int(lastSlot) - 0; gap > 0 && len(uniq) == 1 {
		return nil, errf("repair", 0, "", "only one frame for %d slots", lastSlot+1)
	}
	out := &Clip{Skeleton: clip.Skeleton, SampleRate: clip.SampleRate}
	out.Frames = make([]Frame, 0, lastSlot+1)

	ui := 0 // index into uniq such that uniq[ui].slot <= current slot
	for slot := int64(0); slot <= lastSlot; slot++ {
		for ui+1 < len(uniq) && uniq[ui+1].slot <= slot {
			ui++
		}
		t := t0 + float64(slot)*dt
		if uniq[ui].slot == slot {
			f := uniq[ui].f.Copy()
			f.Time = t // snap drifted timestamp onto the grid
			out.Frames = append(out.Frames, f)
			continue
		}
		// Gap: interpolate between the surrounding captured frames.
		if ui+1 >= len(uniq) {
			return nil, errf("repair", int(slot), "", "no frame after slot %d to interpolate from", slot)
		}
		a, b := uniq[ui], uniq[ui+1]
		gap := b.slot - a.slot - 1
		if cfg.MaxGapFrames >= 0 && int(gap) > cfg.MaxGapFrames {
			return nil, errf("repair", int(slot), "",
				"gap of %d missing frames exceeds MaxGapFrames=%d", gap, cfg.MaxGapFrames)
		}
		tt := float64(slot-a.slot) / float64(b.slot-a.slot)
		out.Frames = append(out.Frames, lerpFrame(clip.Skeleton, a.f, b.f, tt, t))
	}
	fillMissingJoints(clip.Skeleton, out.Frames)
	return out, nil
}

// fillMissingJoints patches per-joint gaps inside captured frames by
// interpolating from the nearest frames that contain the joint.
func fillMissingJoints(skel *Skeleton, frames []Frame) {
	for _, j := range skel.Joints {
		for i := range frames {
			if _, ok := frames[i].Rotations[j.Name]; ok {
				continue
			}
			lo, hi := -1, -1
			for k := i - 1; k >= 0; k-- {
				if _, ok := frames[k].Rotations[j.Name]; ok {
					lo = k
					break
				}
			}
			for k := i + 1; k < len(frames); k++ {
				if _, ok := frames[k].Rotations[j.Name]; ok {
					hi = k
					break
				}
			}
			switch {
			case lo >= 0 && hi >= 0:
				t := float64(i-lo) / float64(hi-lo)
				frames[i].Rotations[j.Name] = frames[lo].Rotations[j.Name].
					Slerp(frames[hi].Rotations[j.Name], t)
			case lo >= 0:
				frames[i].Rotations[j.Name] = frames[lo].Rotations[j.Name]
			case hi >= 0:
				frames[i].Rotations[j.Name] = frames[hi].Rotations[j.Name]
			default:
				frames[i].Rotations[j.Name] = j.RestRot
			}
		}
	}
}

// lerpFrame interpolates two frames. Joints missing from either side
// are filled from the nearest captured value instead of zero.
func lerpFrame(skel *Skeleton, a, b Frame, t, time float64) Frame {
	f := Frame{Time: time, RootPos: a.RootPos.Lerp(b.RootPos, t)}
	f.Rotations = make(map[string]Quat, len(skel.Joints))
	for _, j := range skel.Joints {
		qa, oka := a.Rotations[j.Name]
		qb, okb := b.Rotations[j.Name]
		switch {
		case oka && okb:
			f.Rotations[j.Name] = qa.Slerp(qb, t)
		case oka:
			f.Rotations[j.Name] = qa
		case okb:
			f.Rotations[j.Name] = qb
		default:
			f.Rotations[j.Name] = j.RestRot // never zero: fall back to rest pose
		}
	}
	return f
}
