package mocap

// LoopConfig controls loop alignment.
type LoopConfig struct {
	SearchWindow int     // frames near the end to search for a seam match
	MaxError     float64 // max allowed seam deviation (radians); 0 = exact
}

// AlignLoop trims/blends the tail of a clip so the last frame matches
// the first within MaxError. Returns a locatable error when no
// acceptable seam can be found.
func AlignLoop(clip *Clip, cfg LoopConfig) (*Clip, error) {
	n := len(clip.Frames)
	if n < 3 {
		return nil, errf("loop", -1, "", "clip too short to align (%d frames)", n)
	}
	if cfg.SearchWindow <= 0 || cfg.SearchWindow > n-1 {
		cfg.SearchWindow = n - 1
	}

	// Find the tail frame closest to the first frame.
	first := clip.Frames[0]
	best, bestErr := -1, 0.0
	for i := n - cfg.SearchWindow; i < n; i++ {
		e := poseError(clip.Skeleton, first, clip.Frames[i])
		if best < 0 || e < bestErr {
			best, bestErr = i, e
		}
	}
	if bestErr > cfg.MaxError {
		return nil, errf("loop", best, "",
			"no seamless loop point: best seam deviation %.4f exceeds MaxError %.4f",
			bestErr, cfg.MaxError)
	}

	// Trim everything after the seam frame.
	frames := make([]Frame, best+1)
	for i := 0; i <= best; i++ {
		frames[i] = clip.Frames[i].Copy()
	}

	// Crossfade the tail toward the first frame so the seam has no
	// visible jump, then close the loop with an exact copy of frame 0.
	const blendFrames = 8
	m := blendFrames
	if m > best {
		m = best
	}
	for j := 0; j < m; j++ {
		idx := best - m + 1 + j
		w := float64(j+1) / float64(m+1)
		frames[idx] = blendToward(clip.Skeleton, frames[idx], first, w)
	}
	dt := 1.0
	if clip.SampleRate > 0 {
		dt = 1 / clip.SampleRate
	}
	last := first.Copy()
	last.Time = frames[best].Time + dt
	frames = append(frames, last)

	return &Clip{Skeleton: clip.Skeleton, Frames: frames, SampleRate: clip.SampleRate}, nil
}

// poseError is the max rotation deviation (radians) plus root
// translation distance between two frames.
func poseError(skel *Skeleton, a, b Frame) float64 {
	maxErr := a.RootPos.Sub(b.RootPos).Len()
	for _, j := range skel.Joints {
		qa, oka := a.Rotations[j.Name]
		qb, okb := b.Rotations[j.Name]
		if !oka || !okb {
			continue
		}
		if e := qa.AngleTo(qb); e > maxErr {
			maxErr = e
		}
	}
	return maxErr
}

// blendToward moves frame f a fraction w toward the target frame.
func blendToward(skel *Skeleton, f, target Frame, w float64) Frame {
	out := f.Copy()
	out.RootPos = f.RootPos.Lerp(target.RootPos, w)
	for name, q := range f.Rotations {
		if tq, ok := target.Rotations[name]; ok {
			out.Rotations[name] = q.Slerp(tq, w)
		}
	}
	return out
}
