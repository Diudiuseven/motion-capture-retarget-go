package mocap

// Smooth applies a temporal smoothing filter to reduce jitter.
// window is the kernel half-width in frames (0 = no smoothing).
func Smooth(clip *Clip, window int) *Clip {
	out := clip.Copy()
	if window <= 0 || len(clip.Frames) == 0 {
		return out
	}
	skel := clip.Skeleton
	n := len(clip.Frames)
	for i := 0; i < n; i++ {
		lo, hi := i-window, i+window
		if lo < 0 {
			lo = 0
		}
		if hi > n-1 {
			hi = n - 1
		}
		var pos Vec3
		cnt := float64(hi - lo + 1)
		for k := lo; k <= hi; k++ {
			pos = pos.Add(clip.Frames[k].RootPos)
		}
		out.Frames[i].RootPos = pos.Scale(1 / cnt)
		for _, j := range skel.Joints {
			center := clip.Frames[i].Rotations[j.Name]
			acc := Quat{}
			for k := lo; k <= hi; k++ {
				q := clip.Frames[k].Rotations[j.Name]
				// Align hemispheres so averaging never crosses the
				// double-cover seam (prevents joint flips).
				if q.W*center.W+q.X*center.X+q.Y*center.Y+q.Z*center.Z < 0 {
					q = Quat{-q.W, -q.X, -q.Y, -q.Z}
				}
				acc.W += q.W
				acc.X += q.X
				acc.Y += q.Y
				acc.Z += q.Z
			}
			out.Frames[i].Rotations[j.Name] = acc.Norm()
		}
	}
	return out
}

// Compress removes redundant keyframes so that the deviation between
// the compressed clip (resampled with slerp/lerp) and the original
// stays within tolerance (radians for rotations, same value in length
// units for root translation). Identical input always produces
// identical output.
func Compress(clip *Clip, tolerance float64) (*Clip, error) {
	if tolerance < 0 {
		return nil, errf("compress", -1, "", "tolerance must be >= 0")
	}
	n := len(clip.Frames)
	if n <= 2 {
		return clip.Copy(), nil
	}
	keep := make([]bool, n)
	keep[0], keep[n-1] = true, true

	// Iteratively split the worst interval at its max-error frame.
	type interval struct{ a, b int }
	stack := []interval{{0, n - 1}}
	for len(stack) > 0 {
		iv := stack[len(stack)-1]
		stack = stack[:len(stack)-1]
		worst, worstErr := -1, tolerance
		for i := iv.a + 1; i < iv.b; i++ {
			e := frameError(clip, iv.a, iv.b, i)
			if e > worstErr {
				worst, worstErr = i, e
			}
		}
		if worst < 0 {
			continue // interval within tolerance
		}
		keep[worst] = true
		stack = append(stack, interval{iv.a, worst}, interval{worst, iv.b})
	}

	out := &Clip{Skeleton: clip.Skeleton, SampleRate: clip.SampleRate}
	for i, f := range clip.Frames {
		if keep[i] {
			out.Frames = append(out.Frames, f.Copy())
		}
	}
	return out, nil
}

// frameError returns the deviation at frame i when interpolating
// linearly between keyframes a and b.
func frameError(clip *Clip, a, b, i int) float64 {
	fa, fb, fi := clip.Frames[a], clip.Frames[b], clip.Frames[i]
	span := fb.Time - fa.Time
	t := 0.0
	if span > 0 {
		t = (fi.Time - fa.Time) / span
	}
	maxErr := fi.RootPos.Sub(fa.RootPos.Lerp(fb.RootPos, t)).Len()
	for name, qa := range fa.Rotations {
		qb, ok := fb.Rotations[name]
		if !ok {
			continue
		}
		qi, ok := fi.Rotations[name]
		if !ok {
			continue
		}
		if e := qa.Slerp(qb, t).AngleTo(qi); e > maxErr {
			maxErr = e
		}
	}
	return maxErr
}
