package mocap

// SmoothConfig controls pose smoothing.
type SmoothConfig struct {
	// Window is the smoothing kernel half-width in frames (0 disables).
	Window int
}

// SmoothPoses applies a deterministic moving-average filter to local
// rotations and root positions. Quaternion averaging uses sign-aligned
// weighted accumulation renormalized to the unit sphere.
func SmoothPoses(s *Skeleton, poses []Pose, cfg SmoothConfig) []Pose {
	n := len(poses)
	if n == 0 || cfg.Window <= 0 {
		return poses
	}
	w := cfg.Window
	out := make([]Pose, n)
	for i := 0; i < n; i++ {
		lo, hi := i-w, i+w
		if lo < 0 {
			lo = 0
		}
		if hi > n-1 {
			hi = n - 1
		}
		count := float64(hi - lo + 1)
		p := Pose{Rotations: make([]Quat, len(poses[i].Rotations))}
		for j := range p.Rotations {
			// Align signs to the center frame to avoid cancellation.
			ref := poses[i].Rotations[j]
			var acc Quat
			for k := lo; k <= hi; k++ {
				q := poses[k].Rotations[j]
				if q.X*ref.X+q.Y*ref.Y+q.Z*ref.Z+q.W*ref.W < 0 {
					q = Quat{-q.X, -q.Y, -q.Z, -q.W}
				}
				acc.X += q.X
				acc.Y += q.Y
				acc.Z += q.Z
				acc.W += q.W
			}
			p.Rotations[j] = acc.Normalized()
		}
		for k := lo; k <= hi; k++ {
			p.RootPos = p.RootPos.Add(poses[k].RootPos)
		}
		p.RootPos = p.RootPos.Scale(1 / count)
		out[i] = p
	}
	return out
}

// CompressConfig controls error-bounded keyframe compression.
type CompressConfig struct {
	// Tolerance is the maximum allowed joint-angle deviation in radians.
	Tolerance float64
}

// CompressKeyframes removes frames while keeping the interpolated error
// within tolerance. It uses deterministic recursive splitting: a segment
// is split at its worst-deviating frame until every segment satisfies
// the bound. Identical input always yields identical output.
func CompressKeyframes(s *Skeleton, poses []Pose, frameRate float64, cfg CompressConfig) []Keyframe {
	n := len(poses)
	if n == 0 {
		return nil
	}
	if frameRate <= 0 {
		frameRate = 30
	}
	dt := 1.0 / frameRate
	if n <= 2 || cfg.Tolerance <= 0 {
		keys := make([]Keyframe, n)
		for i, p := range poses {
			keys[i] = Keyframe{Time: float64(i) * dt, Pose: p}
		}
		return keys
	}

	keep := make([]bool, n)
	keep[0], keep[n-1] = true, true
	// Iterative stack of open segments [lo, hi].
	type segment struct{ lo, hi int }
	stack := []segment{{0, n - 1}}
	for len(stack) > 0 {
		seg := stack[len(stack)-1]
		stack = stack[:len(stack)-1]
		a, b := poses[seg.lo], poses[seg.hi]
		span := float64(seg.hi - seg.lo)
		worst, worstIx := cfg.Tolerance, -1
		for i := seg.lo + 1; i < seg.hi; i++ {
			t := float64(i-seg.lo) / span
			interp := BlendPoses(s, a, b, t)
			dev, _ := PoseDeviation(poses[i], interp)
			if dev > worst {
				worst, worstIx = dev, i
			}
		}
		if worstIx >= 0 {
			keep[worstIx] = true
			stack = append(stack, segment{seg.lo, worstIx}, segment{worstIx, seg.hi})
		}
	}
	var keys []Keyframe
	for i, p := range poses {
		if keep[i] {
			keys = append(keys, Keyframe{Time: float64(i) * dt, Pose: p})
		}
	}
	return keys
}
