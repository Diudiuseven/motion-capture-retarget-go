package mocap

import "math"

// Keyframe is a compressed pose sample.
type Keyframe struct {
	Time float64
	Pose Pose
}

// Clip is a playable, reproducible animation on a target skeleton.
type Clip struct {
	Skeleton  *Skeleton
	FrameRate float64
	Duration  float64
	Loop      bool
	Frames    []Pose // uniformly sampled at FrameRate
}

// Sample returns the interpolated pose at time t (seconds).
// For looping clips t wraps around Duration; otherwise it clamps.
func (c *Clip) Sample(t float64) Pose {
	n := len(c.Frames)
	if n == 0 {
		return Pose{}
	}
	if n == 1 || c.FrameRate <= 0 {
		return c.Frames[0]
	}
	if c.Loop && c.Duration > 0 {
		t = math.Mod(t, c.Duration)
		if t < 0 {
			t += c.Duration
		}
	} else {
		if t < 0 {
			t = 0
		}
		if maxT := float64(n-1) / c.FrameRate; t > maxT {
			t = maxT
		}
	}
	pos := t * c.FrameRate
	i := int(math.Floor(pos))
	frac := pos - float64(i)
	if i >= n-1 {
		if c.Loop {
			// Blend last frame back into the first.
			return BlendPoses(c.Skeleton, c.Frames[n-1], c.Frames[0], frac)
		}
		return c.Frames[n-1]
	}
	return BlendPoses(c.Skeleton, c.Frames[i], c.Frames[i+1], frac)
}

// BlendPoses interpolates two poses of the same skeleton.
func BlendPoses(s *Skeleton, a, b Pose, t float64) Pose {
	out := Pose{Rotations: make([]Quat, len(a.Rotations))}
	for i := range a.Rotations {
		out.Rotations[i] = a.Rotations[i].Slerp(b.Rotations[i], t)
	}
	out.RootPos = a.RootPos.Lerp(b.RootPos, t)
	return out
}

// PoseDeviation returns the maximum joint-angle deviation (radians)
// between two poses, plus the root position distance.
func PoseDeviation(a, b Pose) (maxAngle float64, rootDist float64) {
	for i := range a.Rotations {
		d := a.Rotations[i].Normalized().Mul(b.Rotations[i].Normalized().Conjugate()).Angle()
		if d > maxAngle {
			maxAngle = d
		}
	}
	return maxAngle, a.RootPos.Sub(b.RootPos).Len()
}
