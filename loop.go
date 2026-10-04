package mocap

import "fmt"

// LoopConfig controls loop alignment.
type LoopConfig struct {
	// MaxSeamDeviation is the maximum allowed pose deviation (radians)
	// between the last and first frame for a clip to be accepted as a
	// loop. Root position distance is compared against
	// MaxRootSeamDistance.
	MaxSeamDeviation float64
	// MaxRootSeamDistance is the maximum allowed root position jump.
	MaxRootSeamDistance float64
	// BlendFrames is the number of tail frames blended toward the first
	// frame to remove the residual seam.
	BlendFrames int
}

// LoopError reports that a clip cannot be aligned into a loop.
type LoopError struct {
	Deviation float64
	Limit     float64
	RootDist  float64
	RootLimit float64
}

func (e *LoopError) Error() string {
	return fmt.Sprintf("loop alignment failed: seam deviation %.4f rad (limit %.4f), root jump %.4f (limit %.4f)",
		e.Deviation, e.Limit, e.RootDist, e.RootLimit)
}

// AlignLoop verifies the seam and blends the tail into the head so the
// clip loops without visible jumps. It returns a LoopError when the
// deviation exceeds the configured limits.
func AlignLoop(s *Skeleton, poses []Pose, cfg LoopConfig) ([]Pose, error) {
	n := len(poses)
	if n < 2 {
		return poses, nil
	}
	dev, rootDist := PoseDeviation(poses[n-1], poses[0])
	if dev > cfg.MaxSeamDeviation || rootDist > cfg.MaxRootSeamDistance {
		return nil, &LoopError{
			Deviation: dev, Limit: cfg.MaxSeamDeviation,
			RootDist: rootDist, RootLimit: cfg.MaxRootSeamDistance,
		}
	}
	blend := cfg.BlendFrames
	if blend <= 0 || blend > n-1 {
		return poses, nil
	}
	out := make([]Pose, n)
	copy(out, poses)
	// Blend the tail toward the head pose with increasing weight so the
	// final frame exactly matches the first.
	for k := 0; k < blend; k++ {
		i := n - 1 - k
		t := float64(blend-k) / float64(blend+1) // 0 near head-side, ->1 at last frame
		out[i] = BlendPoses(s, poses[i], poses[0], t)
	}
	// Pin the last frame exactly to the first for a zero seam.
	out[n-1] = poses[0]
	return out, nil
}
