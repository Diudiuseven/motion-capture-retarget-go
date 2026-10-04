package mocap

import (
	"math"
	"testing"
)

func TestAlignLoopRemovesSeam(t *testing.T) {
	s, err := NewSkeleton(testJoints())
	if err != nil {
		t.Fatal(err)
	}
	// Build a near-loop: frame 0 and frame N-1 differ slightly.
	n := 40
	poses := make([]Pose, n)
	for i := range poses {
		p := s.NewRestPose()
		phase := 2 * math.Pi * float64(i) / float64(n)
		p.Rotations[s.Index("Spine")] = QuatFromAxisAngle(Vec3{Z: 1}, 0.3*math.Sin(phase))
		poses[i] = p
	}
	cfg := LoopConfig{MaxSeamDeviation: 0.2, MaxRootSeamDistance: 0.01, BlendFrames: 8}
	out, err := AlignLoop(s, poses, cfg)
	if err != nil {
		t.Fatalf("align failed: %v", err)
	}
	// Last frame must exactly equal the first: zero seam.
	dev, rootDist := PoseDeviation(out[n-1], out[0])
	if dev != 0 || rootDist != 0 {
		t.Fatalf("seam not closed: dev=%v rootDist=%v", dev, rootDist)
	}
	// No visible jump between consecutive frames near the seam.
	for i := n - 4; i < n-1; i++ {
		d, _ := PoseDeviation(out[i], out[i+1])
		if d > 0.1 {
			t.Fatalf("visible jump between frames %d and %d: %v rad", i, i+1, d)
		}
	}
}

func TestAlignLoopRejectsUnalignable(t *testing.T) {
	s, err := NewSkeleton(testJoints())
	if err != nil {
		t.Fatal(err)
	}
	poses := make([]Pose, 10)
	for i := range poses {
		p := s.NewRestPose()
		p.Rotations[s.Index("Spine")] = QuatFromAxisAngle(Vec3{Z: 1}, float64(i)*0.5)
		poses[i] = p
	}
	_, err = AlignLoop(s, poses, LoopConfig{MaxSeamDeviation: 0.1, MaxRootSeamDistance: 0.01})
	le, ok := err.(*LoopError)
	if !ok {
		t.Fatalf("error type = %T, want *LoopError", err)
	}
	if le.Deviation <= le.Limit {
		t.Fatalf("loop error inconsistent: %+v", le)
	}
}
