package mocap

import (
	"math"
	"testing"
)

func wavePoses(t *testing.T, n int) (*Skeleton, []Pose) {
	t.Helper()
	s, err := NewSkeleton(testJoints())
	if err != nil {
		t.Fatal(err)
	}
	poses := make([]Pose, n)
	for i := range poses {
		p := s.NewRestPose()
		a := math.Sin(float64(i)*0.1) * 0.5
		p.Rotations[s.Index("Spine")] = QuatFromAxisAngle(Vec3{Z: 1}, a)
		p.RootPos = Vec3{X: float64(i) * 0.01, Y: 1}
		poses[i] = p
	}
	return s, poses
}

func TestCompressKeyframesErrorBound(t *testing.T) {
	s, poses := wavePoses(t, 200)
	const tol = 0.02
	keys := CompressKeyframes(s, poses, 30, CompressConfig{Tolerance: tol})
	if len(keys) >= len(poses) {
		t.Fatalf("no compression: %d keys for %d frames", len(keys), len(poses))
	}
	// Reconstruct and verify every frame stays within tolerance.
	for i, want := range poses {
		time := float64(i) / 30
		got := sampleKeys(s, keys, time)
		dev, _ := PoseDeviation(want, got)
		if dev > tol+1e-9 {
			t.Fatalf("frame %d deviation %v exceeds tolerance %v", i, dev, tol)
		}
	}
}

func sampleKeys(s *Skeleton, keys []Keyframe, time float64) Pose {
	if time <= keys[0].Time {
		return keys[0].Pose
	}
	for i := 0; i < len(keys)-1; i++ {
		if time >= keys[i].Time && time <= keys[i+1].Time {
			span := keys[i+1].Time - keys[i].Time
			t := 0.0
			if span > 0 {
				t = (time - keys[i].Time) / span
			}
			return BlendPoses(s, keys[i].Pose, keys[i+1].Pose, t)
		}
	}
	return keys[len(keys)-1].Pose
}

func TestCompressKeyframesDeterministic(t *testing.T) {
	s, poses := wavePoses(t, 100)
	cfg := CompressConfig{Tolerance: 0.01}
	k1 := CompressKeyframes(s, poses, 30, cfg)
	k2 := CompressKeyframes(s, poses, 30, cfg)
	if len(k1) != len(k2) {
		t.Fatalf("non-deterministic key count: %d vs %d", len(k1), len(k2))
	}
	for i := range k1 {
		if k1[i].Time != k2[i].Time {
			t.Fatalf("key %d time differs", i)
		}
		dev, _ := PoseDeviation(k1[i].Pose, k2[i].Pose)
		if dev != 0 {
			t.Fatalf("key %d pose differs", i)
		}
	}
}

func TestSmoothPosesReducesJitter(t *testing.T) {
	s, poses := wavePoses(t, 50)
	// Inject jitter on one frame (deep-copy rotations to avoid aliasing).
	jittered := make([]Pose, len(poses))
	copy(jittered, poses)
	j := jittered[25]
	j.Rotations = append([]Quat(nil), j.Rotations...)
	j.Rotations[s.Index("Spine")] = QuatFromAxisAngle(Vec3{Z: 1}, 2.0)
	jittered[25] = j

	smoothed := SmoothPoses(s, jittered, SmoothConfig{Window: 3})
	dev, _ := PoseDeviation(smoothed[25], poses[25])
	rawDev, _ := PoseDeviation(jittered[25], poses[25])
	if dev >= rawDev {
		t.Fatalf("smoothing did not reduce deviation: %v >= %v", dev, rawDev)
	}
	if len(smoothed) != len(poses) {
		t.Fatalf("smoothed length = %d, want %d", len(smoothed), len(poses))
	}
}
