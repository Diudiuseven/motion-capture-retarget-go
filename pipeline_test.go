package mocap

import (
	"math"
	"testing"
)

// TestEndToEndPipeline runs the full pipeline: raw source frames with
// drops, duplicates, reordering and drift -> normalize -> clean ->
// retarget -> smooth -> loop-align -> compress -> clip, and verifies
// reproducibility (identical output for identical input).
func TestEndToEndPipeline(t *testing.T) {
	src, dst := buildSkeletons(t)
	spec := SourceSpec{
		NameMap: map[string]string{
			"bone_hips":  "Hips",
			"bone_spine": "Spine",
			"bone_arm":   "Arm",
		},
		Convention: AxisConvention{Forward: Vec3{X: 1}, Up: Vec3{Z: 1}, RightHanded: true},
		SampleRate: 30,
	}

	const n = 60
	makeRaw := func() []RawFrame {
		var frames []RawFrame
		for i := 0; i < n; i++ {
			if i == 17 || i == 38 {
				continue // dropped frames
			}
			angle := 0.4 * math.Sin(2*math.Pi*float64(i)/n)
			frames = append(frames, RawFrame{
				Sequence:  uint64(i),
				Timestamp: float64(i)/30 + float64(i)*0.0004, // drifting
				Rotations: map[string]Quat{
					"bone_hips":  IdentityQuat(),
					"bone_spine": QuatFromAxisAngle(Vec3{Y: 1}, angle), // source up axis
					"bone_arm":   IdentityQuat(),
				},
				RootPos: Vec3{Z: 1}, // source up = 1m
				HasRoot: true,
			})
		}
		// Duplicate and shuffle deterministically.
		frames = append(frames, frames[10])
		for i, j := 0, len(frames)-1; i < j; i, j = i+1, j-1 {
			frames[i], frames[j] = frames[j], frames[i]
		}
		return frames
	}

	run := func() []Keyframe {
		var norm []NormalizedFrame
		for _, rf := range makeRaw() {
			nf, err := NormalizeFrame(spec, rf)
			if err != nil {
				t.Fatalf("normalize: %v", err)
			}
			norm = append(norm, nf)
		}
		cleaner := FrameCleaner{SampleRate: 30, MaxGapFrames: 5}
		clean, err := cleaner.CleanFrames(norm)
		if err != nil {
			t.Fatalf("clean: %v", err)
		}
		if len(clean) != n {
			t.Fatalf("cleaned frame count = %d, want %d", len(clean), n)
		}
		r, err := NewRetargeter(src, dst, map[string]string{
			"Hips": "Hips", "Spine": "Spine", "Arm": "Arm",
		})
		if err != nil {
			t.Fatal(err)
		}
		poses := make([]Pose, len(clean))
		for i, f := range clean {
			p := src.NewRestPose()
			for name, q := range f.Rotations {
				p.Rotations[src.Index(name)] = q
			}
			p.RootPos = f.RootPos
			poses[i] = r.RetargetPose(p)
		}
		poses = SmoothPoses(dst, poses, SmoothConfig{Window: 1})
		poses, err = AlignLoop(dst, poses, LoopConfig{
			MaxSeamDeviation: 0.5, MaxRootSeamDistance: 0.01, BlendFrames: 6,
		})
		if err != nil {
			t.Fatalf("loop align: %v", err)
		}
		return CompressKeyframes(dst, poses, 30, CompressConfig{Tolerance: 0.01})
	}

	k1 := run()
	k2 := run()
	if len(k1) != len(k2) {
		t.Fatalf("non-reproducible pipeline: %d vs %d keys", len(k1), len(k2))
	}
	for i := range k1 {
		dev, _ := PoseDeviation(k1[i].Pose, k2[i].Pose)
		if dev != 0 || k1[i].Time != k2[i].Time {
			t.Fatalf("key %d not reproducible", i)
		}
	}
	if len(k1) == 0 || len(k1) >= n {
		t.Fatalf("unexpected key count %d for %d frames", len(k1), n)
	}
}
