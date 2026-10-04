package mocap

import (
	"math"
	"testing"
)

// bigSkeleton is a differently-proportioned skeleton with a different
// rest pose (arms rotated 90°) and different joint names.
func bigSkeleton() *Skeleton {
	return &Skeleton{
		Name: "big",
		Joints: []Joint{
			{Name: "hips", Parent: "", RestPos: Vec3{Y: 2}},
			{Name: "torso", Parent: "hips", RestPos: Vec3{Y: 1}},
			{Name: "hand", Parent: "torso", RestPos: Vec3{X: 0.6},
				RestRot: QuatFromAxisAngle(Vec3{Z: 1}, math.Pi/2)},
		},
	}
}

func boneMap() map[string]string {
	return map[string]string{"root": "hips", "spine": "torso", "arm": "hand"}
}

func TestRetargetPreservesWorldMotion(t *testing.T) {
	src := testSkeleton()
	dst := bigSkeleton()
	mustValidate(t, src)
	mustValidate(t, dst)
	rt, err := NewRetargeter(src, dst, boneMap())
	if err != nil {
		t.Fatalf("NewRetargeter: %v", err)
	}
	clip := makeClip(src, 30, 30)
	out, err := rt.Retarget(clip)
	if err != nil {
		t.Fatalf("Retarget: %v", err)
	}
	if len(out.Frames) != len(clip.Frames) {
		t.Fatalf("frame count changed")
	}
	// World-space motion delta of the arm must match the source's.
	srcRest, _ := FK(src, Frame{Rotations: map[string]Quat{}})
	dstRest, _ := FK(dst, Frame{Rotations: map[string]Quat{}})
	for i := 0; i < 30; i++ {
		sw, _ := FK(src, clip.Frames[i])
		dw, _ := FK(dst, out.Frames[i])
		srcDelta := sw[2].Mul(srcRest[2].Conj())
		dstDelta := dw[2].Mul(dstRest[2].Conj())
		if d := srcDelta.AngleTo(dstDelta); d > 1e-6 {
			t.Fatalf("frame %d: world motion delta differs by %v", i, d)
		}
	}
}

func TestRetargetScalesRootMotion(t *testing.T) {
	src := testSkeleton()
	dst := bigSkeleton()
	mustValidate(t, src)
	mustValidate(t, dst)
	rt, err := NewRetargeter(src, dst, boneMap())
	if err != nil {
		t.Fatalf("NewRetargeter: %v", err)
	}
	clip := makeClip(src, 10, 30) // root moves along Z at 0.01/frame
	out, err := rt.Retarget(clip)
	if err != nil {
		t.Fatalf("Retarget: %v", err)
	}
	// height ratio 2:1 -> root displacement doubled
	got := out.Frames[9].RootPos.Z
	want := 0.09 * 2
	if math.Abs(got-want) > 1e-9 {
		t.Fatalf("root displacement %v, want %v", got, want)
	}
}

func TestRetargetNoJointFlip(t *testing.T) {
	src := testSkeleton()
	dst := bigSkeleton()
	mustValidate(t, src)
	mustValidate(t, dst)
	rt, _ := NewRetargeter(src, dst, boneMap())
	clip := makeClip(src, 60, 30)
	out, err := rt.Retarget(clip)
	if err != nil {
		t.Fatalf("Retarget: %v", err)
	}
	// Consecutive local rotations must stay close (no sudden flips).
	for i := 1; i < len(out.Frames); i++ {
		d := out.Frames[i].Rotations["hand"].AngleTo(out.Frames[i-1].Rotations["hand"])
		if d > 0.2 {
			t.Fatalf("frame %d: joint flip, step %v rad", i, d)
		}
	}
}

func TestRetargetRejectsBadMap(t *testing.T) {
	src := testSkeleton()
	dst := bigSkeleton()
	mustValidate(t, src)
	mustValidate(t, dst)
	if _, err := NewRetargeter(src, dst, map[string]string{"spine": "torso"}); err == nil {
		t.Fatalf("expected error: target root not covered")
	}
	if _, err := NewRetargeter(src, dst, map[string]string{
		"root": "hips", "spine": "torso", "ghost": "hand",
	}); err == nil {
		t.Fatalf("expected error: unknown source joint")
	}
}

func TestCompressBoundedErrorAndDeterministic(t *testing.T) {
	skel := testSkeleton()
	mustValidate(t, skel)
	clip := makeClip(skel, 100, 30)
	const tol = 0.05
	c1, err := Compress(clip, tol)
	if err != nil {
		t.Fatalf("Compress: %v", err)
	}
	c2, err := Compress(clip, tol)
	if err != nil {
		t.Fatalf("Compress: %v", err)
	}
	if len(c1.Frames) >= len(clip.Frames) {
		t.Fatalf("no compression: %d of %d frames kept", len(c1.Frames), len(clip.Frames))
	}
	if len(c1.Frames) != len(c2.Frames) {
		t.Fatalf("non-deterministic compression: %d vs %d", len(c1.Frames), len(c2.Frames))
	}
	for i := range c1.Frames {
		if c1.Frames[i].Rotations["arm"] != c2.Frames[i].Rotations["arm"] {
			t.Fatalf("non-deterministic keyframe %d", i)
		}
	}
	// Resample compressed clip at original times; error must be <= tol.
	ki := 0
	for _, f := range clip.Frames {
		for ki+1 < len(c1.Frames) && c1.Frames[ki+1].Time < f.Time-1e-12 {
			ki++
		}
		a := c1.Frames[ki]
		b := a
		if ki+1 < len(c1.Frames) {
			b = c1.Frames[ki+1]
		}
		tt := 0.0
		if b.Time > a.Time {
			tt = (f.Time - a.Time) / (b.Time - a.Time)
		}
		q := a.Rotations["arm"].Slerp(b.Rotations["arm"], tt)
		if d := q.AngleTo(f.Rotations["arm"]); d > tol+1e-9 {
			t.Fatalf("t=%v: deviation %v exceeds tolerance %v", f.Time, d, tol)
		}
	}
}

func TestSmoothReducesJitter(t *testing.T) {
	skel := testSkeleton()
	mustValidate(t, skel)
	clip := makeClip(skel, 40, 30)
	// Add alternating jitter to the arm.
	for i := range clip.Frames {
		j := 0.03
		if i%2 == 1 {
			j = -0.03
		}
		q := clip.Frames[i].Rotations["arm"]
		clip.Frames[i].Rotations["arm"] = QuatFromAxisAngle(Vec3{Z: 1}, j).Mul(q).Norm()
	}
	jitterOf := func(c *Clip) float64 {
		total := 0.0
		for i := 2; i < len(c.Frames); i++ {
			d0 := c.Frames[i-1].Rotations["arm"].AngleTo(c.Frames[i-2].Rotations["arm"])
			d1 := c.Frames[i].Rotations["arm"].AngleTo(c.Frames[i-1].Rotations["arm"])
			total += math.Abs(d1 - d0)
		}
		return total
	}
	before := jitterOf(clip)
	after := jitterOf(Smooth(clip, 2))
	if after >= before {
		t.Fatalf("smoothing did not reduce jitter: %v -> %v", before, after)
	}
}

// makeLoopClip builds a clip whose pose returns near its start at frame k.
func makeLoopClip(skel *Skeleton, n, period int) *Clip {
	c := &Clip{Skeleton: skel, SampleRate: 30}
	for i := 0; i < n; i++ {
		a := 0.4 * math.Sin(2*math.Pi*float64(i)/float64(period))
		c.Frames = append(c.Frames, Frame{
			Time:    float64(i) / 30,
			RootPos: Vec3{Y: 1},
			Rotations: map[string]Quat{
				"root":  QuatIdentity(),
				"spine": QuatIdentity(),
				"arm":   QuatFromAxisAngle(Vec3{Z: 1}, a),
			},
		})
	}
	return c
}

func TestAlignLoopSeamless(t *testing.T) {
	skel := testSkeleton()
	mustValidate(t, skel)
	// 30-frame period, 46 frames: tail returns near start around frame 30+.
	clip := makeLoopClip(skel, 46, 30)
	out, err := AlignLoop(clip, LoopConfig{SearchWindow: 20, MaxError: 0.05})
	if err != nil {
		t.Fatalf("AlignLoop: %v", err)
	}
	first := out.Frames[0]
	last := out.Frames[len(out.Frames)-1]
	if d := poseError(skel, first, last); d > 1e-9 {
		t.Fatalf("seam deviation %v, want exact match", d)
	}
	// No visible jump anywhere near the seam.
	for i := len(out.Frames) - 10; i < len(out.Frames)-1; i++ {
		d := poseError(skel, out.Frames[i], out.Frames[i+1])
		if d > 0.1 {
			t.Fatalf("jump of %v at frame %d near seam", d, i)
		}
	}
}

func TestAlignLoopFailsWhenUnalignable(t *testing.T) {
	skel := testSkeleton()
	mustValidate(t, skel)
	clip := makeClip(skel, 30, 30) // monotonic rotation, never returns
	_, err := AlignLoop(clip, LoopConfig{SearchWindow: 10, MaxError: 0.01})
	if err == nil {
		t.Fatalf("expected alignment failure")
	}
	if err.(*Error).Op != "loop" {
		t.Fatalf("error not locatable: %v", err)
	}
}
