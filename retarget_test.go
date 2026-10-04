package mocap

import (
	"math"
	"testing"
)

// bigSkeleton is twice the size of smallSkeleton with a different rest
// pose (arms rotated) to simulate proportion and bind-pose mismatch.
func buildSkeletons(t *testing.T) (src, dst *Skeleton) {
	t.Helper()
	srcJoints := []Joint{
		{Name: "Hips", RestLocalPosition: Vec3{Y: 1.0}},
		{Name: "Spine", Parent: "Hips", RestLocalPosition: Vec3{Y: 0.3}},
		{Name: "Arm", Parent: "Spine", RestLocalPosition: Vec3{X: 0.2}},
	}
	dstJoints := []Joint{
		{Name: "Hips", RestLocalPosition: Vec3{Y: 2.0}},
		{Name: "Spine", Parent: "Hips", RestLocalPosition: Vec3{Y: 0.6}},
		// Target arm rest pose points up instead of sideways.
		{Name: "Arm", Parent: "Spine", RestLocalRotation: QuatFromAxisAngle(Vec3{Z: 1}, math.Pi/2), RestLocalPosition: Vec3{X: 0.4}},
	}
	var err error
	if src, err = NewSkeleton(srcJoints); err != nil {
		t.Fatal(err)
	}
	if dst, err = NewSkeleton(dstJoints); err != nil {
		t.Fatal(err)
	}
	return src, dst
}

func TestRetargetPreservesMotionSemantics(t *testing.T) {
	src, dst := buildSkeletons(t)
	r, err := NewRetargeter(src, dst, map[string]string{
		"Hips": "Hips", "Spine": "Spine", "Arm": "Arm",
	})
	if err != nil {
		t.Fatal(err)
	}
	// Source pose: spine rotated 45 deg about Z.
	p := src.NewRestPose()
	p.Rotations[src.Index("Spine")] = QuatFromAxisAngle(Vec3{Z: 1}, math.Pi/4)
	out := r.RetargetPose(p)

	// The world-space delta of the target spine must match the source's.
	srcGlob := src.GlobalRotations(p)
	srcRest := src.GlobalRotations(src.NewRestPose())
	srcDelta := srcGlob[src.Index("Spine")].Mul(srcRest[src.Index("Spine")].Conjugate())

	dstGlob := dst.GlobalRotations(out)
	dstRest := dst.GlobalRotations(dst.NewRestPose())
	dstDelta := dstGlob[dst.Index("Spine")].Mul(dstRest[dst.Index("Spine")].Conjugate())

	dev := srcDelta.Mul(dstDelta.Conjugate()).Angle()
	if dev > 1e-9 {
		t.Fatalf("world-space motion delta deviates by %v rad", dev)
	}
}

func TestRetargetScalesRootMotion(t *testing.T) {
	src, dst := buildSkeletons(t)
	r, err := NewRetargeter(src, dst, map[string]string{
		"Hips": "Hips", "Spine": "Spine", "Arm": "Arm",
	})
	if err != nil {
		t.Fatal(err)
	}
	p := src.NewRestPose()
	p.RootPos = Vec3{X: 1, Y: 1} // moved 1m sideways
	out := r.RetargetPose(p)
	// Target is 2x taller: root motion must scale by 2.
	if !almostEq(out.RootPos.X, 2, 1e-9) {
		t.Fatalf("scaled root X = %v, want 2", out.RootPos.X)
	}
}

func TestRetargetRestPoseMapsToRestPose(t *testing.T) {
	src, dst := buildSkeletons(t)
	r, err := NewRetargeter(src, dst, map[string]string{
		"Hips": "Hips", "Spine": "Spine", "Arm": "Arm",
	})
	if err != nil {
		t.Fatal(err)
	}
	out := r.RetargetPose(src.NewRestPose())
	rest := dst.NewRestPose()
	dev, _ := PoseDeviation(out, rest)
	if dev > 1e-9 {
		t.Fatalf("rest pose does not map to rest pose, deviation %v", dev)
	}
}

func TestNewRetargeterRejectsUnknownJoints(t *testing.T) {
	src, dst := buildSkeletons(t)
	if _, err := NewRetargeter(src, dst, map[string]string{"Hips": "Nope"}); err == nil {
		t.Fatal("expected error for unknown source joint")
	}
	if _, err := NewRetargeter(src, dst, map[string]string{"Nope": "Hips"}); err == nil {
		t.Fatal("expected error for unknown target joint")
	}
}
