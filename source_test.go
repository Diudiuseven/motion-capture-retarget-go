package mocap

import (
	"math"
	"testing"
)

func TestNormalizeFrameNameMap(t *testing.T) {
	spec := SourceSpec{
		NameMap:    map[string]string{"mixamorig:Hips": "Hips", "mixamorig:Head": "Head"},
		Convention: DefaultConvention,
		SampleRate: 30,
	}
	raw := RawFrame{
		Sequence:  7,
		Timestamp: 0.25,
		Rotations: map[string]Quat{
			"mixamorig:Hips": QuatFromAxisAngle(Vec3{X: 1}, 0.5),
			"mixamorig:Head": IdentityQuat(),
			"unknownBone":    IdentityQuat(), // dropped
		},
	}
	nf, err := NormalizeFrame(spec, raw)
	if err != nil {
		t.Fatalf("normalize failed: %v", err)
	}
	if len(nf.Rotations) != 2 {
		t.Fatalf("got %d rotations, want 2 (unmapped dropped)", len(nf.Rotations))
	}
	if _, ok := nf.Rotations["Hips"]; !ok {
		t.Fatal("mapped joint Hips missing")
	}
}

func TestNormalizeFrameAxisConversion(t *testing.T) {
	// Source: right-handed, Z-up, X-forward.
	spec := SourceSpec{
		NameMap:    map[string]string{"root": "Hips"},
		Convention: AxisConvention{Forward: Vec3{X: 1}, Up: Vec3{Z: 1}, RightHanded: true},
		SampleRate: 60,
	}
	raw := RawFrame{
		Sequence:  0,
		Rotations: map[string]Quat{},
		RootPos:   Vec3{X: 1, Y: 0, Z: 2}, // 1 forward, 2 up in source
		HasRoot:   true,
	}
	nf, err := NormalizeFrame(spec, raw)
	if err != nil {
		t.Fatalf("normalize failed: %v", err)
	}
	// Internal: forward -> +Z, up -> +Y.
	if !almostEq(nf.RootPos.Z, 1, 1e-9) || !almostEq(nf.RootPos.Y, 2, 1e-9) {
		t.Fatalf("axis conversion wrong: %+v", nf.RootPos)
	}
}

func TestNormalizeFrameRejectsZeroQuat(t *testing.T) {
	spec := SourceSpec{
		NameMap:    map[string]string{"a": "A"},
		Convention: DefaultConvention,
		SampleRate: 30,
	}
	raw := RawFrame{Sequence: 42, Rotations: map[string]Quat{"a": {}}}
	_, err := NormalizeFrame(spec, raw)
	pe, ok := err.(*ParseError)
	if !ok {
		t.Fatalf("error type = %T, want *ParseError", err)
	}
	if pe.Sequence != 42 || pe.Joint != "a" {
		t.Fatalf("error not locatable: %+v", pe)
	}
}

func TestConversionQuatRotationConsistency(t *testing.T) {
	// A rotation about source-up must become a rotation about internal-up.
	spec := AxisConvention{Forward: Vec3{X: 1}, Up: Vec3{Z: 1}, RightHanded: true}
	conv := ConversionQuat(spec)
	srcRot := QuatFromAxisAngle(Vec3{Z: 1}, math.Pi/3) // about source up
	internal := conv.Mul(srcRot).Mul(conv.Conjugate())
	// The converted rotation must be about +Y (internal up).
	axis := Vec3{internal.X, internal.Y, internal.Z}.Normalized()
	if axis.Sub(Vec3{Y: 1}).Len() > 1e-6 {
		t.Fatalf("converted axis = %+v, want +Y", axis)
	}
}
