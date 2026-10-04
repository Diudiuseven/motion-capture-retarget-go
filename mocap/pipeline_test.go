package mocap

import (
	"math"
	"testing"
)

// testSkeleton builds a 3-joint chain: root -> spine -> arm.
func testSkeleton() *Skeleton {
	return &Skeleton{
		Name: "test",
		Joints: []Joint{
			{Name: "root", Parent: "", RestPos: Vec3{Y: 1}},
			{Name: "spine", Parent: "root", RestPos: Vec3{Y: 0.5}},
			{Name: "arm", Parent: "spine", RestPos: Vec3{X: 0.3}},
		},
	}
}

func mustValidate(t *testing.T, s *Skeleton) {
	t.Helper()
	if err := s.Validate(); err != nil {
		t.Fatalf("Validate: %v", err)
	}
}

func TestSkeletonValidateOK(t *testing.T) {
	mustValidate(t, testSkeleton())
	s := testSkeleton()
	mustValidate(t, s)
	if s.Index("arm") != 2 || s.Index("nope") != -1 {
		t.Fatalf("bad index map")
	}
	if s.Parent(2) != 1 || s.Parent(0) != -1 {
		t.Fatalf("bad parent table")
	}
	// topological order: parents before children
	pos := map[int]int{}
	for i, j := range s.Order() {
		pos[j] = i
	}
	for i := 1; i < 3; i++ {
		if pos[s.Parent(i)] > pos[i] {
			t.Fatalf("order not topological")
		}
	}
}

func TestSkeletonValidateRejects(t *testing.T) {
	cases := []struct {
		name   string
		joints []Joint
	}{
		{"cycle", []Joint{
			{Name: "a", Parent: "c"},
			{Name: "b", Parent: "a"},
			{Name: "c", Parent: "b"},
		}},
		{"missing parent", []Joint{
			{Name: "a", Parent: ""},
			{Name: "b", Parent: "ghost"},
		}},
		{"duplicate name", []Joint{
			{Name: "a", Parent: ""},
			{Name: "a", Parent: "a"},
		}},
		{"two roots", []Joint{
			{Name: "a", Parent: ""},
			{Name: "b", Parent: ""},
		}},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			s := &Skeleton{Name: "bad", Joints: tc.joints}
			err := s.Validate()
			if err == nil {
				t.Fatalf("expected error for %s", tc.name)
			}
			me, ok := err.(*Error)
			if !ok {
				t.Fatalf("error is not locatable *Error: %T", err)
			}
			if me.Op == "" {
				t.Fatalf("error missing stage info: %v", err)
			}
		})
	}
}

// rawMeta returns capture metadata for a Z-up right-handed source.
func rawMeta() *RawCapture {
	return &RawCapture{
		Source:     "testsrc",
		SampleRate: 30,
		Axes:       AxisConvention{Up: "Z", Forward: "Y", Right: "X", LeftHand: true},
		NameMap: map[string]string{
			"hips":   "root",
			"spine1": "spine",
			"arm_L":  "arm",
		},
	}
}

func TestNormalizeAxesAndNames(t *testing.T) {
	skel := testSkeleton()
	raw := rawMeta()
	raw.Frames = []RawFrame{{
		Timestamp: 0,
		Joints: map[string]JointSample{
			// source is Z-up: root at (0, 0, 1) must become Y-up (0,1,0)
			"hips":   {Pos: Vec3{Z: 1}, Rot: QuatIdentity()},
			"spine1": {Rot: QuatIdentity()},
			"arm_L":  {Rot: QuatIdentity()},
		},
	}}
	clip, err := Normalize(raw, skel)
	if err != nil {
		t.Fatalf("Normalize: %v", err)
	}
	p := clip.Frames[0].RootPos
	if math.Abs(p.Y-1) > 1e-9 || math.Abs(p.X) > 1e-9 || math.Abs(p.Z) > 1e-9 {
		t.Fatalf("axis conversion wrong: got %+v", p)
	}
	if _, ok := clip.Frames[0].Rotations["arm"]; !ok {
		t.Fatalf("joint not renamed to canonical name")
	}
}

func TestNormalizeRotationConversion(t *testing.T) {
	skel := testSkeleton()
	// Right-handed Z-up source (no mirror): 90° around source Z (up)
	// must become 90° around internal Y.
	raw := rawMeta()
	raw.Axes = AxisConvention{Up: "Z", Forward: "-Y", Right: "X"}
	raw.Frames = []RawFrame{{
		Timestamp: 0,
		Joints: map[string]JointSample{
			"hips":   {Rot: QuatFromAxisAngle(Vec3{Z: 1}, math.Pi/2)},
			"spine1": {Rot: QuatIdentity()},
			"arm_L":  {Rot: QuatIdentity()},
		},
	}}
	clip, err := Normalize(raw, skel)
	if err != nil {
		t.Fatalf("Normalize: %v", err)
	}
	got := clip.Frames[0].Rotations["root"]
	want := QuatFromAxisAngle(Vec3{Y: 1}, math.Pi/2)
	if got.AngleTo(want) > 1e-6 {
		t.Fatalf("rotation conversion wrong: angle %v", got.AngleTo(want))
	}

	// Left-handed source: mirroring flips the rotation direction.
	raw.Axes = AxisConvention{Up: "Z", Forward: "Y", Right: "X", LeftHand: true}
	clip2, err := Normalize(raw, skel)
	if err != nil {
		t.Fatalf("Normalize: %v", err)
	}
	got2 := clip2.Frames[0].Rotations["root"]
	want2 := QuatFromAxisAngle(Vec3{Y: 1}, -math.Pi/2)
	if got2.AngleTo(want2) > 1e-6 {
		t.Fatalf("mirrored rotation conversion wrong: angle %v", got2.AngleTo(want2))
	}
}

func TestNormalizeRejectsBadInput(t *testing.T) {
	skel := testSkeleton()
	raw := rawMeta()
	raw.NameMap["hips"] = "nonexistent"
	if _, err := Normalize(raw, skel); err == nil {
		t.Fatalf("expected error for unknown canonical joint")
	}
	raw2 := rawMeta()
	delete(raw2.NameMap, "arm_L")
	if _, err := Normalize(raw2, skel); err == nil {
		t.Fatalf("expected error for unmapped canonical joint")
	}
	raw3 := rawMeta()
	raw3.Axes.LeftHand = false // axes are left-handed, declared right
	if _, err := Normalize(raw3, skel); err == nil {
		t.Fatalf("expected error for inconsistent handedness")
	}
}

// makeClip builds a clip with n frames, arm rotating at constant speed.
func makeClip(skel *Skeleton, n int, rate float64) *Clip {
	c := &Clip{Skeleton: skel, SampleRate: rate}
	for i := 0; i < n; i++ {
		a := float64(i) * 0.05
		c.Frames = append(c.Frames, Frame{
			Time:    float64(i) / rate,
			RootPos: Vec3{Y: 1, Z: float64(i) * 0.01},
			Rotations: map[string]Quat{
				"root":  QuatIdentity(),
				"spine": QuatIdentity(),
				"arm":   QuatFromAxisAngle(Vec3{Z: 1}, a),
			},
		})
	}
	return c
}

func TestRepairOutOfOrderAndDuplicates(t *testing.T) {
	skel := testSkeleton()
	mustValidate(t, skel)
	full := makeClip(skel, 20, 30)

	// Scramble: reverse order and duplicate some frames.
	messy := &Clip{Skeleton: skel, SampleRate: 30}
	for i := len(full.Frames) - 1; i >= 0; i-- {
		messy.Frames = append(messy.Frames, full.Frames[i])
		if i%3 == 0 {
			messy.Frames = append(messy.Frames, full.Frames[i]) // duplicate
		}
	}
	repaired, err := Repair(messy, RepairConfig{MaxGapFrames: 2, DriftTol: 0.01})
	if err != nil {
		t.Fatalf("Repair: %v", err)
	}
	if len(repaired.Frames) != len(full.Frames) {
		t.Fatalf("frame count %d, want %d", len(repaired.Frames), len(full.Frames))
	}
	for i := range full.Frames {
		d := repaired.Frames[i].Rotations["arm"].AngleTo(full.Frames[i].Rotations["arm"])
		if d > 1e-6 {
			t.Fatalf("frame %d deviates by %v after repair", i, d)
		}
	}
}

func TestRepairDroppedFramesInterpolated(t *testing.T) {
	skel := testSkeleton()
	mustValidate(t, skel)
	full := makeClip(skel, 20, 30)
	// Drop frames 5 and 6.
	dropped := &Clip{Skeleton: skel, SampleRate: 30}
	for i, f := range full.Frames {
		if i == 5 || i == 6 {
			continue
		}
		dropped.Frames = append(dropped.Frames, f)
	}
	repaired, err := Repair(dropped, RepairConfig{MaxGapFrames: 3, DriftTol: 0.01})
	if err != nil {
		t.Fatalf("Repair: %v", err)
	}
	if len(repaired.Frames) != 20 {
		t.Fatalf("frame count %d, want 20", len(repaired.Frames))
	}
	for _, i := range []int{5, 6} {
		got := repaired.Frames[i].Rotations["arm"]
		if got == (Quat{}) {
			t.Fatalf("frame %d interpolated as zero quaternion", i)
		}
		// constant-speed rotation: interpolation error must be tiny
		if d := got.AngleTo(full.Frames[i].Rotations["arm"]); d > 0.02 {
			t.Fatalf("frame %d interpolation error %v", i, d)
		}
	}
}

func TestRepairTimestampDrift(t *testing.T) {
	skel := testSkeleton()
	mustValidate(t, skel)
	full := makeClip(skel, 10, 30)
	drifted := &Clip{Skeleton: skel, SampleRate: 30}
	for i, f := range full.Frames {
		nf := f.Copy()
		nf.Time += 0.003 * math.Sin(float64(i)) // small jitter within tolerance
		drifted.Frames = append(drifted.Frames, nf)
	}
	repaired, err := Repair(drifted, RepairConfig{MaxGapFrames: 1, DriftTol: 0.01})
	if err != nil {
		t.Fatalf("Repair: %v", err)
	}
	for i, f := range repaired.Frames {
		want := float64(i) / 30
		if math.Abs(f.Time-want) > 1e-9 {
			t.Fatalf("frame %d time %v not snapped to grid %v", i, f.Time, want)
		}
	}
}

func TestRepairGapTooLarge(t *testing.T) {
	skel := testSkeleton()
	mustValidate(t, skel)
	full := makeClip(skel, 20, 30)
	gappy := &Clip{Skeleton: skel, SampleRate: 30}
	for i, f := range full.Frames {
		if i >= 5 && i <= 9 { // 5 consecutive missing frames
			continue
		}
		gappy.Frames = append(gappy.Frames, f)
	}
	_, err := Repair(gappy, RepairConfig{MaxGapFrames: 2, DriftTol: 0.01})
	if err == nil {
		t.Fatalf("expected gap error")
	}
	me := err.(*Error)
	if me.Frame < 0 {
		t.Fatalf("gap error not locatable to a frame: %v", err)
	}
}
