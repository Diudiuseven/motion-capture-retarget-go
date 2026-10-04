package mocap

import (
	"errors"
	"testing"
)

func streamSetup(t *testing.T) (*Retargeter, *Skeleton) {
	t.Helper()
	src, dst := buildSkeletons(t)
	r, err := NewRetargeter(src, dst, map[string]string{
		"Hips": "Hips", "Spine": "Spine", "Arm": "Arm",
	})
	if err != nil {
		t.Fatal(err)
	}
	return r, src
}

func normFrame(seq uint64, angle float64) NormalizedFrame {
	return NormalizedFrame{
		Sequence: seq,
		Rotations: map[string]Quat{
			"Hips":  IdentityQuat(),
			"Spine": QuatFromAxisAngle(Vec3{Z: 1}, angle),
			"Arm":   IdentityQuat(),
		},
		RootPos: Vec3{Y: 1},
		HasRoot: true,
	}
}

func TestStreamProcessesOutOfOrderAndDuplicates(t *testing.T) {
	r, _ := streamSetup(t)
	p := NewStreamProcessor(FrameCleaner{SampleRate: 30}, r, SmoothConfig{}, 30)
	var got []Pose
	p.OnEmit(func(po Pose) { got = append(got, po) })

	frames := []NormalizedFrame{
		normFrame(0, 0), normFrame(2, 0.2), normFrame(1, 0.1), // out of order
		normFrame(1, 0.1), // duplicate
		normFrame(3, 0.3),
	}
	for _, f := range frames {
		if err := p.Push(f); err != nil {
			t.Fatalf("push seq %d: %v", f.Sequence, err)
		}
	}
	if err := p.Flush(); err != nil {
		t.Fatal(err)
	}
	if len(got) != 4 {
		t.Fatalf("emitted %d poses, want 4", len(got))
	}
}

func TestStreamCheckpointResume(t *testing.T) {
	r, _ := streamSetup(t)
	mk := func() (*StreamProcessor, *[]Pose) {
		p := NewStreamProcessor(FrameCleaner{SampleRate: 30}, r, SmoothConfig{}, 30)
		got := &[]Pose{}
		p.OnEmit(func(po Pose) { *got = append(*got, po) })
		return p, got
	}
	p1, got1 := mk()
	for i := uint64(0); i < 5; i++ {
		if err := p1.Push(normFrame(i, float64(i)*0.1)); err != nil {
			t.Fatal(err)
		}
	}
	cp := p1.Checkpoint()
	if cp.NextSequence != 5 {
		t.Fatalf("checkpoint next = %d, want 5", cp.NextSequence)
	}
	// Simulate crash: new processor resumes from checkpoint.
	p2, got2 := mk()
	p2.Resume(cp)
	for i := cp.NextSequence; i < 10; i++ {
		if err := p2.Push(normFrame(i, float64(i)*0.1)); err != nil {
			t.Fatal(err)
		}
	}
	if err := p2.Flush(); err != nil {
		t.Fatal(err)
	}
	if len(*got1)+len(*got2) != 10 {
		t.Fatalf("total emitted = %d, want 10", len(*got1)+len(*got2))
	}
	// Resumed output must match continuous processing.
	p3, got3 := mk()
	for i := uint64(0); i < 10; i++ {
		if err := p3.Push(normFrame(i, float64(i)*0.1)); err != nil {
			t.Fatal(err)
		}
	}
	p3.Flush()
	combined := append(*got1, *got2...)
	for i := range combined {
		dev, _ := PoseDeviation(combined[i], (*got3)[i])
		if dev > 1e-9 {
			t.Fatalf("frame %d deviates after resume: %v", i, dev)
		}
	}
}

func TestStreamMemoryBound(t *testing.T) {
	r, _ := streamSetup(t)
	p := NewStreamProcessor(FrameCleaner{SampleRate: 30}, r, SmoothConfig{Window: 2}, 30)
	bound := p.MemoryBound()
	// Push a far-future frame: reorder buffer must reject beyond limit.
	p.MaxReorder = 4
	for i := uint64(0); i < 3; i++ {
		if err := p.Push(normFrame(i, 0)); err != nil {
			t.Fatal(err)
		}
	}
	// Sequence 3 never arrives; buffer frames 4..7 (at the limit).
	for i := uint64(4); i < 8; i++ {
		if err := p.Push(normFrame(i, 0)); err != nil {
			t.Fatalf("push seq %d within limit failed: %v", i, err)
		}
	}
	// One more out-of-order frame exceeds the reorder limit.
	err := p.Push(normFrame(8, 0))
	var fe *FrameError
	if !errors.As(err, &fe) {
		t.Fatalf("error type = %T, want *FrameError", err)
	}
	var ge *GapError
	if !errors.As(err, &ge) {
		t.Fatalf("inner error type = %T, want *GapError", err)
	}
	if bound <= 0 {
		t.Fatal("memory bound must be positive")
	}
}

func TestStreamErrorLocatesFrame(t *testing.T) {
	r, _ := streamSetup(t)
	p := NewStreamProcessor(FrameCleaner{SampleRate: 30}, r, SmoothConfig{}, 30)
	bad := normFrame(0, 0)
	bad.Rotations["Ghost"] = IdentityQuat()
	err := p.Push(bad)
	var fe *FrameError
	if !errors.As(err, &fe) {
		t.Fatalf("error type = %T, want *FrameError", err)
	}
	if fe.Sequence != 0 {
		t.Fatalf("error sequence = %d, want 0", fe.Sequence)
	}
}
