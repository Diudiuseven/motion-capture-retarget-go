package mocap

import (
	"math"
	"testing"
)

func mkFrame(seq uint64, ts, angle float64) NormalizedFrame {
	return NormalizedFrame{
		Sequence: seq,
		Time:     ts,
		Rotations: map[string]Quat{
			"Hips": QuatFromAxisAngle(Vec3{Y: 1}, angle),
		},
		RootPos: Vec3{X: angle},
		HasRoot: true,
	}
}

func frameAngle(f NormalizedFrame) float64 {
	q := f.Rotations["Hips"]
	return math.Atan2(2*(q.W*q.Y), 1-2*q.Y*q.Y)
}

func TestCleanFramesDedupAndReorder(t *testing.T) {
	c := FrameCleaner{SampleRate: 30, MaxGapFrames: 10}
	ordered := []NormalizedFrame{mkFrame(0, 0, 0), mkFrame(1, 1.0/30, 0.1), mkFrame(2, 2.0/30, 0.2)}
	// Shuffled with a duplicate.
	messy := []NormalizedFrame{mkFrame(2, 2.0/30, 0.2), mkFrame(0, 0, 0), mkFrame(1, 1.0/30, 0.1), mkFrame(1, 99, 0.1)}
	gotOrdered, err := c.CleanFrames(ordered)
	if err != nil {
		t.Fatal(err)
	}
	gotMessy, err := c.CleanFrames(messy)
	if err != nil {
		t.Fatal(err)
	}
	if len(gotMessy) != 3 {
		t.Fatalf("got %d frames, want 3", len(gotMessy))
	}
	for i := range gotOrdered {
		if gotOrdered[i].Sequence != gotMessy[i].Sequence ||
			math.Abs(frameAngle(gotOrdered[i])-frameAngle(gotMessy[i])) > 1e-12 {
			t.Fatalf("frame %d differs between ordered and messy input", i)
		}
	}
}

func TestCleanFramesInterpolatesMissing(t *testing.T) {
	c := FrameCleaner{SampleRate: 30, MaxGapFrames: 10}
	in := []NormalizedFrame{mkFrame(0, 0, 0), mkFrame(2, 2.0/30, 0.2)}
	out, err := c.CleanFrames(in)
	if err != nil {
		t.Fatal(err)
	}
	if len(out) != 3 {
		t.Fatalf("got %d frames, want 3", len(out))
	}
	mid := out[1]
	if mid.Sequence != 1 {
		t.Fatalf("interpolated sequence = %d, want 1", mid.Sequence)
	}
	// Missing frame must be interpolated, not zero.
	if a := frameAngle(mid); math.Abs(a-0.1) > 1e-9 {
		t.Fatalf("interpolated angle = %v, want ~0.1 (not zero)", a)
	}
	if !almostEq(mid.RootPos.X, 0.1, 1e-9) {
		t.Fatalf("interpolated root X = %v, want 0.1", mid.RootPos.X)
	}
}

func TestCleanFramesGapTooLarge(t *testing.T) {
	c := FrameCleaner{SampleRate: 30, MaxGapFrames: 2}
	in := []NormalizedFrame{mkFrame(0, 0, 0), mkFrame(10, 10.0/30, 1)}
	_, err := c.CleanFrames(in)
	ge, ok := err.(*GapError)
	if !ok {
		t.Fatalf("error type = %T, want *GapError", err)
	}
	if ge.AfterSequence != 0 || ge.Missing != 9 {
		t.Fatalf("gap error not locatable: %+v", ge)
	}
}

func TestCleanFramesCorrectsTimestampDrift(t *testing.T) {
	c := FrameCleaner{SampleRate: 30, MaxGapFrames: 10}
	// Timestamps drift by +0.5ms per frame.
	in := []NormalizedFrame{
		mkFrame(0, 0.0000, 0),
		mkFrame(1, 0.0338, 0.1),
		mkFrame(2, 0.0676, 0.2),
	}
	out, err := c.CleanFrames(in)
	if err != nil {
		t.Fatal(err)
	}
	for i, f := range out {
		want := float64(i) / 30
		if math.Abs(f.Time-want) > 1e-12 {
			t.Fatalf("frame %d time = %v, want %v (drift not corrected)", i, f.Time, want)
		}
	}
}
