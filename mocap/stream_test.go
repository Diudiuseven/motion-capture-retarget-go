package mocap

import (
	"math"
	"testing"
)

// rawFrames builds n raw frames with a steadily rotating arm joint.
func rawFrames(n int, rate float64) []RawFrame {
	out := make([]RawFrame, n)
	for i := 0; i < n; i++ {
		out[i] = RawFrame{
			Timestamp: float64(i) / rate,
			Joints: map[string]JointSample{
				"hips":   {Pos: Vec3{Z: 1}, Rot: QuatIdentity()},
				"spine1": {Rot: QuatIdentity()},
				"arm_L":  {Rot: QuatFromAxisAngle(Vec3{Z: 1}, float64(i)*0.05)},
			},
		}
	}
	return out
}

func streamConfig() StreamConfig {
	return StreamConfig{
		Repair:       RepairConfig{MaxGapFrames: 2, DriftTol: 0.01},
		SmoothWindow: 1,
	}
}

func TestStreamMatchesBatch(t *testing.T) {
	skel := testSkeleton()
	mustValidate(t, skel)
	meta := rawMeta()
	const n = 200
	raw := rawFrames(n, meta.SampleRate)

	// Batch reference: normalize + repair + smooth.
	full := &RawCapture{
		Source: meta.Source, SampleRate: meta.SampleRate,
		Axes: meta.Axes, NameMap: meta.NameMap, Frames: raw,
	}
	clip, err := Normalize(full, skel)
	if err != nil {
		t.Fatalf("Normalize: %v", err)
	}
	rep, err := Repair(clip, RepairConfig{MaxGapFrames: 2, DriftTol: 0.01})
	if err != nil {
		t.Fatalf("Repair: %v", err)
	}
	want := Smooth(rep, 1)

	// Streaming.
	var got []Frame
	sp, err := NewStreamProcessor(skel, meta, streamConfig(), func(f Frame) error {
		got = append(got, f)
		return nil
	})
	if err != nil {
		t.Fatalf("NewStreamProcessor: %v", err)
	}
	emittedBeforeFlush := 0
	for _, rf := range raw {
		if err := sp.Push(rf); err != nil {
			t.Fatalf("Push: %v", err)
		}
		emittedBeforeFlush = len(got)
	}
	if emittedBeforeFlush == 0 {
		t.Fatalf("no frames emitted before flush: not streaming")
	}
	if err := sp.Flush(); err != nil {
		t.Fatalf("Flush: %v", err)
	}
	if len(got) != len(want.Frames) {
		t.Fatalf("streamed %d frames, want %d", len(got), len(want.Frames))
	}
	for i := range want.Frames {
		d := got[i].Rotations["arm"].AngleTo(want.Frames[i].Rotations["arm"])
		if d > 1e-6 {
			t.Fatalf("frame %d: stream/batch mismatch %v", i, d)
		}
	}
}

func TestStreamBoundedBuffer(t *testing.T) {
	skel := testSkeleton()
	mustValidate(t, skel)
	meta := rawMeta()
	sp, err := NewStreamProcessor(skel, meta, streamConfig(), func(Frame) error { return nil })
	if err != nil {
		t.Fatalf("NewStreamProcessor: %v", err)
	}
	const n = 5000
	maxBuf := 0
	for _, rf := range rawFrames(n, meta.SampleRate) {
		if err := sp.Push(rf); err != nil {
			t.Fatalf("Push: %v", err)
		}
		if len(sp.buf) > maxBuf {
			maxBuf = len(sp.buf)
		}
	}
	// Buffer must stay within window + gap horizon, independent of n.
	limit := 2*(streamConfig().SmoothWindow+streamConfig().Repair.MaxGapFrames+1) + 2
	if maxBuf > limit {
		t.Fatalf("buffer grew to %d frames, limit %d", maxBuf, limit)
	}
}

func TestStreamErrorLocatesFrame(t *testing.T) {
	skel := testSkeleton()
	mustValidate(t, skel)
	meta := rawMeta()
	sp, err := NewStreamProcessor(skel, meta, streamConfig(), func(Frame) error { return nil })
	if err != nil {
		t.Fatalf("NewStreamProcessor: %v", err)
	}
	raw := rawFrames(10, meta.SampleRate)
	// Corrupt frame 7 with an unknown joint name.
	raw[7].Joints["elbow_L"] = JointSample{Rot: QuatIdentity()}
	var gotErr error
	for i, rf := range raw {
		if err := sp.Push(rf); err != nil {
			gotErr = err
			if gotErr.(*Error).Frame != i {
				t.Fatalf("error frame %d, want %d", gotErr.(*Error).Frame, i)
			}
			break
		}
	}
	if gotErr == nil {
		t.Fatalf("expected error for unknown joint")
	}
}

func TestStreamCheckpointResume(t *testing.T) {
	skel := testSkeleton()
	mustValidate(t, skel)
	meta := rawMeta()
	raw := rawFrames(120, meta.SampleRate)

	// Uninterrupted reference run.
	var want []Frame
	sp1, _ := NewStreamProcessor(skel, meta, streamConfig(), func(f Frame) error {
		want = append(want, f)
		return nil
	})
	for _, rf := range raw {
		if err := sp1.Push(rf); err != nil {
			t.Fatalf("Push: %v", err)
		}
	}
	if err := sp1.Flush(); err != nil {
		t.Fatalf("Flush: %v", err)
	}

	// Run with a simulated crash at frame 60, resumed from checkpoint.
	var got []Frame
	sp2, _ := NewStreamProcessor(skel, meta, streamConfig(), func(f Frame) error {
		got = append(got, f)
		return nil
	})
	var cp Checkpoint
	for i := 0; i < 40; i++ {
		if err := sp2.Push(raw[i]); err != nil {
			t.Fatalf("Push: %v", err)
		}
	}
	cp = sp2.Checkpoint()
	// "crash": abandon sp2, start a fresh processor from the checkpoint.
	sp3, _ := NewStreamProcessor(skel, meta, streamConfig(), func(f Frame) error {
		got = append(got, f)
		return nil
	})
	sp3.Resume(cp)
	for i := cp.NextIndex; i < len(raw); i++ {
		if err := sp3.Push(raw[i]); err != nil {
			t.Fatalf("Push after resume: %v", err)
		}
	}
	if err := sp3.Flush(); err != nil {
		t.Fatalf("Flush after resume: %v", err)
	}

	if len(got) != len(want) {
		t.Fatalf("resumed run emitted %d frames, want %d", len(got), len(want))
	}
	for i := range want {
		d := got[i].Rotations["arm"].AngleTo(want[i].Rotations["arm"])
		if d > 1e-6 || math.Abs(got[i].Time-want[i].Time) > 1e-12 {
			t.Fatalf("frame %d diverged after resume", i)
		}
	}
}

func TestStreamDuplicateAndDriftedFrames(t *testing.T) {
	skel := testSkeleton()
	mustValidate(t, skel)
	meta := rawMeta()
	sp, err := NewStreamProcessor(skel, meta, streamConfig(), func(Frame) error { return nil })
	if err != nil {
		t.Fatalf("NewStreamProcessor: %v", err)
	}
	raw := rawFrames(30, meta.SampleRate)
	var messy []RawFrame
	for i, rf := range raw {
		messy = append(messy, rf)
		if i%4 == 0 {
			messy = append(messy, rf) // duplicate
		}
	}
	// Drift every timestamp slightly within tolerance.
	for i := range messy {
		messy[i].Timestamp += 0.002
	}
	for _, rf := range messy {
		if err := sp.Push(rf); err != nil {
			t.Fatalf("Push: %v", err)
		}
	}
	if err := sp.Flush(); err != nil {
		t.Fatalf("Flush: %v", err)
	}
}
