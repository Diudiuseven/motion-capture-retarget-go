package mocap

import "math"

// StreamConfig controls streaming behaviour.
type StreamConfig struct {
	Repair       RepairConfig
	SmoothWindow int // smoothing half-width in frames (0 = none)
}

// StreamProcessor consumes raw frames incrementally with bounded
// memory: only a sliding window of recent frames is retained.
// It supports checkpoints so processing can resume after a failure.
type StreamProcessor struct {
	skel   *Skeleton
	conv   *axisConverter
	mapped map[string]string // canonical -> source joint name
	root   string
	cfg    StreamConfig
	rate   float64
	sink   func(Frame) error

	t0       float64 // grid origin (timestamp of first frame)
	hasT0    bool
	buf      map[int64]Frame // slot -> canonical frame, sliding window
	maxSlot  int64           // highest slot seen
	nextEmit int64           // next slot to emit
	pushed   int             // raw frames accepted so far
	emitted  int64
	horizon  int64 // slots beyond an emit point that must be known
}

// Checkpoint is a resumable snapshot of the processor state. It only
// contains the bounded sliding window, so its size does not grow with
// stream length.
type Checkpoint struct {
	NextIndex int     // index of the next raw frame to feed
	T0        float64 // grid origin to keep slot alignment identical
	HasT0     bool
	NextEmit  int64
	MaxSlot   int64
	Buf       map[int64]Frame
}

// NewStreamProcessor creates a processor that normalizes, repairs and
// smooths incoming frames, emitting finished output frames to sink.
// The raw capture supplies metadata only (axes, name map, rate);
// its Frames field is ignored.
func NewStreamProcessor(skel *Skeleton, meta *RawCapture, cfg StreamConfig, sink func(Frame) error) (*StreamProcessor, error) {
	if err := skel.Validate(); err != nil {
		return nil, err
	}
	if meta.SampleRate <= 0 {
		return nil, errf("stream", -1, "", "sample rate must be > 0")
	}
	conv, err := newAxisConverter(meta.Axes)
	if err != nil {
		return nil, errf("stream", -1, "", "source %q: %v", meta.Source, err)
	}
	mapped := make(map[string]string, len(meta.NameMap))
	for src, canon := range meta.NameMap {
		if skel.Index(canon) < 0 {
			return nil, errf("stream", -1, canon, "name map targets unknown canonical joint")
		}
		mapped[canon] = src
	}
	if cfg.Repair.MaxGapFrames < 0 {
		cfg.Repair.MaxGapFrames = 0
	}
	sp := &StreamProcessor{
		skel:   skel,
		conv:   conv,
		mapped: mapped,
		root:   skel.Joints[skel.RootIndex()].Name,
		cfg:    cfg,
		rate:   meta.SampleRate,
		sink:   sink,
		buf:    make(map[int64]Frame),
	}
	sp.horizon = int64(cfg.SmoothWindow) + int64(cfg.Repair.MaxGapFrames) + 1
	return sp, nil
}

// Push feeds one raw frame. Errors carry the raw frame index.
func (sp *StreamProcessor) Push(f RawFrame) error {
	idx := sp.pushed
	// Validate joint names before mutating any state.
	for src := range f.Joints {
		if _, ok := sp.canonical(src); !ok {
			return errf("stream", idx, src, "unknown source joint name")
		}
	}
	if !sp.hasT0 {
		sp.t0, sp.hasT0 = f.Timestamp, true
	}
	dt := 1 / sp.rate
	slot := int64(math.Round((f.Timestamp - sp.t0) / dt))
	if slot < sp.nextEmit {
		return errf("stream", idx, "",
			"frame timestamp %.4f maps to slot %d already emitted (too late)", f.Timestamp, slot)
	}
	if _, dup := sp.buf[slot]; !dup {
		cf := Frame{Time: sp.t0 + float64(slot)*dt, Rotations: make(map[string]Quat, len(sp.mapped))}
		for canon, src := range sp.mapped {
			if smp, ok := f.Joints[src]; ok {
				cf.Rotations[canon] = sp.conv.quat(smp.Rot)
				if canon == sp.root {
					cf.RootPos = sp.conv.pos(smp.Pos)
				}
			}
		}
		sp.buf[slot] = cf
	} // duplicate slot: keep the first sample, ignore the rest
	if slot > sp.maxSlot {
		sp.maxSlot = slot
	}
	sp.pushed++
	return sp.drain(false)
}

// canonical returns the canonical name for a source joint name.
func (sp *StreamProcessor) canonical(src string) (string, bool) {
	for canon, s := range sp.mapped {
		if s == src {
			return canon, true
		}
	}
	return "", false
}

// drain emits every slot that is final: all slots it may depend on
// (smoothing window + interpolation gap) are already observed.
func (sp *StreamProcessor) drain(flush bool) error {
	for {
		if !flush && sp.maxSlot < sp.nextEmit+sp.horizon {
			return nil
		}
		if flush && sp.nextEmit > sp.maxSlot {
			return nil
		}
		if err := sp.emitSlot(sp.nextEmit); err != nil {
			return err
		}
		delete(sp.buf, sp.nextEmit-int64(sp.cfg.SmoothWindow)-int64(sp.cfg.Repair.MaxGapFrames)-1)
		sp.nextEmit++
		sp.emitted++
	}
}

// materialize returns the frame at a slot, interpolating across gaps.
func (sp *StreamProcessor) materialize(slot int64) (Frame, error) {
	if f, ok := sp.buf[slot]; ok {
		return f, nil
	}
	// Find nearest captured slots around the missing slot.
	lo, hi := int64(-1), int64(-1)
	for s := slot - 1; s >= 0 && s >= slot-int64(sp.cfg.Repair.MaxGapFrames)-1; s-- {
		if _, ok := sp.buf[s]; ok {
			lo = s
			break
		}
	}
	for s := slot + 1; s <= sp.maxSlot; s++ {
		if _, ok := sp.buf[s]; ok {
			hi = s
			break
		}
	}
	if lo < 0 || hi < 0 {
		return Frame{}, errf("stream", int(slot), "",
			"cannot interpolate slot %d: no captured frame on one side", slot)
	}
	if gap := hi - lo - 1; gap > int64(sp.cfg.Repair.MaxGapFrames) {
		return Frame{}, errf("stream", int(slot), "",
			"gap of %d missing frames exceeds MaxGapFrames=%d", gap, sp.cfg.Repair.MaxGapFrames)
	}
	t := float64(slot-lo) / float64(hi-lo)
	return lerpFrame(sp.skel, sp.buf[lo], sp.buf[hi], t, sp.t0+float64(slot)/sp.rate), nil
}

// emitSlot materializes the smoothing window around slot and emits.
func (sp *StreamProcessor) emitSlot(slot int64) error {
	w := int64(sp.cfg.SmoothWindow)
	if w == 0 {
		f, err := sp.materialize(slot)
		if err != nil {
			return err
		}
		return sp.sink(f)
	}
	lo := slot - w
	if lo < 0 {
		lo = 0
	}
	hi := slot + w
	if hi > sp.maxSlot {
		hi = sp.maxSlot
	}
	window := make([]Frame, 0, hi-lo+1)
	for s := lo; s <= hi; s++ {
		f, err := sp.materialize(s)
		if err != nil {
			return err
		}
		window = append(window, f)
	}
	tmp := &Clip{Skeleton: sp.skel, Frames: window, SampleRate: sp.rate}
	smoothed := Smooth(tmp, sp.cfg.SmoothWindow)
	out := smoothed.Frames[slot-lo]
	out.Time = sp.t0 + float64(slot)/sp.rate
	return sp.sink(out)
}

// Checkpoint returns a snapshot from which processing can resume.
func (sp *StreamProcessor) Checkpoint() Checkpoint {
	cp := Checkpoint{
		NextIndex: sp.pushed,
		T0:        sp.t0,
		HasT0:     sp.hasT0,
		NextEmit:  sp.nextEmit,
		MaxSlot:   sp.maxSlot,
		Buf:       make(map[int64]Frame, len(sp.buf)),
	}
	for k, v := range sp.buf {
		cp.Buf[k] = v.Copy()
	}
	return cp
}

// Resume restores a previously taken checkpoint. The caller re-feeds
// raw frames starting at cp.NextIndex; output continues exactly as if
// the run had never been interrupted.
func (sp *StreamProcessor) Resume(cp Checkpoint) {
	sp.pushed = cp.NextIndex
	sp.t0, sp.hasT0 = cp.T0, cp.HasT0
	sp.nextEmit = cp.NextEmit
	sp.maxSlot = cp.MaxSlot
	sp.buf = make(map[int64]Frame, len(cp.Buf))
	for k, v := range cp.Buf {
		sp.buf[k] = v.Copy()
	}
}

// Flush finalizes the stream and emits remaining frames.
func (sp *StreamProcessor) Flush() error { return sp.drain(true) }
