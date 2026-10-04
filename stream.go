package mocap

import "fmt"

// StreamProcessor consumes normalized frames incrementally with bounded
// memory and supports checkpoint/resume.
type StreamProcessor struct {
	Cleaner   FrameCleaner
	Retarget  *Retargeter
	Smooth    SmoothConfig
	FrameRate float64

	// MaxReorder is the maximum number of out-of-order frames buffered
	// while waiting for a missing sequence number.
	MaxReorder int

	pending map[uint64]NormalizedFrame // reorder buffer
	window  []Pose                     // trailing smoothing window
	next    uint64                     // next expected sequence number
	emitted int
	emitFn  func(Pose)
	flushed bool
}

// Checkpoint captures the processor state for resume.
type Checkpoint struct {
	NextSequence uint64
	FramesDone   int
}

// FrameError locates a streaming failure to a specific frame.
type FrameError struct {
	Sequence uint64
	Err      error
}

func (e *FrameError) Error() string {
	return fmt.Sprintf("stream frame %d: %v", e.Sequence, e.Err)
}

// Unwrap returns the underlying error.
func (e *FrameError) Unwrap() error { return e.Err }

// NewStreamProcessor creates a processor with the given pipeline stages.
func NewStreamProcessor(c FrameCleaner, r *Retargeter, sm SmoothConfig, rate float64) *StreamProcessor {
	return &StreamProcessor{
		Cleaner:    c,
		Retarget:   r,
		Smooth:     sm,
		FrameRate:  rate,
		MaxReorder: 64,
		pending:    make(map[uint64]NormalizedFrame),
	}
}

// OnEmit registers the callback receiving retargeted poses.
func (p *StreamProcessor) OnEmit(fn func(Pose)) { p.emitFn = fn }

// Push ingests one frame. Frames may arrive out of order or duplicated;
// duplicates are dropped and out-of-order frames are buffered within the
// reorder limit. Completed output poses are delivered via OnEmit.
func (p *StreamProcessor) Push(f NormalizedFrame) error {
	if p.flushed {
		return &FrameError{Sequence: f.Sequence, Err: fmt.Errorf("push after flush")}
	}
	if f.Sequence < p.next {
		return nil // duplicate of an already-processed frame
	}
	if _, dup := p.pending[f.Sequence]; dup {
		return nil // duplicate already buffered
	}
	p.pending[f.Sequence] = f
	if len(p.pending) > p.reorderLimit() {
		return &FrameError{Sequence: f.Sequence, Err: &GapError{
			AfterSequence: p.next - 1,
			Missing:       int(f.Sequence - p.next),
		}}
	}
	return p.drain()
}

// drain emits every frame that is ready in sequence order.
func (p *StreamProcessor) drain() error {
	for {
		f, ok := p.pending[p.next]
		if !ok {
			return nil
		}
		delete(p.pending, p.next)
		if err := p.process(f); err != nil {
			return err
		}
		p.next++
	}
}

// process retargets one frame and emits it once the smoothing window is
// filled, keeping memory bounded by the window size.
func (p *StreamProcessor) process(f NormalizedFrame) error {
	src := Pose{Rotations: make([]Quat, p.Retarget.Source.NumJoints())}
	rest := p.Retarget.Source.NewRestPose()
	copy(src.Rotations, rest.Rotations)
	for name, q := range f.Rotations {
		ix := p.Retarget.Source.Index(name)
		if ix < 0 {
			return &FrameError{Sequence: f.Sequence,
				Err: fmt.Errorf("joint %q not in source skeleton", name)}
		}
		src.Rotations[ix] = q
	}
	if f.HasRoot {
		src.RootPos = f.RootPos
	} else {
		src.RootPos = rest.RootPos
	}
	p.window = append(p.window, p.Retarget.RetargetPose(src))
	return p.emitReady(false)
}

// emitReady emits poses whose smoothing window is complete. With
// half-width w, pose i is ready once pose i+w has arrived; on flush all
// remaining poses are emitted with clamped windows.
func (p *StreamProcessor) emitReady(flush bool) error {
	w := p.Smooth.Window
	if w < 0 {
		w = 0
	}
	for len(p.window) > 0 {
		ready := len(p.window) > w
		if flush {
			ready = len(p.window) > 0
		}
		if !ready {
			return nil
		}
		if p.emitFn != nil {
			p.emitFn(smoothedHead(p.window, w))
		}
		p.window = p.window[1:]
		p.emitted++
	}
	return nil
}

// smoothedHead averages the head pose with up to w following poses,
// mirroring SmoothPoses for a trailing window.
func smoothedHead(window []Pose, w int) Pose {
	if w <= 0 || len(window) == 1 {
		return window[0]
	}
	hi := w
	if hi > len(window)-1 {
		hi = len(window) - 1
	}
	head := window[0]
	out := Pose{Rotations: make([]Quat, len(head.Rotations))}
	for j := range out.Rotations {
		ref := head.Rotations[j]
		var acc Quat
		for k := 0; k <= hi; k++ {
			q := window[k].Rotations[j]
			if q.X*ref.X+q.Y*ref.Y+q.Z*ref.Z+q.W*ref.W < 0 {
				q = Quat{-q.X, -q.Y, -q.Z, -q.W}
			}
			acc.X += q.X
			acc.Y += q.Y
			acc.Z += q.Z
			acc.W += q.W
		}
		out.Rotations[j] = acc.Normalized()
	}
	for k := 0; k <= hi; k++ {
		out.RootPos = out.RootPos.Add(window[k].RootPos)
	}
	out.RootPos = out.RootPos.Scale(1 / float64(hi+1))
	return out
}

// Flush finalizes the stream and emits remaining poses.
func (p *StreamProcessor) Flush() error {
	if p.flushed {
		return nil
	}
	if len(p.pending) > 0 {
		return &FrameError{Sequence: p.next, Err: &GapError{
			AfterSequence: p.next - 1,
			Missing:       len(p.pending),
		}}
	}
	p.flushed = true
	return p.emitReady(true)
}

// Checkpoint returns the current resumable state.
func (p *StreamProcessor) Checkpoint() Checkpoint {
	return Checkpoint{NextSequence: p.next, FramesDone: p.emitted}
}

// Resume restores the processor to a checkpoint taken earlier. Frames
// with sequence numbers below the checkpoint are ignored on Push, so a
// caller can replay the source from any point at or after the
// checkpoint.
func (p *StreamProcessor) Resume(cp Checkpoint) {
	p.next = cp.NextSequence
	p.emitted = cp.FramesDone
	p.pending = make(map[uint64]NormalizedFrame)
	p.window = nil
	p.flushed = false
}

// MemoryBound returns the maximum number of buffered frames.
func (p *StreamProcessor) MemoryBound() int {
	return p.reorderLimit() + p.Smooth.Window + 1
}

func (p *StreamProcessor) reorderLimit() int {
	if p.MaxReorder <= 0 {
		return 64
	}
	return p.MaxReorder
}
