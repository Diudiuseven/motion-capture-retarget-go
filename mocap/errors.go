package mocap

import "fmt"

// Error is a locatable pipeline error: it carries the pipeline stage,
// the frame index (or -1) and the joint name (or "") involved.
type Error struct {
	Op    string // pipeline stage, e.g. "validate", "repair", "retarget"
	Frame int    // frame index, -1 if not frame-specific
	Joint string // joint name, "" if not joint-specific
	Msg   string
}

func (e *Error) Error() string {
	loc := e.Op
	if e.Frame >= 0 {
		loc += fmt.Sprintf(" frame=%d", e.Frame)
	}
	if e.Joint != "" {
		loc += fmt.Sprintf(" joint=%q", e.Joint)
	}
	return fmt.Sprintf("mocap: %s: %s", loc, e.Msg)
}

func errf(op string, frame int, joint, format string, args ...any) *Error {
	return &Error{Op: op, Frame: frame, Joint: joint, Msg: fmt.Sprintf(format, args...)}
}
