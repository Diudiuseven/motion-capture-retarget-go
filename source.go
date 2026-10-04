package mocap

import "fmt"

// AxisConvention describes the coordinate system of a capture source.
type AxisConvention struct {
	Forward     Vec3 // world forward in source coordinates
	Up          Vec3 // world up in source coordinates
	RightHanded bool // whether the source uses a right-handed system
}

// DefaultConvention is the internal convention: right-handed, Y-up, Z-forward.
var DefaultConvention = AxisConvention{
	Forward:     Vec3{Z: 1},
	Up:          Vec3{Y: 1},
	RightHanded: true,
}

// SourceSpec describes how to normalize one capture source.
type SourceSpec struct {
	// NameMap maps source joint names to internal joint names.
	// Unmapped source joints are dropped.
	NameMap map[string]string
	// Convention is the source coordinate convention.
	Convention AxisConvention
	// SampleRate is the nominal source sample rate in Hz.
	SampleRate float64
}

// RawFrame is one unprocessed capture frame from a source.
type RawFrame struct {
	Timestamp float64 // seconds, may drift or be out of order
	Sequence  uint64  // monotonically increasing capture sequence number
	// Rotations maps source joint names to local rotations.
	Rotations map[string]Quat
	// RootPos is the root translation in source coordinates.
	RootPos Vec3
	// HasRoot reports whether RootPos carries real data.
	HasRoot bool
}

// ParseError locates a problem in a specific raw frame.
type ParseError struct {
	Sequence uint64
	Joint    string
	Reason   string
}

func (e *ParseError) Error() string {
	if e.Joint != "" {
		return fmt.Sprintf("frame %d, joint %q: %s", e.Sequence, e.Joint, e.Reason)
	}
	return fmt.Sprintf("frame %d: %s", e.Sequence, e.Reason)
}

// NormalizedFrame is a frame in the internal representation.
type NormalizedFrame struct {
	Time      float64 // corrected timestamp, seconds
	Sequence  uint64
	Rotations map[string]Quat // keyed by internal joint name
	RootPos   Vec3
	HasRoot   bool
}

// ConversionQuat returns the rotation converting source coordinates to
// the internal convention. For left-handed sources it also flips the
// handedness by mirroring across the source's forward-up plane.
func ConversionQuat(c AxisConvention) Quat {
	srcF := c.Forward.Normalized()
	srcU := c.Up.Normalized()
	if srcF.Len() < 1e-9 || srcU.Len() < 1e-9 {
		return IdentityQuat()
	}
	// Orthonormalize: remove the forward component from up.
	srcU = srcU.Sub(srcF.Scale(srcF.Dot(srcU))).Normalized()
	if srcF.Dot(DefaultConvention.Forward) > 0.999999 &&
		srcU.Dot(DefaultConvention.Up) > 0.999999 && c.RightHanded {
		return IdentityQuat()
	}
	// Build source basis (right = up x forward for right-handed).
	var srcR Vec3
	if c.RightHanded {
		srcR = srcU.Cross(srcF)
	} else {
		srcR = srcF.Cross(srcU)
	}
	// Rotation matrix whose columns map internal axes to source axes.
	// We want the inverse: source -> internal.
	m := [9]float64{
		srcR.X, srcU.X, srcF.X,
		srcR.Y, srcU.Y, srcF.Y,
		srcR.Z, srcU.Z, srcF.Z,
	}
	return QuatFromMatrix(m).Conjugate().Normalized()
}

// NormalizeFrame converts a raw frame to the internal representation:
// applies the joint name map, converts rotations and root position to
// the internal coordinate convention, and validates quaternion data.
func NormalizeFrame(spec SourceSpec, f RawFrame) (NormalizedFrame, error) {
	if spec.SampleRate <= 0 {
		return NormalizedFrame{}, &ParseError{Sequence: f.Sequence,
			Reason: "source sample rate must be positive"}
	}
	conv := ConversionQuat(spec.Convention)
	convInv := conv.Conjugate()
	out := NormalizedFrame{
		Time:      f.Timestamp,
		Sequence:  f.Sequence,
		Rotations: make(map[string]Quat, len(f.Rotations)),
	}
	for srcName, q := range f.Rotations {
		internal, ok := spec.NameMap[srcName]
		if !ok {
			continue // unmapped joint dropped by design
		}
		if internal == "" {
			return NormalizedFrame{}, &ParseError{Sequence: f.Sequence,
				Joint: srcName, Reason: "name map entry has empty target"}
		}
		qlen := q.X*q.X + q.Y*q.Y + q.Z*q.Z + q.W*q.W
		if qlen < 1e-12 {
			return NormalizedFrame{}, &ParseError{Sequence: f.Sequence,
				Joint: srcName, Reason: "degenerate (zero) quaternion"}
		}
		nq := q.Normalized()
		// Change of basis for a rotation: R' = C * R * C^-1.
		out.Rotations[internal] = conv.Mul(nq).Mul(convInv).Normalized()
	}
	if f.HasRoot {
		out.RootPos = conv.Rotate(f.RootPos)
		out.HasRoot = true
	}
	return out, nil
}
