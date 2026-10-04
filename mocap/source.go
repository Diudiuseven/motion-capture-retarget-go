package mocap

import "fmt"

// AxisConvention describes the coordinate convention of a raw capture
// source. The internal space is right-handed Y-up, forward +Z.
type AxisConvention struct {
	Up       string // source axis pointing up: "X","-X","Y","-Y","Z","-Z"
	Forward  string // source axis pointing forward
	Right    string // source axis pointing right
	LeftHand bool   // true for left-handed sources
}

// JointSample is a raw per-joint sample in source coordinates.
type JointSample struct {
	Pos Vec3
	Rot Quat
}

// RawFrame is one raw captured frame before normalization.
type RawFrame struct {
	Timestamp float64
	Joints    map[string]JointSample // keyed by source joint name
}

// RawCapture is unparsed capture data from one source.
type RawCapture struct {
	Source     string            // source identifier, for error messages
	SampleRate float64           // nominal samples per second
	Axes       AxisConvention    // source coordinate convention
	NameMap    map[string]string // source joint name -> canonical joint name
	Frames     []RawFrame
}

// axisVector parses an axis string like "-Z" into a signed basis vector.
func axisVector(s string) (Vec3, error) {
	neg := false
	if len(s) == 2 && s[0] == '-' {
		neg = true
		s = s[1:]
	}
	var v Vec3
	switch s {
	case "X":
		v = Vec3{X: 1}
	case "Y":
		v = Vec3{Y: 1}
	case "Z":
		v = Vec3{Z: 1}
	default:
		return Vec3{}, fmt.Errorf("invalid axis %q", s)
	}
	if neg {
		v = v.Scale(-1)
	}
	return v, nil
}

// basisConversion builds the source->internal change-of-basis matrix
// and the mirror flag for left-handed sources.
func basisConversion(a AxisConvention) (m Mat3, mirror bool, err error) {
	up, err := axisVector(a.Up)
	if err != nil {
		return Mat3{}, false, err
	}
	fwd, err := axisVector(a.Forward)
	if err != nil {
		return Mat3{}, false, err
	}
	right, err := axisVector(a.Right)
	if err != nil {
		return Mat3{}, false, err
	}
	// Column j of m is the internal-space image of source basis axis j.
	// up/fwd/right are expressed in *source* coordinates, so the
	// source->internal matrix rows are the transposed semantic vectors:
	// internal = [right up fwd]^T * source (semantic axes as rows).
	m = Mat3{
		{right.X, right.Y, right.Z},
		{up.X, up.Y, up.Z},
		{fwd.X, fwd.Y, fwd.Z},
	}
	det := m.Det()
	if det > 0.5 {
		if a.LeftHand {
			return Mat3{}, false, fmt.Errorf("convention declared left-handed but axes form a right-handed system")
		}
		return m, false, nil
	}
	if det < -0.5 {
		if !a.LeftHand {
			return Mat3{}, false, fmt.Errorf("convention declared right-handed but axes form a left-handed system")
		}
		return m, true, nil
	}
	return Mat3{}, false, fmt.Errorf("degenerate axis convention (axes not orthogonal)")
}

// axisConverter converts positions and rotations between spaces.
type axisConverter struct {
	m      Mat3 // full source->internal matrix (may include mirror)
	rot    Quat // proper rotation part of m
	mirror bool
}

func newAxisConverter(a AxisConvention) (*axisConverter, error) {
	m, mirror, err := basisConversion(a)
	if err != nil {
		return nil, err
	}
	rm := m
	if mirror {
		// Split off the X-reflection: m = rm * S, S = diag(-1,1,1).
		s := Mat3{{-1, 0, 0}, {0, 1, 0}, {0, 0, 1}}
		rm = m.Mul(s)
	}
	return &axisConverter{m: m, rot: QuatFromMat3(rm), mirror: mirror}, nil
}

func (c *axisConverter) pos(v Vec3) Vec3 { return c.m.MulVec(v) }

func (c *axisConverter) quat(q Quat) Quat {
	if c.mirror {
		// Conjugate by S=diag(-1,1,1): (w,x,y,z) -> (w,x,-y,-z).
		q = Quat{W: q.W, X: q.X, Y: -q.Y, Z: -q.Z}
	}
	return c.rot.Mul(q).Mul(c.rot.Conj()).Norm()
}

// Normalize converts a raw capture into the unified internal
// representation bound to the canonical skeleton: joints are renamed,
// axes are converted to the internal Y-up right-handed space, and the
// result is validated against the skeleton.
func Normalize(raw *RawCapture, skel *Skeleton) (*Clip, error) {
	if raw == nil {
		return nil, errf("normalize", -1, "", "nil raw capture")
	}
	if raw.SampleRate <= 0 {
		return nil, errf("normalize", -1, "", "source %q: sample rate must be > 0", raw.Source)
	}
	if err := skel.Validate(); err != nil {
		return nil, err
	}
	conv, err := newAxisConverter(raw.Axes)
	if err != nil {
		return nil, errf("normalize", -1, "", "source %q: %v", raw.Source, err)
	}
	// Every canonical joint must be reachable through the name map.
	mapped := make(map[string]string, len(raw.NameMap)) // canonical -> source
	for src, canon := range raw.NameMap {
		if skel.Index(canon) < 0 {
			return nil, errf("normalize", -1, canon,
				"source %q: name map targets unknown canonical joint", raw.Source)
		}
		mapped[canon] = src
	}
	for _, j := range skel.Joints {
		if _, ok := mapped[j.Name]; !ok {
			return nil, errf("normalize", -1, j.Name,
				"source %q: no source joint maps to canonical joint", raw.Source)
		}
	}

	clip := &Clip{Skeleton: skel, SampleRate: raw.SampleRate}
	clip.Frames = make([]Frame, len(raw.Frames))
	rootName := skel.Joints[skel.RootIndex()].Name
	for fi, rf := range raw.Frames {
		f := Frame{Time: rf.Timestamp, Rotations: make(map[string]Quat, len(skel.Joints))}
		for canon, src := range mapped {
			smp, ok := rf.Joints[src]
			if !ok {
				continue // missing sample: left absent, repair interpolates
			}
			f.Rotations[canon] = conv.quat(smp.Rot)
			if canon == rootName {
				f.RootPos = conv.pos(smp.Pos)
			}
		}
		clip.Frames[fi] = f
	}
	return clip, nil
}
