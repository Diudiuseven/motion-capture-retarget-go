package mocap

// Frame is one sampled pose on the canonical skeleton.
// Rotations are local (parent-relative) rotations indexed by joint name.
type Frame struct {
	Time      float64         // seconds from clip start
	RootPos   Vec3            // root joint world position
	Rotations map[string]Quat // local rotation per joint name
}

// Clip is an animation clip bound to a skeleton.
type Clip struct {
	Skeleton   *Skeleton
	Frames     []Frame
	SampleRate float64 // nominal samples per second
}

// Copy returns a deep copy of the frame.
func (f Frame) Copy() Frame {
	out := Frame{Time: f.Time, RootPos: f.RootPos}
	if f.Rotations != nil {
		out.Rotations = make(map[string]Quat, len(f.Rotations))
		for k, v := range f.Rotations {
			out.Rotations[k] = v
		}
	}
	return out
}

// Copy returns a deep copy of the whole clip (skeleton is shared).
func (c *Clip) Copy() *Clip {
	out := &Clip{Skeleton: c.Skeleton, SampleRate: c.SampleRate}
	out.Frames = make([]Frame, len(c.Frames))
	for i, f := range c.Frames {
		out.Frames[i] = f.Copy()
	}
	return out
}

// FK computes world-space rotations and positions for every joint of
// the skeleton under the given frame pose. Parents are evaluated
// before children (topological order).
func FK(skel *Skeleton, f Frame) (worldRot []Quat, worldPos []Vec3) {
	n := len(skel.Joints)
	worldRot = make([]Quat, n)
	worldPos = make([]Vec3, n)
	for _, i := range skel.Order() {
		j := skel.Joints[i]
		local := f.Rotations[j.Name]
		if _, ok := f.Rotations[j.Name]; !ok {
			local = j.RestRot
		}
		p := skel.Parent(i)
		if p < 0 {
			worldRot[i] = local.Norm()
			worldPos[i] = f.RootPos
		} else {
			worldRot[i] = worldRot[p].Mul(local).Norm()
			worldPos[i] = worldPos[p].Add(worldRot[p].Rotate(j.RestPos))
		}
	}
	return worldRot, worldPos
}
