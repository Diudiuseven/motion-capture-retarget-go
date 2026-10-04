package mocap

import "fmt"

// Joint describes one bone in a skeleton hierarchy.
type Joint struct {
	Name   string
	Parent string // empty for the root
	// RestLocalRotation and RestLocalPosition define the rest (bind) pose
	// of this joint relative to its parent.
	RestLocalRotation Quat
	RestLocalPosition Vec3
}

// Skeleton is a validated joint hierarchy.
type Skeleton struct {
	Joints   []Joint // topologically ordered, parents before children
	index    map[string]int
	parentIx []int // -1 for root
}

// ValidationError describes a structural problem in a skeleton definition.
type ValidationError struct {
	Joint  string // offending joint, if applicable
	Reason string
}

func (e *ValidationError) Error() string {
	if e.Joint == "" {
		return fmt.Sprintf("skeleton validation: %s", e.Reason)
	}
	return fmt.Sprintf("skeleton validation: joint %q: %s", e.Joint, e.Reason)
}

// NewSkeleton validates the joint list and builds a Skeleton.
// It rejects duplicate names, missing parents, and cycles.
func NewSkeleton(joints []Joint) (*Skeleton, error) {
	if len(joints) == 0 {
		return nil, &ValidationError{Reason: "empty joint list"}
	}
	byName := make(map[string]int, len(joints))
	for i, j := range joints {
		if j.Name == "" {
			return nil, &ValidationError{Reason: fmt.Sprintf("joint at index %d has empty name", i)}
		}
		if _, dup := byName[j.Name]; dup {
			return nil, &ValidationError{Joint: j.Name, Reason: "duplicate joint name"}
		}
		byName[j.Name] = i
	}
	roots := 0
	for _, j := range joints {
		if j.Parent == "" {
			roots++
			continue
		}
		if j.Parent == j.Name {
			return nil, &ValidationError{Joint: j.Name, Reason: "joint is its own parent"}
		}
		if _, ok := byName[j.Parent]; !ok {
			return nil, &ValidationError{Joint: j.Name,
				Reason: fmt.Sprintf("parent %q not found", j.Parent)}
		}
	}
	if roots == 0 {
		return nil, &ValidationError{Reason: "no root joint (cycle or all joints have parents)"}
	}
	if roots > 1 {
		return nil, &ValidationError{Reason: fmt.Sprintf("%d root joints found, exactly one required", roots)}
	}

	// Topological order via iterative DFS; detects cycles.
	const (
		white = 0 // unvisited
		gray  = 1 // on stack
		black = 2 // done
	)
	color := make([]int, len(joints))
	parentOf := make(map[string]string, len(joints))
	for _, j := range joints {
		parentOf[j.Name] = j.Parent
	}
	order := make([]Joint, 0, len(joints))
	for _, j := range joints {
		if color[byName[j.Name]] != white {
			continue
		}
		// Walk up the parent chain marking the path.
		var chain []string
		cur := j.Name
		for cur != "" {
			ci := byName[cur]
			if color[ci] == gray {
				return nil, &ValidationError{Joint: cur,
					Reason: "cycle detected in hierarchy"}
			}
			if color[ci] == black {
				break
			}
			color[ci] = gray
			chain = append(chain, cur)
			cur = parentOf[cur]
		}
		// Emit chain in reverse (root-most first).
		for k := len(chain) - 1; k >= 0; k-- {
			name := chain[k]
			color[byName[name]] = black
			order = append(order, joints[byName[name]])
		}
	}
	if len(order) != len(joints) {
		return nil, &ValidationError{Reason: "unreachable joints detected"}
	}

	s := &Skeleton{
		Joints:   order,
		index:    make(map[string]int, len(order)),
		parentIx: make([]int, len(order)),
	}
	for i, j := range order {
		s.index[j.Name] = i
	}
	for i, j := range order {
		if j.Parent == "" {
			s.parentIx[i] = -1
		} else {
			s.parentIx[i] = s.index[j.Parent]
		}
	}
	return s, nil
}

// Index returns the topological index of a joint, or -1.
func (s *Skeleton) Index(name string) int {
	if i, ok := s.index[name]; ok {
		return i
	}
	return -1
}

// ParentIndex returns the parent index of joint i, or -1 for the root.
func (s *Skeleton) ParentIndex(i int) int { return s.parentIx[i] }

// NumJoints returns the joint count.
func (s *Skeleton) NumJoints() int { return len(s.Joints) }

// BoneLength returns the rest-pose offset length of joint i.
func (s *Skeleton) BoneLength(i int) float64 {
	return s.Joints[i].RestLocalPosition.Len()
}

// Pose holds local transforms for every joint of a skeleton.
type Pose struct {
	Rotations []Quat // local rotations, indexed like Skeleton.Joints
	RootPos   Vec3   // root translation (hips), world space
}

// NewRestPose returns the skeleton's rest pose.
func (s *Skeleton) NewRestPose() Pose {
	p := Pose{Rotations: make([]Quat, len(s.Joints))}
	for i, j := range s.Joints {
		p.Rotations[i] = j.RestLocalRotation
	}
	p.RootPos = s.Joints[0].RestLocalPosition
	return p
}

// GlobalRotations computes world-space rotations for each joint.
func (s *Skeleton) GlobalRotations(p Pose) []Quat {
	g := make([]Quat, len(s.Joints))
	for i := range s.Joints {
		if s.parentIx[i] < 0 {
			g[i] = p.Rotations[i].Normalized()
		} else {
			g[i] = g[s.parentIx[i]].Mul(p.Rotations[i]).Normalized()
		}
	}
	return g
}

// JointWorldPositions computes world positions of every joint.
func (s *Skeleton) JointWorldPositions(p Pose) []Vec3 {
	g := s.GlobalRotations(p)
	pos := make([]Vec3, len(s.Joints))
	for i, j := range s.Joints {
		pi := s.parentIx[i]
		if pi < 0 {
			pos[i] = p.RootPos
		} else {
			pos[i] = pos[pi].Add(g[pi].Rotate(j.RestLocalPosition))
		}
	}
	return pos
}
