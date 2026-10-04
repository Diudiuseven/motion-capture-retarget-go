package mocap

// Joint describes one bone in a skeleton hierarchy.
type Joint struct {
	Name    string // canonical joint name
	Parent  string // parent joint name, "" for the root
	RestPos Vec3   // rest-pose offset from the parent joint
	RestRot Quat   // rest-pose local rotation
}

// Skeleton is a validated joint hierarchy with a rest pose.
type Skeleton struct {
	Name   string
	Joints []Joint

	parent []int          // parent index per joint, -1 for root
	index  map[string]int // name -> joint index
	order  []int          // topological order (parents before children)
}

// Validate checks the hierarchy: duplicate names, missing parents,
// multiple roots and cycles are rejected with locatable errors.
// On success it builds the internal lookup tables.
func (s *Skeleton) Validate() error {
	if len(s.Joints) == 0 {
		return errf("validate", -1, "", "skeleton %q has no joints", s.Name)
	}
	index := make(map[string]int, len(s.Joints))
	for i, j := range s.Joints {
		if j.Name == "" {
			return errf("validate", -1, "", "skeleton %q: joint %d has empty name", s.Name, i)
		}
		if _, dup := index[j.Name]; dup {
			return errf("validate", -1, j.Name, "skeleton %q: duplicate joint name", s.Name)
		}
		index[j.Name] = i
	}

	parent := make([]int, len(s.Joints))
	roots := 0
	for i, j := range s.Joints {
		if j.Parent == "" {
			parent[i] = -1
			roots++
			continue
		}
		p, ok := index[j.Parent]
		if !ok {
			return errf("validate", -1, j.Name,
				"skeleton %q: parent %q not found", s.Name, j.Parent)
		}
		parent[i] = p
	}
	if roots == 0 {
		return errf("validate", -1, "", "skeleton %q: no root joint (cycle?)", s.Name)
	}
	if roots > 1 {
		return errf("validate", -1, "", "skeleton %q: %d root joints, want exactly 1", s.Name, roots)
	}

	// Topological order via Kahn's algorithm; leftover joints mean a cycle.
	indeg := make([]int, len(s.Joints))
	children := make([][]int, len(s.Joints))
	for i, p := range parent {
		if p >= 0 {
			indeg[i]++
			children[p] = append(children[p], i)
		}
	}
	var queue []int
	for i := range indeg {
		if indeg[i] == 0 {
			queue = append(queue, i)
		}
	}
	order := make([]int, 0, len(s.Joints))
	for len(queue) > 0 {
		n := queue[0]
		queue = queue[1:]
		order = append(order, n)
		for _, c := range children[n] {
			indeg[c]--
			if indeg[c] == 0 {
				queue = append(queue, c)
			}
		}
	}
	if len(order) != len(s.Joints) {
		// find a joint still on a cycle for a locatable error
		for i := range indeg {
			if indeg[i] > 0 {
				return errf("validate", -1, s.Joints[i].Name,
					"skeleton %q: hierarchy contains a cycle", s.Name)
			}
		}
	}

	s.parent = parent
	s.index = index
	s.order = order
	return nil
}

// Index returns the joint index for a canonical name, or -1.
func (s *Skeleton) Index(name string) int {
	if s.index == nil {
		return -1
	}
	i, ok := s.index[name]
	if !ok {
		return -1
	}
	return i
}

// Parent returns the parent joint index, or -1 for the root.
func (s *Skeleton) Parent(i int) int {
	if i < 0 || i >= len(s.parent) {
		return -1
	}
	return s.parent[i]
}

// Order returns joint indices in topological order.
func (s *Skeleton) Order() []int { return s.order }

// BoneLen returns the rest-pose bone length of joint i.
func (s *Skeleton) BoneLen(i int) float64 {
	if i < 0 || i >= len(s.Joints) {
		return 0
	}
	return s.Joints[i].RestPos.Len()
}

// RootIndex returns the index of the root joint.
func (s *Skeleton) RootIndex() int {
	if len(s.order) == 0 {
		return -1
	}
	for i, p := range s.parent {
		if p == -1 {
			return i
		}
	}
	return -1
}
