package mocap

// Retargeter maps motion from a source skeleton to a target skeleton
// with possibly different proportions, rest poses and orientation
// conventions.
type Retargeter struct {
	Src     *Skeleton
	Dst     *Skeleton
	BoneMap map[string]string // source joint name -> target joint name

	srcRestWorld []Quat // rest-pose world rotations on the source
	dstRestWorld []Quat // rest-pose world rotations on the target
	dstFromSrc   []int  // dst joint index -> src joint index, -1 if unmapped
	posScale     float64
}

// NewRetargeter builds a retargeter and validates the bone map.
func NewRetargeter(src, dst *Skeleton, boneMap map[string]string) (*Retargeter, error) {
	if err := src.Validate(); err != nil {
		return nil, err
	}
	if err := dst.Validate(); err != nil {
		return nil, err
	}
	r := &Retargeter{Src: src, Dst: dst, BoneMap: boneMap}
	r.dstFromSrc = make([]int, len(dst.Joints))
	for i := range r.dstFromSrc {
		r.dstFromSrc[i] = -1
	}
	for sName, dName := range boneMap {
		si := src.Index(sName)
		if si < 0 {
			return nil, errf("retarget", -1, sName, "bone map source joint not in source skeleton")
		}
		di := dst.Index(dName)
		if di < 0 {
			return nil, errf("retarget", -1, dName, "bone map target joint not in target skeleton")
		}
		r.dstFromSrc[di] = si
	}
	srcRoot, dstRoot := src.RootIndex(), dst.RootIndex()
	if r.dstFromSrc[dstRoot] < 0 {
		return nil, errf("retarget", -1, dst.Joints[dstRoot].Name,
			"bone map must cover the target root joint")
	}

	// Rest-pose world rotations on both skeletons.
	restFrame := Frame{Rotations: map[string]Quat{}}
	r.srcRestWorld, _ = FK(src, restFrame)
	r.dstRestWorld, _ = FK(dst, restFrame)

	// Scale root translation by the skeletons' standing height ratio so
	// locomotion keeps its semantics without proportion distortion.
	sh := src.Joints[srcRoot].RestPos.Len()
	dh := dst.Joints[dstRoot].RestPos.Len()
	r.posScale = 1
	if sh > 1e-9 {
		r.posScale = dh / sh
	}
	return r, nil
}

// Retarget converts a clip on the source skeleton to the target
// skeleton, preserving motion semantics without proportion
// distortion or joint flips.
func (r *Retargeter) Retarget(clip *Clip) (*Clip, error) {
	if clip.Skeleton != r.Src {
		return nil, errf("retarget", -1, "", "clip skeleton %q is not the retarget source %q",
			clip.Skeleton.Name, r.Src.Name)
	}
	out := &Clip{Skeleton: r.Dst, SampleRate: clip.SampleRate}
	out.Frames = make([]Frame, len(clip.Frames))

	for fi, f := range clip.Frames {
		srcWorld, _ := FK(r.Src, f)
		nf := Frame{
			Time:      f.Time,
			RootPos:   f.RootPos.Scale(r.posScale),
			Rotations: make(map[string]Quat, len(r.Dst.Joints)),
		}
		dstWorld := make([]Quat, len(r.Dst.Joints))
		for _, di := range r.Dst.Order() {
			si := r.dstFromSrc[di]
			var world Quat
			if si < 0 {
				// Unmapped joint: keep its rest-pose world orientation.
				world = r.dstRestWorld[di]
			} else {
				// World-space motion delta of the source joint applied
				// on top of the target's rest orientation.
				delta := srcWorld[si].Mul(r.srcRestWorld[si].Conj())
				world = delta.Mul(r.dstRestWorld[di]).Norm()
			}
			dstWorld[di] = world
			p := r.Dst.Parent(di)
			if p < 0 {
				nf.Rotations[r.Dst.Joints[di].Name] = world
			} else {
				nf.Rotations[r.Dst.Joints[di].Name] = dstWorld[p].Conj().Mul(world).Norm()
			}
		}
		out.Frames[fi] = nf
	}
	return out, nil
}
