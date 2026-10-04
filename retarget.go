package mocap

import "fmt"

// Retargeter maps poses from a source skeleton to a target skeleton
// with possibly different proportions, rest poses and orientations.
//
// Method: for every mapped joint the world-space rotation delta from
// the source rest pose is transferred onto the target rest pose, so
// motion semantics are preserved regardless of differing bind poses or
// bone lengths. Root translation is scaled by the hip-height ratio.
type Retargeter struct {
	Source *Skeleton
	Target *Skeleton
	// JointMap maps target joint names to source joint names.
	JointMap map[string]string
	// RootMotionScale scales root translation; if 0 it is derived
	// automatically from the root bone-length ratio.
	RootMotionScale float64

	// precomputed per-target-joint data
	srcIx        []int  // source joint index per target joint, -1 if unmapped
	srcRestInv   []Quat // inverse source rest global rotation
	tgtRestGlob  []Quat // target rest global rotation
	tgtRestLocal []Quat // target rest local rotation
	scale        float64
}

// NewRetargeter validates that every mapped joint exists in both
// skeletons and precomputes rest-pose data.
func NewRetargeter(src, dst *Skeleton, jointMap map[string]string) (*Retargeter, error) {
	if src == nil || dst == nil {
		return nil, fmt.Errorf("retargeter: source and target skeletons are required")
	}
	if len(jointMap) == 0 {
		return nil, fmt.Errorf("retargeter: joint map is empty")
	}
	r := &Retargeter{
		Source:       src,
		Target:       dst,
		JointMap:     jointMap,
		srcIx:        make([]int, dst.NumJoints()),
		srcRestInv:   make([]Quat, dst.NumJoints()),
		tgtRestGlob:  dst.GlobalRotations(dst.NewRestPose()),
		tgtRestLocal: make([]Quat, dst.NumJoints()),
	}
	srcRestGlob := src.GlobalRotations(src.NewRestPose())
	for i := 0; i < dst.NumJoints(); i++ {
		r.tgtRestLocal[i] = dst.Joints[i].RestLocalRotation
		r.srcIx[i] = -1
		r.srcRestInv[i] = IdentityQuat()
	}
	for tgtName, srcName := range jointMap {
		ti := dst.Index(tgtName)
		if ti < 0 {
			return nil, fmt.Errorf("retargeter: target joint %q not in target skeleton", tgtName)
		}
		si := src.Index(srcName)
		if si < 0 {
			return nil, fmt.Errorf("retargeter: source joint %q not in source skeleton", srcName)
		}
		r.srcIx[ti] = si
		r.srcRestInv[ti] = srcRestGlob[si].Conjugate()
	}
	// Root must be mapped for root motion transfer.
	if r.srcIx[0] < 0 {
		return nil, fmt.Errorf("retargeter: root joint %q is not mapped", dst.Joints[0].Name)
	}
	// Derive root motion scale from rest root heights if not overridden.
	r.scale = r.RootMotionScale
	if r.scale == 0 {
		sh := src.Joints[0].RestLocalPosition.Len()
		th := dst.Joints[0].RestLocalPosition.Len()
		if sh > 1e-9 {
			r.scale = th / sh
		} else {
			r.scale = 1
		}
	}
	return r, nil
}

// RetargetPose converts one source pose to a target pose.
func (r *Retargeter) RetargetPose(p Pose) Pose {
	srcGlob := r.Source.GlobalRotations(p)
	n := r.Target.NumJoints()
	out := Pose{Rotations: make([]Quat, n)}
	tgtGlob := make([]Quat, n)
	for i := 0; i < n; i++ {
		si := r.srcIx[i]
		if si < 0 {
			// Unmapped target joint: keep its rest local rotation.
			out.Rotations[i] = r.tgtRestLocal[i]
		} else {
			// World delta from source rest, applied to target rest.
			delta := srcGlob[si].Mul(r.srcRestInv[si])
			tgtGlob[i] = delta.Mul(r.tgtRestGlob[i]).Normalized()
			pi := r.Target.ParentIndex(i)
			if pi < 0 {
				out.Rotations[i] = tgtGlob[i]
			} else {
				out.Rotations[i] = tgtGlob[pi].Conjugate().Mul(tgtGlob[i]).Normalized()
			}
			continue
		}
		// Maintain tgtGlob for unmapped joints too.
		pi := r.Target.ParentIndex(i)
		if pi < 0 {
			tgtGlob[i] = out.Rotations[i].Normalized()
		} else {
			tgtGlob[i] = tgtGlob[pi].Mul(out.Rotations[i]).Normalized()
		}
	}
	out.RootPos = p.RootPos.Scale(r.scale)
	return out
}

// RetargetClip converts a sequence of source poses into a Clip on the
// target skeleton.
func (r *Retargeter) RetargetClip(poses []Pose, frameRate float64) *Clip {
	clip := &Clip{
		Skeleton:  r.Target,
		FrameRate: frameRate,
		Frames:    make([]Pose, len(poses)),
	}
	for i, p := range poses {
		clip.Frames[i] = r.RetargetPose(p)
	}
	if frameRate > 0 {
		clip.Duration = float64(len(poses)) / frameRate
	}
	return clip
}
