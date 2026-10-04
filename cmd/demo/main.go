// Command demo runs the full mocap pipeline end to end on synthetic
// local input and prints a deterministic summary. No devices, no
// network: the same input always produces the same output.
package main

import (
	"crypto/sha256"
	"encoding/binary"
	"fmt"
	"math"

	"github.com/Diudiuseven/motion-capture-retarget-go/mocap"
)

func main() {
	// Source skeleton (capture rig) and target skeleton (game character).
	src := &mocap.Skeleton{Name: "capture-rig", Joints: []mocap.Joint{
		{Name: "root", RestPos: mocap.Vec3{Y: 1}},
		{Name: "spine", Parent: "root", RestPos: mocap.Vec3{Y: 0.5}},
		{Name: "arm", Parent: "spine", RestPos: mocap.Vec3{X: 0.3}},
	}}
	dst := &mocap.Skeleton{Name: "hero", Joints: []mocap.Joint{
		{Name: "hips", RestPos: mocap.Vec3{Y: 1.6}},
		{Name: "torso", Parent: "hips", RestPos: mocap.Vec3{Y: 0.8}},
		{Name: "hand", Parent: "torso", RestPos: mocap.Vec3{X: 0.5},
			RestRot: mocap.QuatFromAxisAngle(mocap.Vec3{Z: 1}, math.Pi/2)},
	}}

	// Synthetic raw capture: Z-up left-handed source, 30 Hz, with a
	// dropped frame, a duplicate frame and timestamp drift.
	raw := &mocap.RawCapture{
		Source:     "synthetic",
		SampleRate: 30,
		Axes:       mocap.AxisConvention{Up: "Z", Forward: "Y", Right: "X", LeftHand: true},
		NameMap:    map[string]string{"hips": "root", "spine1": "spine", "arm_L": "arm"},
	}
	const n = 90 // 3 seconds, one full swing period
	for i := 0; i < n; i++ {
		if i == 40 {
			continue // dropped frame
		}
		a := 0.6 * math.Sin(2*math.Pi*float64(i)/30)
		f := mocap.RawFrame{
			Timestamp: float64(i)/30 + 0.001*math.Sin(float64(i)), // drift
			Joints: map[string]mocap.JointSample{
				"hips":   {Pos: mocap.Vec3{Z: 1}, Rot: mocap.QuatIdentity()},
				"spine1": {Rot: mocap.QuatIdentity()},
				"arm_L":  {Rot: mocap.QuatFromAxisAngle(mocap.Vec3{Z: 1}, a)},
			},
		}
		raw.Frames = append(raw.Frames, f)
		if i == 10 {
			raw.Frames = append(raw.Frames, f) // duplicate frame
		}
	}

	clip, err := mocap.Normalize(raw, src)
	check("Normalize", err)
	clip, err = mocap.Repair(clip, mocap.RepairConfig{MaxGapFrames: 3, DriftTol: 0.01})
	check("Repair", err)
	clip = mocap.Smooth(clip, 1)

	rt, err := mocap.NewRetargeter(src, dst, map[string]string{
		"root": "hips", "spine": "torso", "arm": "hand",
	})
	check("NewRetargeter", err)
	out, err := rt.Retarget(clip)
	check("Retarget", err)

	loop, err := mocap.AlignLoop(out, mocap.LoopConfig{SearchWindow: 40, MaxError: 0.1})
	check("AlignLoop", err)
	final, err := mocap.Compress(loop, 0.02)
	check("Compress", err)

	fmt.Printf("frames: %d -> %d after compression\n", len(clip.Frames), len(final.Frames))
	fmt.Printf("duration: %.3fs @ %.0f Hz\n",
		final.Frames[len(final.Frames)-1].Time, final.SampleRate)
	fmt.Printf("checksum: %x\n", checksum(final))
}

func check(stage string, err error) {
	if err != nil {
		panic(fmt.Sprintf("%s failed: %v", stage, err))
	}
}

// checksum hashes the whole clip for reproducibility verification.
func checksum(c *mocap.Clip) []byte {
	h := sha256.New()
	put := func(v float64) {
		var b [8]byte
		binary.LittleEndian.PutUint64(b[:], math.Float64bits(v))
		h.Write(b[:])
	}
	for _, f := range c.Frames {
		put(f.Time)
		put(f.RootPos.X)
		put(f.RootPos.Y)
		put(f.RootPos.Z)
		for _, j := range c.Skeleton.Joints {
			q := f.Rotations[j.Name]
			put(q.W)
			put(q.X)
			put(q.Y)
			put(q.Z)
		}
	}
	return h.Sum(nil)[:8]
}
