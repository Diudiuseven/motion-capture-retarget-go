package mocap

import (
	"math"
	"testing"
)

func almostEq(a, b, eps float64) bool { return math.Abs(a-b) < eps }

func TestQuatRotateRoundTrip(t *testing.T) {
	q := QuatFromAxisAngle(Vec3{Y: 1}, math.Pi/2)
	v := q.Rotate(Vec3{Z: 1})
	// Rotating +Z by 90 deg about Y yields +X.
	if !almostEq(v.X, 1, 1e-9) || !almostEq(v.Y, 0, 1e-9) || !almostEq(v.Z, 0, 1e-9) {
		t.Fatalf("unexpected rotation result: %+v", v)
	}
	back := q.Conjugate().Rotate(v)
	if !almostEq(back.Z, 1, 1e-9) {
		t.Fatalf("inverse rotation failed: %+v", back)
	}
}

func TestSlerpMidpoint(t *testing.T) {
	a := IdentityQuat()
	b := QuatFromAxisAngle(Vec3{X: 1}, math.Pi/2)
	m := a.Slerp(b, 0.5)
	if got := m.Angle(); !almostEq(got, math.Pi/4, 1e-9) {
		t.Fatalf("slerp midpoint angle = %v, want pi/4", got)
	}
}

func TestQuatBetween(t *testing.T) {
	a := Vec3{X: 1}.Normalized()
	b := Vec3{Y: 1}.Normalized()
	q := QuatBetween(a, b)
	got := q.Rotate(a)
	if got.Sub(b).Len() > 1e-9 {
		t.Fatalf("QuatBetween rotated to %+v, want %+v", got, b)
	}
	// Opposite vectors must not produce NaN.
	q2 := QuatBetween(a, a.Scale(-1))
	if got := q2.Rotate(a); got.Sub(a.Scale(-1)).Len() > 1e-9 {
		t.Fatalf("QuatBetween opposite failed: %+v", got)
	}
}
