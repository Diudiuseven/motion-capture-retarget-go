package mocap

import "math"

// Vec3 is a 3D vector in the unified internal coordinate space.
type Vec3 struct {
	X, Y, Z float64
}

func (a Vec3) Add(b Vec3) Vec3 { return Vec3{a.X + b.X, a.Y + b.Y, a.Z + b.Z} }
func (a Vec3) Sub(b Vec3) Vec3 { return Vec3{a.X - b.X, a.Y - b.Y, a.Z - b.Z} }
func (a Vec3) Scale(s float64) Vec3 {
	return Vec3{a.X * s, a.Y * s, a.Z * s}
}
func (a Vec3) Len() float64 { return math.Sqrt(a.X*a.X + a.Y*a.Y + a.Z*a.Z) }

// Norm returns the unit vector; zero vector stays zero.
func (a Vec3) Norm() Vec3 {
	l := a.Len()
	if l < 1e-12 {
		return Vec3{}
	}
	return a.Scale(1 / l)
}

// Lerp linearly interpolates between a and b.
func (a Vec3) Lerp(b Vec3, t float64) Vec3 {
	return a.Add(b.Sub(a).Scale(t))
}

// Quat is a unit quaternion (W is the scalar part).
type Quat struct {
	W, X, Y, Z float64
}

// QuatIdentity is the no-rotation quaternion.
func QuatIdentity() Quat { return Quat{W: 1} }

// Mul composes two rotations: result applies b first, then a.
func (a Quat) Mul(b Quat) Quat {
	return Quat{
		W: a.W*b.W - a.X*b.X - a.Y*b.Y - a.Z*b.Z,
		X: a.W*b.X + a.X*b.W + a.Y*b.Z - a.Z*b.Y,
		Y: a.W*b.Y - a.X*b.Z + a.Y*b.W + a.Z*b.X,
		Z: a.W*b.Z + a.X*b.Y - a.Y*b.X + a.Z*b.W,
	}
}

// Conj returns the conjugate (inverse for unit quaternions).
func (a Quat) Conj() Quat { return Quat{W: a.W, X: -a.X, Y: -a.Y, Z: -a.Z} }

// Norm returns the normalized quaternion; near-zero input yields identity.
func (a Quat) Norm() Quat {
	l := math.Sqrt(a.W*a.W + a.X*a.X + a.Y*a.Y + a.Z*a.Z)
	if l < 1e-12 {
		return QuatIdentity()
	}
	return Quat{a.W / l, a.X / l, a.Y / l, a.Z / l}
}

// Rotate applies the rotation to vector v.
func (a Quat) Rotate(v Vec3) Vec3 {
	qv := Vec3{a.X, a.Y, a.Z}
	uv := cross(qv, v)
	uuv := cross(qv, uv)
	return v.Add(uv.Scale(2 * a.W)).Add(uuv.Scale(2))
}

func cross(a, b Vec3) Vec3 {
	return Vec3{a.Y*b.Z - a.Z*b.Y, a.Z*b.X - a.X*b.Z, a.X*b.Y - a.Y*b.X}
}

// Slerp interpolates between unit quaternions a and b (shortest path).
func (a Quat) Slerp(b Quat, t float64) Quat {
	d := a.W*b.W + a.X*b.X + a.Y*b.Y + a.Z*b.Z
	if d < 0 { // take the short way
		b = Quat{-b.W, -b.X, -b.Y, -b.Z}
		d = -d
	}
	if d > 0.9995 { // nearly equal: nlerp
		return Quat{
			W: a.W + t*(b.W-a.W),
			X: a.X + t*(b.X-a.X),
			Y: a.Y + t*(b.Y-a.Y),
			Z: a.Z + t*(b.Z-a.Z),
		}.Norm()
	}
	theta := math.Acos(d)
	s := math.Sin(theta)
	wa := math.Sin((1-t)*theta) / s
	wb := math.Sin(t*theta) / s
	return Quat{
		W: wa*a.W + wb*b.W,
		X: wa*a.X + wb*b.X,
		Y: wa*a.Y + wb*b.Y,
		Z: wa*a.Z + wb*b.Z,
	}
}

// AngleTo returns the absolute rotation angle (radians) between a and b.
func (a Quat) AngleTo(b Quat) float64 {
	d := math.Abs(a.W*b.W + a.X*b.X + a.Y*b.Y + a.Z*b.Z)
	if d > 1 {
		d = 1
	}
	return 2 * math.Acos(d)
}

// QuatFromAxisAngle builds a quaternion from an axis and angle.
func QuatFromAxisAngle(axis Vec3, angle float64) Quat {
	a := axis.Norm()
	s := math.Sin(angle / 2)
	return Quat{W: math.Cos(angle / 2), X: a.X * s, Y: a.Y * s, Z: a.Z * s}
}

// QuatBetween returns the rotation taking unit vector a to unit vector b.
func QuatBetween(a, b Vec3) Quat {
	d := a.X*b.X + a.Y*b.Y + a.Z*b.Z
	if d > 1-1e-9 {
		return QuatIdentity()
	}
	if d < -1+1e-9 { // opposite: rotate 180° around any perpendicular axis
		axis := cross(Vec3{X: 1}, a)
		if axis.Len() < 1e-9 {
			axis = cross(Vec3{Y: 1}, a)
		}
		return QuatFromAxisAngle(axis, math.Pi)
	}
	c := cross(a, b)
	return Quat{W: 1 + d, X: c.X, Y: c.Y, Z: c.Z}.Norm()
}

// Transform is a local joint transform relative to its parent.
type Transform struct {
	Pos Vec3
	Rot Quat
}

// Mat3 is a row-major 3x3 matrix.
type Mat3 [3][3]float64

// MulVec applies the matrix to a vector.
func (m Mat3) MulVec(v Vec3) Vec3 {
	return Vec3{
		X: m[0][0]*v.X + m[0][1]*v.Y + m[0][2]*v.Z,
		Y: m[1][0]*v.X + m[1][1]*v.Y + m[1][2]*v.Z,
		Z: m[2][0]*v.X + m[2][1]*v.Y + m[2][2]*v.Z,
	}
}

// Mul returns m*n.
func (m Mat3) Mul(n Mat3) Mat3 {
	var out Mat3
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			out[i][j] = m[i][0]*n[0][j] + m[i][1]*n[1][j] + m[i][2]*n[2][j]
		}
	}
	return out
}

// Det returns the determinant.
func (m Mat3) Det() float64 {
	return m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1]) -
		m[0][1]*(m[1][0]*m[2][2]-m[1][2]*m[2][0]) +
		m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0])
}

// Transpose returns the transposed matrix.
func (m Mat3) Transpose() Mat3 {
	var out Mat3
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			out[i][j] = m[j][i]
		}
	}
	return out
}

// QuatFromMat3 converts a pure rotation matrix to a quaternion.
func QuatFromMat3(m Mat3) Quat {
	tr := m[0][0] + m[1][1] + m[2][2]
	var q Quat
	switch {
	case tr > 0:
		s := math.Sqrt(tr+1) * 2
		q = Quat{W: s / 4, X: (m[2][1] - m[1][2]) / s, Y: (m[0][2] - m[2][0]) / s, Z: (m[1][0] - m[0][1]) / s}
	case m[0][0] > m[1][1] && m[0][0] > m[2][2]:
		s := math.Sqrt(1+m[0][0]-m[1][1]-m[2][2]) * 2
		q = Quat{W: (m[2][1] - m[1][2]) / s, X: s / 4, Y: (m[0][1] + m[1][0]) / s, Z: (m[0][2] + m[2][0]) / s}
	case m[1][1] > m[2][2]:
		s := math.Sqrt(1+m[1][1]-m[0][0]-m[2][2]) * 2
		q = Quat{W: (m[0][2] - m[2][0]) / s, X: (m[0][1] + m[1][0]) / s, Y: s / 4, Z: (m[1][2] + m[2][1]) / s}
	default:
		s := math.Sqrt(1+m[2][2]-m[0][0]-m[1][1]) * 2
		q = Quat{W: (m[1][0] - m[0][1]) / s, X: (m[0][2] + m[2][0]) / s, Y: (m[1][2] + m[2][1]) / s, Z: s / 4}
	}
	return q.Norm()
}
