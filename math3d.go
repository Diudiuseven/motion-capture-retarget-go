package mocap

import "math"

// Vec3 is a 3D vector in the internal coordinate convention
// (right-handed, Y-up, Z-forward).
type Vec3 struct {
	X, Y, Z float64
}

// Quat is a unit quaternion (x, y, z, w) representing a rotation.
type Quat struct {
	X, Y, Z, W float64
}

// Add returns a+b.
func (a Vec3) Add(b Vec3) Vec3 { return Vec3{a.X + b.X, a.Y + b.Y, a.Z + b.Z} }

// Sub returns a-b.
func (a Vec3) Sub(b Vec3) Vec3 { return Vec3{a.X - b.X, a.Y - b.Y, a.Z - b.Z} }

// Scale returns a*s.
func (a Vec3) Scale(s float64) Vec3 { return Vec3{a.X * s, a.Y * s, a.Z * s} }

// Dot returns the dot product.
func (a Vec3) Dot(b Vec3) float64 { return a.X*b.X + a.Y*b.Y + a.Z*b.Z }

// Cross returns the cross product.
func (a Vec3) Cross(b Vec3) Vec3 {
	return Vec3{
		a.Y*b.Z - a.Z*b.Y,
		a.Z*b.X - a.X*b.Z,
		a.X*b.Y - a.Y*b.X,
	}
}

// Len returns the Euclidean length.
func (a Vec3) Len() float64 { return math.Sqrt(a.Dot(a)) }

// Normalized returns the unit vector; zero vector stays zero.
func (a Vec3) Normalized() Vec3 {
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

// IdentityQuat returns the identity rotation.
func IdentityQuat() Quat { return Quat{W: 1} }

// Mul returns q*r (apply r first, then q).
func (q Quat) Mul(r Quat) Quat {
	return Quat{
		X: q.W*r.X + q.X*r.W + q.Y*r.Z - q.Z*r.Y,
		Y: q.W*r.Y - q.X*r.Z + q.Y*r.W + q.Z*r.X,
		Z: q.W*r.Z + q.X*r.Y - q.Y*r.X + q.Z*r.W,
		W: q.W*r.W - q.X*r.X - q.Y*r.Y - q.Z*r.Z,
	}
}

// Conjugate returns the conjugate (inverse for unit quaternions).
func (q Quat) Conjugate() Quat { return Quat{-q.X, -q.Y, -q.Z, q.W} }

// Normalized returns the unit quaternion; falls back to identity.
func (q Quat) Normalized() Quat {
	l := math.Sqrt(q.X*q.X + q.Y*q.Y + q.Z*q.Z + q.W*q.W)
	if l < 1e-12 {
		return IdentityQuat()
	}
	return Quat{q.X / l, q.Y / l, q.Z / l, q.W / l}
}

// Rotate applies the quaternion to vector v.
func (q Quat) Rotate(v Vec3) Vec3 {
	u := Vec3{q.X, q.Y, q.Z}
	uv := u.Cross(v)
	uuv := u.Cross(uv)
	return v.Add(uv.Scale(2 * q.W)).Add(uuv.Scale(2))
}

// Slerp interpolates between q and r along the shortest arc.
func (q Quat) Slerp(r Quat, t float64) Quat {
	dot := q.X*r.X + q.Y*r.Y + q.Z*r.Z + q.W*r.W
	if dot < 0 {
		r = Quat{-r.X, -r.Y, -r.Z, -r.W}
		dot = -dot
	}
	if dot > 0.9995 {
		return Quat{
			q.X + t*(r.X-q.X),
			q.Y + t*(r.Y-q.Y),
			q.Z + t*(r.Z-q.Z),
			q.W + t*(r.W-q.W),
		}.Normalized()
	}
	theta := math.Acos(math.Min(1, math.Max(-1, dot)))
	sinT := math.Sin(theta)
	a := math.Sin((1-t)*theta) / sinT
	b := math.Sin(t*theta) / sinT
	return Quat{
		a*q.X + b*r.X,
		a*q.Y + b*r.Y,
		a*q.Z + b*r.Z,
		a*q.W + b*r.W,
	}
}

// QuatFromAxisAngle builds a quaternion from an axis and angle in radians.
func QuatFromAxisAngle(axis Vec3, angle float64) Quat {
	n := axis.Normalized()
	h := angle / 2
	s := math.Sin(h)
	return Quat{n.X * s, n.Y * s, n.Z * s, math.Cos(h)}
}

// QuatBetween returns the rotation taking unit vector a to unit vector b.
func QuatBetween(a, b Vec3) Quat {
	d := a.Dot(b)
	if d > 1-1e-9 {
		return IdentityQuat()
	}
	if d < -1+1e-9 {
		// Opposite vectors: pick any orthogonal axis.
		axis := Vec3{X: 1}.Cross(a)
		if axis.Len() < 1e-9 {
			axis = Vec3{Y: 1}.Cross(a)
		}
		return QuatFromAxisAngle(axis.Normalized(), math.Pi)
	}
	axis := a.Cross(b)
	return Quat{axis.X, axis.Y, axis.Z, 1 + d}.Normalized()
}

// QuatFromMatrix builds a quaternion from a 3x3 row-major rotation matrix.
func QuatFromMatrix(m [9]float64) Quat {
	tr := m[0] + m[4] + m[8]
	switch {
	case tr > 0:
		s := math.Sqrt(tr+1) * 2
		return Quat{(m[7] - m[5]) / s, (m[2] - m[6]) / s, (m[3] - m[1]) / s, s / 4}.Normalized()
	case m[0] > m[4] && m[0] > m[8]:
		s := math.Sqrt(1+m[0]-m[4]-m[8]) * 2
		return Quat{s / 4, (m[1] + m[3]) / s, (m[2] + m[6]) / s, (m[7] - m[5]) / s}.Normalized()
	case m[4] > m[8]:
		s := math.Sqrt(1+m[4]-m[0]-m[8]) * 2
		return Quat{(m[1] + m[3]) / s, s / 4, (m[5] + m[7]) / s, (m[2] - m[6]) / s}.Normalized()
	default:
		s := math.Sqrt(1+m[8]-m[0]-m[4]) * 2
		return Quat{(m[2] + m[6]) / s, (m[5] + m[7]) / s, s / 4, (m[3] - m[1]) / s}.Normalized()
	}
}

// Angle returns the rotation angle in radians in [0, pi].
func (q Quat) Angle() float64 {
	q = q.Normalized()
	w := math.Min(1, math.Max(-1, math.Abs(q.W)))
	return 2 * math.Acos(w)
}
