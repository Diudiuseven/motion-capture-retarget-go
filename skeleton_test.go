package mocap

import (
	"strings"
	"testing"
)

func testJoints() []Joint {
	return []Joint{
		{Name: "Hips", RestLocalPosition: Vec3{Y: 1}},
		{Name: "Spine", Parent: "Hips", RestLocalPosition: Vec3{Y: 0.3}},
		{Name: "Head", Parent: "Spine", RestLocalPosition: Vec3{Y: 0.4}},
	}
}

func TestNewSkeletonValid(t *testing.T) {
	s, err := NewSkeleton(testJoints())
	if err != nil {
		t.Fatalf("valid skeleton rejected: %v", err)
	}
	if s.NumJoints() != 3 {
		t.Fatalf("NumJoints = %d, want 3", s.NumJoints())
	}
	// Topological order: parents before children.
	if s.Index("Hips") > s.Index("Spine") || s.Index("Spine") > s.Index("Head") {
		t.Fatalf("joints not topologically ordered")
	}
}

func TestNewSkeletonRejectsDuplicates(t *testing.T) {
	j := testJoints()
	j = append(j, Joint{Name: "Spine", Parent: "Hips"})
	_, err := NewSkeleton(j)
	if err == nil || !strings.Contains(err.Error(), "duplicate") {
		t.Fatalf("expected duplicate error, got %v", err)
	}
}

func TestNewSkeletonRejectsMissingParent(t *testing.T) {
	j := testJoints()
	j = append(j, Joint{Name: "Arm", Parent: "Ghost"})
	_, err := NewSkeleton(j)
	var ve *ValidationError
	if err == nil {
		t.Fatal("expected missing parent error")
	}
	if e, ok := err.(*ValidationError); ok {
		ve = e
	} else {
		t.Fatalf("error type = %T, want *ValidationError", err)
	}
	if ve.Joint != "Arm" || !strings.Contains(ve.Reason, "Ghost") {
		t.Fatalf("error not locatable: %v", ve)
	}
}

func TestNewSkeletonRejectsCycle(t *testing.T) {
	j := []Joint{
		{Name: "A", Parent: "C"},
		{Name: "B", Parent: "A"},
		{Name: "C", Parent: "B"},
	}
	_, err := NewSkeleton(j)
	if err == nil {
		t.Fatal("expected cycle error")
	}
	if !strings.Contains(err.Error(), "cycle") && !strings.Contains(err.Error(), "no root") {
		t.Fatalf("unexpected error: %v", err)
	}
}

func TestWorldPositions(t *testing.T) {
	s, _ := NewSkeleton(testJoints())
	p := s.NewRestPose()
	pos := s.JointWorldPositions(p)
	head := pos[s.Index("Head")]
	if !almostEq(head.Y, 1.7, 1e-9) {
		t.Fatalf("head world Y = %v, want 1.7", head.Y)
	}
}
