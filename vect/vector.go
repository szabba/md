// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

// Package vect implements a 3D vector type useful in physics and computational
// geometry calculations.
package vect

import (
	"math"
)

// A three component cartesian vector
type Vector []float64

// Create new zero vector
func NewZeroVector() Vector {

	return Vector(make([]float64, 3))
}

// Create new vector
func NewVector(x, y, z float64) Vector {

	v := NewZeroVector()

	v[0], v[1], v[2] = x, y, z

	return v
}

var (
	// A zero vector
	Zero = NewZeroVector()
	// An unit vector along the Cartesian X axis
	UnitX = NewVector(1, 0, 0)
	// An unit vector along the Cartesian Y axis
	UnitY = NewVector(0, 1, 0)
	// An unit vector along the Cartesian Z axis
	UnitZ = NewVector(0, 0, 1)
)

// Copy the vector
func (v Vector) Copy() (u Vector) {

	u = NewZeroVector()

	for i, vComp := range v {

		u[i] = vComp
	}

	return u
}

// Add two vectors
func (a Vector) Plus(b Vector) (c Vector) {

	c = NewZeroVector()

	for i, _ := range a {

		c[i] = a[i] + b[i]
	}

	return c
}

// Invert a vector's direction
func (a Vector) Negate() (minusA Vector) {

	minusA = NewZeroVector()

	for i, aComponent := range a {

		minusA[i] = -aComponent
	}

	return minusA
}

// Subtract two vectors
func (a Vector) Minus(b Vector) Vector {

	return a.Plus(b.Negate())
}

// Take the dot product of two vectors
func (a Vector) Dot(b Vector) float64 {

	sum := 0.0

	for i, _ := range a {

		sum += a[i] * b[i]
	}

	return sum
}

// Calculate the norm of a vector
func (a Vector) Norm() float64 {

	return math.Sqrt(a.Dot(a))
}

// Scale a vector by s
func (a Vector) Scale(s float64) (b Vector) {

	b = a.Copy()

	for i, _ := range a {

		b[i] *= s
	}

	return b
}

// Produce the unit vector oriented the same as a
//
// If the vector is zero -- returns itself
func (a Vector) Unit() Vector {

	if a.Equal(Zero) {

		return a
	}

	return a.Scale(1 / a.Norm())
}

// Calculate a unit vector and the vector norm at once
func (a Vector) UnitAndNorm() (aU Vector, norm float64) {

	return a.Unit(), a.Norm()
}

// Take the cross product of two vectors
func (a Vector) Cross(b Vector) (c Vector) {

	c = NewZeroVector()

	for i, _ := range a {

		c[i] += a[(i+1)%3] * b[(i+2)%3]

		c[i] -= a[(i+2)%3] * b[(i+1)%3]
	}

	return c
}

// Compare two vectors for equality
func (a Vector) Equal(b Vector) bool {

	for i, _ := range a {

		if a[i] != b[i] {

			return false
		}
	}

	return true
}

// Decompose a vector into the part parallel and orthogonal to another one.
func (a Vector) ProjectionRejection(wRespectTo Vector) (prj, rej Vector) {

	prj = wRespectTo.Unit().Scale(a.Dot(wRespectTo))

	rej = a.Minus(prj)

	return prj, rej
}

// Component vector parallel to another one.
func (a Vector) Projection(onto Vector) (prj Vector) {

	prj, _ = a.ProjectionRejection(onto)

	return prj
}

// Component vector orthogonal to another one.
func (a Vector) Rejection(ofOf Vector) (rej Vector) {

	_, rej = a.ProjectionRejection(ofOf)

	return rej
}
