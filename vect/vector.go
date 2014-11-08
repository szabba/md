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
type Vector struct {
	x, y, z float64
}

// Create new zero vector
func NewZeroVector() Vector {

	return Vector{}
}

// Create new vector
func NewVector(x, y, z float64) Vector {

	return Vector{x: x, y: y, z: z}
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
func (v Vector) Copy() Vector {

	return v
}

// Add two vectors
func (a Vector) Plus(b Vector) Vector {

	return Vector{
		x: a.x + b.x,
		y: a.y + b.y,
		z: a.z + b.z,
	}
}

// Invert a vector's direction
func (a Vector) Negate() Vector {

	return Vector{x: -a.x, y: -a.y, z: -a.z}
}

// Subtract two vectors
func (a Vector) Minus(b Vector) Vector {

	return a.Plus(b.Negate())
}

// Take the dot product of two vectors
func (a Vector) Dot(b Vector) float64 {

	return a.x*b.x + a.y*b.y + a.z*b.z
}

// Calculate the norm of a vector
func (a Vector) Norm() float64 {

	return math.Sqrt(a.Dot(a))
}

// Scale a vector by s
func (a Vector) Scale(s float64) (b Vector) {

	return Vector{x: s * a.x, y: s * a.y, z: s * a.z}
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

	return Vector{
		x: a.y*b.z - a.z*b.y,
		y: a.z*b.x - a.x*b.z,
		z: a.x*b.y - a.y*b.x,
	}
}

// Compare two vectors for equality
func (a Vector) Equal(b Vector) bool {

	return a == b
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
