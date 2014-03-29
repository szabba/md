// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

package vect

import (
	"testing"
)

func UnitVectors() (eX, eY, eZ Vector) {

	eX = NewVector(1, 0, 0)
	eY = NewVector(0, 1, 0)
	eZ = NewVector(0, 0, 1)

	return
}

func TestDotProduct(t *testing.T) {

	eX, eY, _ := UnitVectors()

	if eX.Dot(eY) != 0 {

		t.Fatalf(
			"%v.Dot(%v) should be 0 not %f",
			eX, eY, eX.Dot(eY),
		)
	}

	if eX.Dot(eX) != 1 {

		t.Fatalf(
			"%v.Dot(%v) should be 1 not %f",
			eX, eX, eX.Dot(eX),
		)
	}
}

func aCrossBGivesC(a, b, c Vector, t *testing.T) {

	if a.Cross(b).Dot(c) != 1 {

		t.Fatalf(
			"%v.Cross(%v).Dot(%v) should be 1 not %f",
			a, b, c, a.Cross(b).Dot(c),
		)
	}
}

func TestCrossProduct(t *testing.T) {

	eX, eY, eZ := UnitVectors()

	aCrossBGivesC(eX, eY, eZ, t)
	aCrossBGivesC(eZ, eX, eY, t)
	aCrossBGivesC(eY, eZ, eX, t)

	aCrossBGivesC(eY, eX, eZ.Negate(), t)
	aCrossBGivesC(eX, eZ, eY.Negate(), t)
	aCrossBGivesC(eZ, eY, eX.Negate(), t)
}
