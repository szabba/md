// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

package main

import (
	"github.com/szabba/md/newton"
	"github.com/szabba/md/vect"
)

// A 'picky' force, that doesn't affect some bodies
type PickyForce struct {
	force   newton.Force
	zeroFor []int
}

// Creates a picky version of a force
func NewPicky(f newton.Force, zeroFor ...int) newton.Force {

	return &PickyForce{force: f, zeroFor: zeroFor}
}

func (picky *PickyForce) Accel(bs []*newton.Body, i int) (a vect.Vector) {

	for _, ignored := range picky.zeroFor {

		if ignored == i {

			return vect.Zero
		}
	}

	return picky.force.Accel(bs, i)
}

// Creates a system containing a rectangular grid of m times n particles.
func SetUpRect(m, n int) *newton.System {

	sys := newton.NewSystem(newton.Verlet, m*n)

	for i := 0; i < sys.Bodies(); i++ {

		sys.Body(i).SetMass(1)
	}

	return sys
}

func main() {
}
