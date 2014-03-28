// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

package newton

import (
	"github.com/szabba/md/src/vect"
)

type Body struct {
	Xs, Vs []vect.Vector
	M      float64
	currAt int
}

// Constructs a body of specified mass suitable for working with the integrator
func NewBody(algo Integrator, mass float64) *Body {

	b := new(Body)

	b.Xs = make([]vect.Vector, algo.StateLen())
	b.Vs = make([]vect.Vector, algo.StateLen())

	b.M = mass
	b.currAt = algo.CurrentAt()

	return b
}

// Shifts all the values in xs by one and puts x at the beginning.
func Shift(xs []vect.Vector, x vect.Vector) {

	for i := len(xs) - 1; i > 0; i-- {

		xs[i] = xs[i-1]
	}

	xs[0] = x
}
