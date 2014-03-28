// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

package newton

import (
	"github.com/szabba/md/src/vect"
)

type Body struct {
	Xs, Vs []vect.Vector
	mass   float64
	currAt int
}

// Constructs a body of specified mass suitable for working with the integrator
func NewBody(algo Integrator, mass float64) *Body {

	b := new(Body)

	b.Xs = make([]vect.Vector, algo.StateLen())
	b.Vs = make([]vect.Vector, algo.StateLen())

	b.mass = mass
	b.currAt = algo.CurrentAt()

	return b
}

// Give a body's mass
func (b *Body) Mass() float64 {

	return b.mass
}

// Put new values of x and v a the beginning of the remembered values
//
// The oldest values get discarded
func (b *Body) Shift(x, v vect.Vector) {

	Shift(b.Xs, x)
	Shift(b.Vs, v)
}

// Current positon and velocity
func (b *Body) Now() (x, v vect.Vector) {

	return b.XNow(), b.VNow()
}

// Current position
func (b *Body) XNow() vect.Vector {

	return b.Xs[b.currAt]
}

// Current velocity
func (b *Body) VNow() vect.Vector {

	return b.Vs[b.currAt]
}

// Shifts all the values in xs by one and puts x at the beginning.
func Shift(xs []vect.Vector, x vect.Vector) {

	for i := len(xs) - 1; i > 0; i-- {

		xs[i] = xs[i-1]
	}

	xs[0] = x
}
