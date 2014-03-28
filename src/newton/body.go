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

// Set current positon and velocity
func (b *Body) SetNow(x, v vect.Vector) {

	b.SetXNow(x)
	b.SetVNow(v)
}

// Set current positon
func (b *Body) SetXNow(x vect.Vector) {

	b.Xs[b.currAt] = x
}

// Set current velocity
func (b *Body) SetVNow(v vect.Vector) {

	b.Vs[b.currAt] = v
}

// Position and velocity delta steps before now
func (b *Body) Before(delta int) (x, v vect.Vector) {

	return b.XBefore(delta), b.VBefore(delta)
}

// Position delta steps before now
func (b *Body) XBefore(delta int) vect.Vector {

	return b.Xs[b.currAt+delta]
}

// Velocity delta steps before now
func (b *Body) VBefore(delta int) vect.Vector {

	return b.Vs[b.currAt+delta]
}

// Set positon and velocity delta steps in the past
func (b *Body) SetBefore(x, v vect.Vector, delta int) {

	b.SetXBefore(x, delta)
	b.SetVBefore(v, delta)
}

// Set positon delta steps in the past
func (b *Body) SetXBefore(x vect.Vector, delta int) {

	b.Xs[b.currAt+delta] = x
}

// Set velocity delta steps in the past
func (b *Body) SetVBefore(v vect.Vector, delta int) {

	b.Vs[b.currAt+delta] = v
}

// Position and velocity delta steps after now
func (b *Body) After(delta int) (x, v vect.Vector) {

	return b.XAfter(delta), b.VAfter(delta)
}

// Position delta steps after now
func (b *Body) XAfter(delta int) vect.Vector {

	return b.Xs[b.currAt+delta]
}

// Velocity delta steps after now
func (b *Body) VAfter(delta int) vect.Vector {

	return b.Vs[b.currAt+delta]
}

// Set positon and velocity delta steps in the future
func (b *Body) SetAfter(x, v vect.Vector, delta int) {

	b.SetXAfter(x, delta)
	b.SetVAfter(v, delta)
}

// Set positon delta steps in the future
func (b *Body) SetXAfter(x vect.Vector, delta int) {

	b.Xs[b.currAt-delta] = x
}

// Set velocity delta steps in the future
func (b *Body) SetVAfter(v vect.Vector, delta int) {

	b.Vs[b.currAt-delta] = v
}

// Shifts all the values in xs by one and puts x at the beginning.
func Shift(xs []vect.Vector, x vect.Vector) {

	for i := len(xs) - 1; i > 0; i-- {

		xs[i] = xs[i-1]
	}

	xs[0] = x
}
