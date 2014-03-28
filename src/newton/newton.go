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
}

// Shifts all the values in xs by one and puts x at the beginning.
func Shift(xs []vect.Vector, x vect.Vector) {

	for i := len(xs) - 1; i > 0; i-- {

		xs[i] = xs[i-1]
	}

	xs[0] = x
}

type Integrator func(xs, vs []vect.Vector, a vect.Vector, dt float64)

// Performs a step of an Euler integration
func Euler(xs, vs []vect.Vector, a vect.Vector, dt float64) {

	v := vs[0].Plus(a.Scale(dt))
	x := xs[0].Plus(vs[0].Scale(dt))

	Shift(vs, v)
	Shift(xs, x)
}

// Performs a step of a Verlet integrator
//
// Note that v[0] will not be calculated until the next step
func Verlet(xs, vs []vect.Vector, a vect.Vector, dt float64) {

	xNext := xs[0].Scale(2).Minus(xs[1]).Plus(a.Scale(dt * dt))
	vs[0] = xNext.Minus(xs[1]).Scale(1 / (2 * dt))

	Shift(vs, vect.Zero)
	Shift(xs, xNext)
}

func Step(alg Integrator, bs []Body, f Force, dt float64) {

	as := make([]vect.Vector, len(bs))

	for i, _ := range bs {

		as[i] = f.Accel(bs, i)
	}

	for i, body := range bs {

		alg(body.Xs, body.Vs, as[i], dt)
	}
}
