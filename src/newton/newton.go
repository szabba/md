// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

package newton

import (
	"github.com/szabba/md/src/vect"
	"math"
)

// An integrator algorithm
type Integrator interface {
	// The total number of states kept at once by the algorithm
	StateLen() int
	// The location of the current state in the sequence of states kept track
	// of
	CurrentAt() int
	// Performs the integration for a single body
	Integrate(b *Body, a vect.Vector, dt float64)
}

var (
	Euler Integrator = euler{}
	Verlet Integrator = verlet{}
)

type euler struct{}

func (_ euler) StateLen() int {
	return 1
}

func (_ euler) CurrentAt() int {
	return 0
}

func (_ euler) Integrate(b *Body, a vect.Vector, dt float64) {

	x0, v0 := b.Now()

	v := v0.Plus(a.Scale(dt))
	x := x0.Plus(v0.Scale(dt))

	b.Shift(x, v)
}

type verlet struct {}

func (_ verlet) StateLen() int {
	return 2
}

func (_ verlet) CurrentAt() int {
	return 1
}

func (_ verlet) Integrate(b *Body, a vect.Vector, dt float64) {

	xPast := b.XBefore(1)
	xNext := b.XNow().Scale(2).Minus(xPast).Plus(a.Scale(math.Pow(dt, 2)))

	b.Shift(xNext, vect.Zero)
	b.SetVNow(xNext.Minus(xPast).Scale(1 / (2 * dt)))
}

func Step(algo Integrator, bs []*Body, f Force, dt float64) {

	as := make([]vect.Vector, len(bs))

	for i, _ := range bs {

		as[i] = f.Accel(bs, i)
	}

	for i, body := range bs {

		algo.Integrate(body, as[i], dt)
	}
}
