// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

package newton

import (
	"github.com/szabba/md/vect"
)

// A molecular dynamics system
type System struct {
	algo   Integrator
	bodies []*Body
	force  Force
}

// Construct an empty system that will use the given integrator and has space
// for the given number of bodies
func NewSystem(algo Integrator, bodyCount int) *System {

	sys := new(System)

	sys.algo = algo

	sys.bodies = make([]*Body, bodyCount)
	for i, _ := range sys.bodies {

		sys.bodies[i] = NewBody(sys.algo)
	}

	return sys
}

// Set the system force
func (sys *System) SetForce(f Force) {

	sys.force = f
}

// Add a force to the system
func (sys *System) AddForce(f Force) {

	if sys.force != nil {

		sys.force = Combine(sys.force, f)

	} else {

		sys.SetForce(f)
	}
}

// Perform an integration step with the given dt
func (sys *System) Step(dt float64) {

	as := make([]vect.Vector, len(sys.bodies))

	for i, _ := range sys.bodies {

		as[i] = sys.force.Accel(sys.bodies, i, dt)
	}

	for i, body := range sys.bodies {

		sys.algo.Integrate(body, as[i], dt)
	}
}

// The number of bodies in the system
func (sys *System) Bodies() int {

	return len(sys.bodies)
}

// The i-th body within the system
func (sys *System) Body(i int) *Body {

	return sys.bodies[i]
}
