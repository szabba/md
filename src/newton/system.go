// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

package newton

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
	sys.bodies = make([]*Body, 0, bodyCount)

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
