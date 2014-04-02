// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

package newton

import (
	"github.com/szabba/md/vect"
)

// A 'picky' force, that doesn't affect some bodies
type PickyForce struct {
	force   Force
	zeroFor []int
}

// Creates a picky version of a force
func NewPicky(f Force, zeroFor ...int) Force {

	return &PickyForce{force: f, zeroFor: zeroFor}
}

func (picky *PickyForce) Accel(bs []*Body, i int, dt float64) (a vect.Vector) {

	for _, ignored := range picky.zeroFor {

		if ignored == i {

			return vect.Zero
		}
	}

	return picky.force.Accel(bs, i, dt)
}
