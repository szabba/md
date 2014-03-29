// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

package newton

import (
	"github.com/szabba/md/vect"
)

// A force that acts upon a body
type Force interface {
	Accel(bs []*Body, i int) (a vect.Vector)
}

// A combination of simple forces
type SumForce []Force

func (sf SumForce) Accel(bs []*Body, i int) (a vect.Vector) {

	a = vect.Zero

	for _, f := range sf {

		a = a.Plus(f.Accel(bs, i))
	}

	return
}

// Combines multiple forces into one.
//
// If any of the combined forces is a SumForce, the result is flattened.
func Combine(fs ...Force) SumForce {

	simple := 0
	for _, f := range fs {

		if sum, ok := f.(SumForce); ok {

			simple += len(sum)

		} else {

			simple++
		}
	}

	simples := make([]Force, simple)
	i := 0

	for _, f := range fs {

		if sum, ok := f.(SumForce); ok {

			for _, f := range sum {

				simples[i] = f
				i++
			}

		} else {

			simples[i] = f
			i++
		}
	}

	return SumForce(fs)
}
