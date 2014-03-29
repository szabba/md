// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

package newton

import (
	"github.com/szabba/md/src/vect"
)

type Spring struct {
	K, L0 float64
}

type Hooke struct {
	Springs [][]Spring
}

func (h Hooke) Accel(bs []*Body, i int) (a vect.Vector) {

	f := vect.NewZeroVector()

	b := bs[i]

	for j, b2 := range bs {

		spring := h.Springs[i][j]

		dir, l := b2.Xs[0].Minus(b.Xs[0]).UnitAndNorm()

		f = f.Plus(dir.Scale(spring.K * (l - spring.L0)))
	}

	return f.Scale(1 / b.Mass())
}
