// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

package main

import (
	"github.com/szabba/md/newton"
)

// Creates a system containing a rectangular grid of m times n particles.
func SetUpRect(m, n int) *newton.System {

	sys := newton.NewSystem(newton.Verlet, m*n)

	for i := 0; i < sys.Bodies(); i++ {

		sys.Body(i).SetMass(1)
	}

	return sys
}

func main() {
}
