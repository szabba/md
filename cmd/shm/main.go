// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

package main

import (
	"github.com/szabba/md/vect"
)

func main() {

	shm := AnalyticSHM{
		K: 1, M: 1, A: vect.NewVector(1, 0, 0),
	}

	shm.Run(0.05, 5000)

}
