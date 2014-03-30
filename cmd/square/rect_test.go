// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

package main

import (
	"testing"
)

func TestRowAndColumn(t *testing.T) {

	rows, cols := 10, 10
	rect := NewRect(rows, cols)

	for y := 0; y < cols; y++ {
		for x := 0; x < rows; x++ {

			i := y*cols + x

			row, col := rect.RowAndColumn(i)

			if row != x || col != y {

				t.Errorf(
					"%d-th particle should be at (%d, %d) not (%d, %d)",
					i, x, y, row, col,
				)
			} else {

				t.Logf(
					"%d-th particle is at (%d, %d), as expected",
					i, x, y,
				)
			}
		}
	}
}
