// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

package main

import (
	"fmt"
	"github.com/szabba/md/newton"
	"github.com/szabba/md/vect"
	"io"
	"os"
)

// A force that is always zero
type ZeroForce struct{}

func (_ ZeroForce) Accel(bs []*newton.Body, i int) (a vect.Vector) {

	return vect.Zero
}

// A 'picky' force, that doesn't affect some bodies
type PickyForce struct {
	force   newton.Force
	zeroFor []int
}

// Creates a picky version of a force
func NewPicky(f newton.Force, zeroFor ...int) newton.Force {

	return &PickyForce{force: f, zeroFor: zeroFor}
}

func (picky *PickyForce) Accel(bs []*newton.Body, i int) (a vect.Vector) {

	for _, ignored := range picky.zeroFor {

		if ignored == i {

			return vect.Zero
		}
	}

	return picky.force.Accel(bs, i)
}

type ParticleRect struct {
	*newton.System
	rows, cols int
}

// Creates a rectangular grid of particles
func NewRect(rows, cols int) *ParticleRect {

	rect := &ParticleRect{
		rows: rows, cols: cols,
	}

	rect.System = newton.NewSystem(newton.Verlet, rows*cols)

	for i := 0; i < rect.Bodies(); i++ {

		b := rect.Body(i)

		b.SetMass(1)

		pos := rect.RestingPosition(i)

		b.Shift(pos, vect.Zero)
		b.Shift(pos, vect.Zero)

	}

	rect.SetForce(ZeroForce{})

	return rect
}

// The row and column in which the i-th particle is
func (rect *ParticleRect) RowAndColumn(ith int) (row, col int) {

	return ith % rect.rows, ith / rect.rows
}

// Initial, resting position of the i-th particle
func (rect *ParticleRect) RestingPosition(ith int) vect.Vector {

	row, col := rect.RowAndColumn(ith)

	return vect.UnitX.Scale(float64(row)).Plus(
		vect.UnitY.Scale(float64(col)),
	)
}

// Dimmensions of the rectangle
func (rect *ParticleRect) Size() (rows, cols int) {

	return rect.rows, rect.cols
}

func (rect *ParticleRect) Run(writeTo io.Writer, dt float64, steps int) {

	format := &Formatter{rect: rect, writeTo: writeTo}

	format.Header()

	for i := 0; i < steps; i++ {

		format.Frame()

		rect.Step(dt)
	}
}

// An output formatting type
type Formatter struct {
	rect    *ParticleRect
	writeTo io.Writer
}

// Formats a data header
func (f Formatter) Header() {

	fmt.Fprintf(f.writeTo, "%d\n\n", f.rect.Bodies())
}

// Formats the description of ball states
func (f Formatter) Frame() {

	for i := 0; i < f.rect.Bodies(); i++ {

		b := f.rect.Body(i)

		x, v := b.Now()

		fmt.Fprintf(
			f.writeTo, "%d %f %f %f %f %f %f\n", i,
			x[0], x[1], x[2],
			v[0], v[1], v[2],
		)

	}
	fmt.Fprintf(f.writeTo, "\n")
}

func main() {

	rect := NewRect(2, 4)
	rect.Run(os.Stdout, 0.05, 3)
}
