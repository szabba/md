// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

package main

import (
	"flag"
	"fmt"
	"github.com/szabba/md/newton"
	"github.com/szabba/md/vect"
	"io"
	"log"
	"os"
	"strconv"
)

// A constant force
type ConstForce vect.Vector

func (f ConstForce) Accel(bs []*newton.Body, i int, dt float64) (a vect.Vector) {

	return vect.Vector(f).Scale(1 / bs[i].Mass())
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

func (picky *PickyForce) Accel(bs []*newton.Body, i int, dt float64) (a vect.Vector) {

	for _, ignored := range picky.zeroFor {

		if ignored == i {

			return vect.Zero
		}
	}

	return picky.force.Accel(bs, i, dt)
}

type ParticleRect struct {
	*newton.System
	rows, cols int
}

// Creates a rectangular grid of particles
//
// The program's behaviour when rows or cols are less than 1 is unspecified
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

	rect.SetForce(ConstForce(vect.Zero))

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

// Are the i-th and j-th particles neighbours?
func (rect *ParticleRect) Neighbours(i, j int) bool {

	xI, yI := rect.RowAndColumn(i)
	xJ, yJ := rect.RowAndColumn(j)

	nearInX := xI-1 == xJ || xJ == xI+1
	nearInY := yI-1 == yJ || yJ == yI+1

	sameX := xI == xJ
	sameY := yI == yJ

	colNeighbour := sameX && nearInY
	rowNeighbour := sameY && nearInX

	return colNeighbour || rowNeighbour
}

// Prepare a Hooke's force bidning neihbouring particles
func (rect *ParticleRect) Hooke(k float64) newton.Force {

	var h newton.Hooke

	h.Springs = make([][]newton.Spring, rect.Bodies())
	for i, _ := range h.Springs {

		h.Springs[i] = make([]newton.Spring, rect.Bodies())
		for j, _ := range h.Springs[i] {

			if rect.Neighbours(i, j) {

				h.Springs[i][j].K = k

			}
			h.Springs[i][j].L0 = 1
		}
	}

	return h
}

// Is the i-th particle near the center in it's resting position?
func (rect *ParticleRect) NearCenter(ith int) bool {

	rows, cols := rect.Size()

	center := vect.NewVector(float64(rows)/2, float64(cols)/2, 0)

	fromCenter := rect.RestingPosition(ith).Minus(center)

	return fromCenter.Norm() < 1
}

// Number of particles that are not near the center in their resting positions
func (rect *ParticleRect) ExceptCenter() []int {

	rows, cols := rect.Size()

	centerSize := 1
	if rows%2 == 0 {
		centerSize *= 2
	}
	if cols%2 == 0 {
		centerSize *= 2
	}

	rest := make([]int, rows*cols-centerSize)

	for i, j := 0, 0; i < len(rest); j++ {

		if !rect.NearCenter(j) {

			i, rest[i] = i+1, j
		}
	}

	return rest
}

// Prepare a constant force that only affects the center of the rectangle (ie
// ignores everything except it)
//
// Depending on the shape of the rectangle this will pull 1, 2 or 4 particles.
// The force applied per particle will be divided by this number.
func (rect *ParticleRect) CentralPull(pull vect.Vector) newton.Force {

	ignored := rect.ExceptCenter()

	pulled := float64(rect.Bodies() - len(ignored))

	return NewPicky(ConstForce(pull.Scale(1/pulled)), ignored...)
}

// Runs the simulation for the given number of steps at a time step of dt
// printing to writeTo
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

const usage string = `Usage of %s:

	Simulate a rectangular surface made of particles interconnected with
	strings.

		%s [options] ROWS COLS

	ROWS and COLS determines the shape and size of the rectangle. They must
	both be at least 1.

Options:
`

func Help() {

	program := os.Args[0]

	fmt.Fprintf(os.Stderr, usage, program, program)

	flag.PrintDefaults()
}

func main() {

	var (
		usage    bool
		p, k, dt float64
		steps    int
	)

	log.SetFlags(0)

	flag.Float64Var(
		&p, "pull", 1,
		"Magnitude of the vertical pulling force. When negative, the force pulls down.",
	)
	flag.Float64Var(&dt, "dt", 0.05, "Time step")
	flag.Float64Var(&k, "k", 1, "Hooke's constant")
	flag.IntVar(&steps, "steps", 5, "Simulation steps to perform")
	flag.BoolVar(&usage, "help", false, "Print usage string")

	flag.Parse()

	if usage {

		Help()

	} else if len(flag.Args()) != 2 {

		log.Fatal("Both ROWS and COLS need to be specified")

	} else {

		var (
			rows, cols int
			err        error
		)

		rows, err = strconv.Atoi(flag.Arg(0))
		if err != nil {

			log.Print("ROWS needs to be an integer")
			log.Fatal(err.Error())
		}
		cols, err = strconv.Atoi(flag.Arg(1))
		if err != nil {

			log.Print("COLS needs to be an integer")
			log.Fatal(err.Error())
		}

		rect := NewRect(rows, cols)
		rect.AddForce(rect.Hooke(k))
		rect.AddForce(rect.CentralPull(vect.UnitZ.Scale(p)))
		rect.Run(os.Stdout, dt, steps)
	}
}
