// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

package main

import (
	"fmt"
	"github.com/szabba/md/src/newton"
	"github.com/szabba/md/src/vect"
	"math"
)

type SingleHooke struct {
	K float64
}

func (sh SingleHooke) Accel(bs []*newton.Body, i int) (a vect.Vector) {

	b := bs[i]

	return b.Xs[0].Scale(-sh.K / b.Mass())
}

type AnalyticSHM struct {
	K, M float64
	A    vect.Vector
}

func (ashm AnalyticSHM) XVAt(t float64) (x, v vect.Vector) {

	omega := math.Sqrt(ashm.K / ashm.M)

	x = ashm.A.Scale(math.Cos(omega * t))
	v = ashm.A.Negate().Scale(omega * math.Sin(omega*t))

	return
}

func (ashm AnalyticSHM) Force() newton.Force {

	return newton.Force(SingleHooke{K: ashm.K})
}

func (ashm AnalyticSHM) ForEuler() *newton.System {

	sys := newton.NewSystem(newton.Euler, 1)

	sys.SetForce(ashm.Force())

	b := sys.Body(0)

	b.SetMass(ashm.M)
	b.SetNow(ashm.XVAt(0))

	return sys
}

func (ashm AnalyticSHM) ForVerlet(dt float64) []*newton.Body {

	b := newton.NewBody(newton.Verlet)
	b.SetMass(ashm.M)
	b.Shift(ashm.XVAt(-dt))
	b.Shift(ashm.XVAt(0))

	bs := []*newton.Body{b}

	newton.Step(newton.Verlet, bs, ashm.Force(), dt)

	return bs
}

func (ashm AnalyticSHM) DataHeader() {

	//fmt.Printf("# ")
	fmt.Printf("t x v E Ek U ")
	fmt.Printf("x_e v_e E_e Ek_e U_e x_e_resid ")
	fmt.Printf("x_v v_v E_v Ek_v U_v x_v_resid ")
	fmt.Println()
}

func (ashm AnalyticSHM) Run(dt float64, steps int) {

	ashm.DataHeader()

	force := ashm.Force()

	eulerState := ashm.ForEuler()

	verletState := ashm.ForVerlet(dt)

	for t := 0.0; steps > 0; {

		fmt.Printf("%f ", t)

		x := ashm.Analytic(t)

		ashm.EulerFormat(eulerState, x)
		eulerState.Step(dt)

		ashm.VerletFormat(verletState, x)
		newton.Step(newton.Verlet, verletState, force, dt)

		fmt.Println()

		t += dt
		steps--
	}
}

func (ashm AnalyticSHM) Analytic(t float64) (x vect.Vector) {

	x, v := ashm.XVAt(t)

	kinetic := ashm.M * math.Pow(v.Norm(), 2) / 2
	potential := ashm.A.Norm() * math.Pow(x.Norm(), 2) / 2

	totalE := kinetic + potential

	fmt.Printf(
		"%f %f %f %f %f ",
		x.Dot(vect.UnitX), v.Dot(vect.UnitX), totalE, kinetic, potential,
	)

	return x
}

func (ashm AnalyticSHM) EulerFormat(sys *newton.System, x vect.Vector) {

	xE, vE := sys.Body(0).Now()

	xVal, vVal := xE.Dot(vect.UnitX), vE.Dot(vect.UnitX)

	kinetic := ashm.M * math.Pow(vVal, 2) / 2
	potential := ashm.K * math.Pow(xVal, 2) / 2

	total := kinetic + potential

	residue := math.Abs(x.Dot(vect.UnitX) - xVal)

	fmt.Printf(
		"%f %f %f %f %f %f ",
		xVal, vVal, total, kinetic, potential, residue,
	)
}

func (ashm AnalyticSHM) VerletFormat(bs []*newton.Body, x vect.Vector) {

	xV, vV := bs[0].Xs[1].Dot(vect.UnitX), bs[0].Vs[1].Dot(vect.UnitX)

	kinetic := ashm.M * math.Pow(vV, 2) / 2
	potential := ashm.K * math.Pow(xV, 2) / 2

	total := kinetic + potential

	residue := math.Abs(x.Dot(vect.UnitX) - xV)

	fmt.Printf(
		"%f %f %f %f %f %f ",
		xV, vV, total, kinetic, potential, residue,
	)
}
