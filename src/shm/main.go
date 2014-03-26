package main

import (
	"fmt"
	"math"
)

type Spring struct {
	K, L0 float64
}

type Hooke struct {
	Springs [][]Spring
}

func (h Hooke) Accel(bs []Body, i int) (a Vector) {

	f := NewZeroVector()

	b := bs[i]

	for j, b2 := range bs {

		spring := h.Springs[i][j]

		dir, l := b2.Xs[0].Minus(b.Xs[0]).UnitAndNorm()

		f = f.Plus(dir.Scale(spring.K * (l - spring.L0)))
	}

	return f.Scale(1 / b.M)
}

type SingleHooke struct {
	K float64
}

func (sh SingleHooke) Accel(bs []Body, i int) (a Vector) {

	b := bs[i]

	return b.Xs[0].Scale(-sh.K / b.M)
}

type Force interface {
	Accel(bs []Body, i int) (a Vector)
}

type AnalyticSHM struct {
	K, M float64
	A    Vector
}

func (ashm AnalyticSHM) XVAt(t float64) (x, v Vector) {

	omega := math.Sqrt(ashm.K / ashm.M)

	x = ashm.A.Scale(math.Cos(omega * t))
	v = ashm.A.Negate().Scale(omega * math.Sin(omega*t))

	return
}

func (ashm AnalyticSHM) Force() Force {

	return Force(SingleHooke{K: ashm.K})
}

func (ashm AnalyticSHM) ForEuler() []Body {

	x0, v0 := ashm.XVAt(0)

	return []Body{
		Body{
			Xs: []Vector{x0},
			Vs: []Vector{v0},
			M:  ashm.M,
		},
	}
}

func (ashm AnalyticSHM) ForVerlet(dt float64) []Body {

	x0, v0 := ashm.XVAt(0)
	xPrev, vPrev := ashm.XVAt(-dt)

	bs := []Body{
		Body{
			Xs: []Vector{x0, xPrev},
			Vs: []Vector{v0, vPrev},
			M:  ashm.M,
		},
	}

	ashm.Step(Verlet, bs, ashm.Force(), dt)

	return bs
}

func (ashm AnalyticSHM) DataHeader() {

	//fmt.Printf("# ")
	fmt.Printf("t x v E Ek U ")
	fmt.Printf("x_e v_e E_e Ek_e U_e x_e_resid ")
	fmt.Printf("x_v v_e E_v Ek_v U_v x_v_resid ")
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
		ashm.Step(Euler, eulerState, force, dt)

		ashm.VerletFormat(verletState, x)
		ashm.Step(Verlet, verletState, force, dt)

		fmt.Println()

		t += dt
		steps--
	}
}

var e_x = NewVector(1, 0, 0)

func (ashm AnalyticSHM) Analytic(t float64) (x Vector) {

	x, v := ashm.XVAt(t)

	kinetic := ashm.M * math.Pow(v.Norm(), 2) / 2
	potential := ashm.A.Norm() * math.Pow(x.Norm(), 2) / 2

	totalE := kinetic + potential

	fmt.Printf(
		"%f %f %f %f %f ",
		x.Dot(e_x), v.Dot(e_x), totalE, kinetic, potential,
	)

	return x
}

func (ashm AnalyticSHM) Step(alg Integrator, bs []Body, f Force, dt float64) {

	as := make([]Vector, len(bs))

	for i, _ := range bs {

		as[i] = f.Accel(bs, i)
	}

	for i, body := range bs {

		alg(body.Xs, body.Vs, as[i], dt)
	}
}

func (ashm AnalyticSHM) EulerFormat(bs []Body, x Vector) {

	xE, vE := bs[0].Xs[0].Dot(e_x), bs[0].Vs[0].Dot(e_x)

	kinetic := ashm.M * math.Pow(vE, 2) / 2
	potential := ashm.K * math.Pow(xE, 2) / 2

	total := kinetic + potential

	residue := math.Abs(x.Dot(e_x) - xE)

	fmt.Printf(
		"%f %f %f %f %f %f ",
		xE, vE, total, kinetic, potential, residue,
	)
}

func (ashm AnalyticSHM) VerletFormat(bs []Body, x Vector) {

	xV, vV := bs[0].Xs[1].Dot(e_x), bs[0].Xs[1].Dot(e_x)

	kinetic := ashm.M * math.Pow(vV, 2) / 2
	potential := ashm.K * math.Pow(xV, 2) / 2

	total := kinetic + potential

	residue := math.Abs(x.Dot(e_x) - xV)

	fmt.Printf(
		"%f %f %f %f %f %f ",
		xV, vV, total, kinetic, potential, residue,
	)
}

func main() {

	shm := AnalyticSHM{
		K: 1, M: 1, A: NewVector(1, 0, 0),
	}

	shm.Run(0.05, 5000)

}
