package main

import (
	"fmt"
	"math"
)

type Body struct {
	Xs, Vs []Vector
	M      float64
}

// Shifts all the values in xs by one and puts x at the beginning.
func Shift(xs []Vector, x Vector) {

	for i := len(xs) - 1; i > 0; i-- {

		xs[i] = xs[i-1]
	}

	xs[0] = x
}

type Integrator func(xs, vs []Vector, a Vector, dt float64)

// Performs a step of an Euler integration
func Euler(xs, vs []Vector, a Vector, dt float64) {

	v := vs[0].Plus(a.Scale(dt))
	x := xs[0].Plus(v.Scale(dt))

	Shift(vs, v)
	Shift(xs, x)
}

// Performs a step of a Verlet integrator
//
// Note that v[0] will not be calculated until the next step
func Verlet(xs, vs []Vector, a Vector, dt float64) {

	xNext := xs[0].Scale(2).Minus(xs[1]).Plus(a.Scale(dt * dt))
	vs[0] = xNext.Minus(xs[1]).Scale(1 / (2 * dt))

	Shift(vs, NewZeroVector())
	Shift(xs, xNext)
}

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

	return []Body{
		Body{
			Xs: []Vector{x0, xPrev},
			Vs: []Vector{v0, vPrev},
			M:  ashm.M,
		},
	}
}

func (ashm AnalyticSHM) DataHeader() {

	fmt.Printf("#")
	fmt.Printf("t x E Ek U ")
	fmt.Printf("x_e v_e E_e Ek_e U_e x_e_resid ")
	fmt.Printf("x_v v_e E_v Ek_v U_v x_v_resid\n")
}

func (ashm AnalyticSHM) Run(dt float64, steps int) {

	ashm.DataHeader()

	//force := ashm.Force()

	//eulerState := ashm.ForEuler()
	//verletState := ashm.ForVerlet(dt)

	for t := 0.0; steps > 0; {

		fmt.Printf("%f ", t)

		_ = ashm.Analytic(t)

		fmt.Println()

		t += dt
		steps--
	}
}

func (ashm AnalyticSHM) Analytic(t float64) (x Vector) {

	x, v := ashm.XVAt(t)

	kinetic := ashm.M * math.Pow(v.Norm(), 2) / 2
	potential := ashm.A.Norm() * math.Pow(x.Norm(), 2) / 2

	totalE := kinetic + potential

	fmt.Printf("%f %f %f %f ", x.Norm(), totalE, kinetic, potential)

	return x
}

func main() {

	shm := AnalyticSHM{
		K: 1, M: 1, A: NewVector(1, 0, 0),
	}

	shm.Run(0.01, 500)

}
