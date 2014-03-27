package main

type Body struct {
	Xs, Vs []Vector
	M      float64
}

type Force interface {
	Accel(bs []Body, i int) (a Vector)
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
	x := xs[0].Plus(vs[0].Scale(dt))

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
