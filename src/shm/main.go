package main

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

func main() {
}
