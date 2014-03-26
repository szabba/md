package main

func Shift(xs []float64, x float64) {

	for i := len(xs) - 1; i > 0; i-- {

		xs[i] = xs[i-1]
	}

	xs[0] = x
}

type Integrator func(xs, vs []float64, a, dt float64)

func Euler(xs, vs []float64, a, dt float64) {

	v := vs[0] + dt*a
	x := xs[0] + dt*v

	Shift(vs, v)
	Shift(xs, x)

	return
}

func main() {
}
