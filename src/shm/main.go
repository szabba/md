package main

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

	shm := AnalyticSHM{
		K: 1, M: 1, A: NewVector(1, 0, 0),
	}

	shm.Run(0.05, 500)

}
