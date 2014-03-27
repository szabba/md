package main

import (
	"../vect"
)

func main() {

	shm := AnalyticSHM{
		K: 1, M: 1, A: vect.NewVector(1, 0, 0),
	}

	shm.Run(0.05, 5000)

}
