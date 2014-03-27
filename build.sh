for pkg in vect newton shm; do
	go fmt "./src/$pkg"
	go test "./src/$pkg"
done

go build ./src/shm && ./shm > shm.dat
