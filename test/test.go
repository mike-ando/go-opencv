package test

/*
#cgo LDFLAGS: -lstdc++
void runalgo();

*/
import "C"

// GoRunAlgo
func GoRunAlgo() {
	C.runalgo()
}
