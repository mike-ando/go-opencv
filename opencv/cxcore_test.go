package opencv

import (
	"testing"
	"fmt"
)

func TestContourArea(t *testing.T){
	img := CreateImage(100,100,IPL_DEPTH_8U,1)
	defer img.Release()
	// Creating object of area 100
	shape := []Point{
		Point{10,10}, 
		Point{20,10},
		Point{20,20},
		Point{10,20},
		Point{10,10}}
	for i:=0; i<(len(shape)-1); i++{
		Line(img, shape[i], shape[i+1], ScalarAll(255), 1, 8, 0)		
	}
	c,n := FindContours(img, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, nil)
	fmt.Printf("Found %d contours\n", n)
	a := ContourArea(c, 0)
	fmt.Printf("Area of contour is %v \n", a)
	p := ArcLength(c,0)
	fmt.Printf("Perimeter of contour is %v \n", p)
}

func TestConvexityDefects(t *testing.T) {
	img := CreateImage(100,100,IPL_DEPTH_8U,1)
	defer img.Release()
	// Object with convex defect of 10 and 3
	shape := []Point{
		Point{10,10}, 
		Point{20,10},
		Point{20,20},
		Point{30,20},
		Point{30,10}, 
		Point{40,10}, 
		Point{40,40},
		Point{30,40},
		Point{30,37},
		Point{20,37},
		Point{20,40},
		Point{10,40},
		Point{10,10}}
	for i:=0; i<(len(shape)-1); i++{
		Line(img, shape[i], shape[i+1], ScalarAll(255), 1, 8, 0)		
	}
	c,n := FindContours(img, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, nil)
	fmt.Printf("Found %d contours\n", n)
	a := ContourArea(c, 0)
	fmt.Printf("Area of contour is %v \n", a)
	p := ArcLength(c,0)
	fmt.Printf("Perimeter of contour is %v \n", p)
	d, n2 := ConvexityDefects(c, 0)
	fmt.Printf("Found %d defects\n", n2)
	for i:=0; i<n2; i++{
		fmt.Printf("Found defect of size %v \n", d[i])
	}
}

