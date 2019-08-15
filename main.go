package main

import (
    "fmt"
    "math"
    "sort"
)

type Point struct {
	x int64
    y int64
}


func (p Point) Dist(target *Point) int64 {
    diffX := Abs(p.x - target.x)
    diffY := Abs(p.y - target.y)
    if diffX > diffY {
        return diffX
    } else {
        return diffY
    }
}

func (p Point) Eq(target *Point) bool {
    return p.x == target.x && p.y == target.y
}

func Abs(x int64) int64 {
	if x < 0 {
		return -x
	}
	return x
}


type Points []Point


func (p Points) Len() int {
    return len(p)
}

func (p Points) Swap(i, j int) {
    p[i], p[j] = p[j], p[i]
}

type ByX struct {
    Points
}

func (p ByX) Less(i, j int) bool {
    return p.Points[i].x < p.Points[j].x
}

type ByY struct {
    Points
}

func (p ByY) Less(i, j int) bool {
    return p.Points[i].y < p.Points[j].y
}

type Node struct {
    current *Point
    axis int
    left *Node
    right *Node
}

func createKdTree (points Points, depth int) *Node {
    if len(points) == 0 {
    	return nil
	}
    // Select axis in this depth.
    axis := depth % 2
    if len(points) == 1 {
        return &Node{
            &points[0],
            axis,
            nil,
            nil,
        }
    }

    // Select pivot for split.
    pivotCandidate := []Point {
        points[0],
        points[len(points) / 2],
        points[len(points) - 1],
    }

    // Select pivot.
    var pivot Point
    if axis == 0 {
        sort.Sort(ByX{pivotCandidate})
        pivot = pivotCandidate[1]
    } else {
        sort.Sort(ByY{pivotCandidate})
    }
    pivot = pivotCandidate[1]

    // Delete pivot from slice.
    if pivot.Eq(&points[0]) {
        points = points[1:]
    } else if pivot.Eq(&points[len(points) / 2]) {
        points = append(points[:len(points) / 2], points[len(points) / 2 + 1:]...)
    } else if pivot.Eq(&points[len(points) - 1]) {
        points = points[:len(points) - 1]
    }

    // Split by pivot.
    leftIndex := 0
    rightIndex := len(points) - 1
    for {
        for leftIndex < len(points) {
            if axis == 0 {
                if points[leftIndex].x >= pivot.x {
                    break
                }
            } else {
                if points[leftIndex].y >= pivot.y {
                    break
                }
            }
            leftIndex++
        }
        for rightIndex >= 0 {
            if axis == 0 {
                if points[rightIndex].x < pivot.x {
                    break
                }
            } else {
                if points[rightIndex].y < pivot.y {
                    break
                }
            }
            rightIndex--
        }
        if leftIndex >= rightIndex {
            break
        }
        points.Swap(leftIndex, rightIndex)
    }
    fmt.Println("pivot: ", pivot)
    fmt.Println("left: ", points[:leftIndex])
    fmt.Println("right: ", points[leftIndex:])

    // Create node.
    return &Node{
        &pivot,
        axis,
        createKdTree(points[:leftIndex], depth + 1),
        createKdTree(points[leftIndex:], depth + 1),
    }
}

func getNeighbor(node *Node, point *Point) *Point {
	var minDist int64 = math.MaxInt64
    var minPoint Point
    searchKdTree(node, point, &minDist, &minPoint)
    return &minPoint
}

func searchKdTree(node *Node, point *Point, minDist *int64, minPoint *Point)  {
    if node == nil {
        return
    }

    // Calc distance between current point and searching point.
    dist := point.Dist(node.current)
    // Update minimum point.
    if dist < *minDist {
        *minDist = dist
        *minPoint = *node.current
    }

    // Decide direction for depth-first search.
    var dir int
    if node.axis == 0 {
        if point.x < node.current.x {
            dir = 0
        } else {
        	dir = 1
        }
    } else {
        if point.y < node.current.y {
            dir = 0
        } else {
            dir = 1
        }
    }

    // Search high priority node.
    if dir == 0 {
        searchKdTree(node.left, point, minDist, minPoint)
    } else {
        searchKdTree(node.right, point, minDist, minPoint)
    }

    // Search secondary priority node if it has possibility of containing point.
    if node.axis == 0 {
        diff := Abs(node.current.x - point.x)
        if diff <= *minDist {
            if dir == 0 {
                searchKdTree(node.right, point, minDist, minPoint)
            } else {
                searchKdTree(node.left, point, minDist, minPoint)
            }
        }
    }

    return
}

func main() {
    var points Points = []Point {
        {1, 1},
        {1, 10},
        {10, 1},
        {10, 10},
        {100, 50},
        {5, 5}}

    var tree = createKdTree(points, 0)
    fmt.Println(getNeighbor(tree, &Point{80, 50}))
}
