package main

import (
    "bufio"
    "fmt"
    "math"
    "os"
    "sort"
    "strconv"
    "strings"
)

type Point struct {
	x int
    y int
	connected bool
}


func (p Point) Dist(target *Point) int {
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

func Abs(x int) int {
	if x < 0 {
		return -x
	}
	return x
}


type Points []*Point


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
            points[0],
            axis,
            nil,
            nil,
        }
    }

    // Select pivot for split.
    pivotCandidate := []*Point {
        points[0],
        points[len(points) / 2],
        points[len(points) - 1],
    }

    // Select pivot.
    var pivot *Point
    if axis == 0 {
        sort.Sort(ByX{pivotCandidate})
        pivot = pivotCandidate[1]
    } else {
        sort.Sort(ByY{pivotCandidate})
    }
    pivot = pivotCandidate[1]

    // Delete pivot from slice.
    if pivot.Eq(points[0]) {
        points = points[1:]
    } else if pivot.Eq(points[len(points) / 2]) {
        points = append(points[:len(points) / 2], points[len(points) / 2 + 1:]...)
    } else if pivot.Eq(points[len(points) - 1]) {
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

    // Create node.
    return &Node{
        pivot,
        axis,
        createKdTree(points[:leftIndex], depth + 1),
        createKdTree(points[leftIndex:], depth + 1),
    }
}

func getNeighbor(node *Node, point *Point) *Point {
	var minDist = math.MaxInt64
    var minPoint *Point
    searchKdTree(node, point, &minDist, &minPoint)
    return minPoint
}

func searchKdTree(node *Node, point *Point, minDist *int, minPoint **Point)  {
    if node == nil {
        return
    }

    // Calc distance between current point and searching point.
    dist := point.Dist(node.current)
    // Update minimum point.
    if dist < *minDist && !node.current.connected {
        *minDist = dist
        *minPoint = node.current
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
    } else {
        diff := Abs(node.current.y - point.y)
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
	var n int
    fmt.Scan(&n)
    points := make([]*Point, n)
    sc := bufio.NewScanner(os.Stdin)
    for i := 0; i < n; i++ {
        sc.Scan()
        input := strings.Split(sc.Text(), " ")
        x, _ := strconv.Atoi(input[0])
        y, _ := strconv.Atoi(input[1])
        points[i] = &Point{
            x: x,
            y: y,
            connected: false,
        }
    }

    tree := createKdTree(points, 0)

    total := 0
    connected := map[*Point]bool{}
    neighborMap := map[*Point]*Point{}
    current := points[0]
    current.connected = true
    connected[current] = true
    for i := 0; i < n-1; i++ {
        n := getNeighbor(tree, current)
        neighborMap[current] = n
        minDist := math.MaxInt64
        var minPoint *Point
        for from, to := range neighborMap {
            if _, ok := connected[to]; ok {
                to = getNeighbor(tree, from)
                neighborMap[from] = to
            }
            dist := from.Dist(to)
            if dist < minDist {
                minDist = dist
                minPoint = to
            }
        }
        total += minDist
        current = minPoint
        current.connected = true
        connected[minPoint] = true
    }

    fmt.Println(total)
}
