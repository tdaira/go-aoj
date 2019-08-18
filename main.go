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
	vals []int
}


func (p Point) Dist(target *Point) int {
    diffX := Abs(p.vals[0] - target.vals[0])
    diffY := Abs(p.vals[1] - target.vals[1])
    if diffX > diffY {
        return diffX
    } else {
        return diffY
    }
}

func (p Point) Eq(target *Point) bool {
    return p.vals[0] == target.vals[0] && p.vals[1] == target.vals[1]
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
    return p.Points[i].vals[0] < p.Points[j].vals[0]
}

type ByY struct {
    Points
}

func (p ByY) Less(i, j int) bool {
    return p.Points[i].vals[1] < p.Points[j].vals[1]
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
            if points[leftIndex].vals[axis] >= pivot.vals[axis] {
                break
            }
            leftIndex++
        }
        for rightIndex >= 0 {
            if points[rightIndex].vals[axis] < pivot.vals[axis] {
                break
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

func getNeighbor(node *Node, point *Point) (*Node, int) {
	var minDist = math.MaxInt64
    var minNode *Node
    searchKdTree(node, point, &minDist, &minNode)
    return minNode, minDist
}

func searchKdTree(node *Node, point *Point, minDist *int, minNode **Node)  {
    if node == nil {
        return
    }

    // Calc distance between current point and searching point.
    dist := point.Dist(node.current)
    // Update minimum point.
    if dist < *minDist {
        *minDist = dist
        *minNode = node
    }

    // Decide direction for depth-first search.
    var dir int
    if point.vals[node.axis] < node.current.vals[node.axis] {
        dir = 0
    } else {
        dir = 1
    }

    // Search high priority node.
    if dir == 0 {
        searchKdTree(node.left, point, minDist, minNode)
    } else {
        searchKdTree(node.right, point, minDist, minNode)
    }

    // Search secondary priority node if it has possibility of containing point.
    diff := Abs(node.current.vals[node.axis] - point.vals[node.axis])
    if diff <= *minDist {
        if dir == 0 {
            searchKdTree(node.right, point, minDist, minNode)
        } else {
            searchKdTree(node.left, point, minDist, minNode)
        }
    }

    return
}

func getMinNode(node *Node, axis int) *Node {
    if node == nil {
        return nil
    }

    var minNode = node
    if node.axis == axis {
        leftMinNode := getMinNode(node.left, axis)
        if leftMinNode != nil {
            if leftMinNode.current.vals[axis] < minNode.current.vals[axis] {
                minNode = leftMinNode
            }
        }
    } else {
        leftMinNode := getMinNode(node.left, axis)
        if leftMinNode != nil {
            if leftMinNode.current.vals[axis] < minNode.current.vals[axis] {
                minNode = leftMinNode
            }
        }
        rightMinNode := getMinNode(node.right, axis)
        if rightMinNode != nil {
            if rightMinNode.current.vals[axis] < minNode.current.vals[axis] {
                minNode = rightMinNode
            }
        }
    }

    return minNode
}

func deleteNode(root *Node, point *Point) {
    node, _ := getNeighbor(root, point)
	leaf := deleteNodeRec(node)
	minDist := math.MaxInt64
    deleteLeafNode(root, leaf, &minDist)
}

func deleteNodeRec(node *Node) *Node {
    minNode := node
	if node.right != nil {
        minNode = getMinNode(node.right, node.axis)
        if minNode != nil {
            node.current = minNode.current
            return deleteNodeRec(minNode)
        }
    }
	if node.left != nil && node.right == nil {
        minNode := getMinNode(node.left, node.axis)
        if minNode != nil {
            node.current = minNode.current
            node.right = node.left
            node.left = nil
            return deleteNodeRec(minNode)
        }
    }
    // Return leaf node.
    return node
}

func deleteLeafNode(node *Node, leaf *Node, minDist *int) {
	if node == nil {
	    return
    }
    if node.left != nil {
        if node.left == leaf {
            node.left = nil
            return
        }
    }
    if node.right != nil {
        if node.right == leaf {
            node.right = nil
            return
        }
    }

    // Calc distance between current point and searching point.
    dist := leaf.current.Dist(node.current)
    // Update minimum point.
    if dist < *minDist {
        *minDist = dist
    }

    // Decide direction for depth-first search.
    var dir int
    if leaf.current.vals[node.axis] < node.current.vals[node.axis] {
        dir = 0
    } else {
        dir = 1
    }

    // Search high priority node.
    if dir == 0 {
        deleteLeafNode(node.left, leaf, minDist)
    } else {
        deleteLeafNode(node.right, leaf, minDist)
    }

    // Search secondary priority node if it has possibility of containing point.
    diff := Abs(node.current.vals[node.axis] - leaf.current.vals[node.axis])
    if diff <= *minDist {
        if dir == 0 {
            deleteLeafNode(node.right, leaf, minDist)
        } else {
            deleteLeafNode(node.left, leaf, minDist)
        }
    }
}

type Edge struct {
    distance int
    from *Point
    to *Point
}

type BinaryNode struct {
    Value *Edge
    Left  *BinaryNode
    Right *BinaryNode
}

type BinaryTree struct {
    Root *BinaryNode
}

func NewBinaryNode(val *Edge) *BinaryNode {
    return &BinaryNode{val, nil, nil}
}

func (n *BinaryTree) Add(val *Edge) {
    if n.Root == nil {
        n.Root = NewBinaryNode(val)
    } else {
        n.Root.Add(val)
    }
}

func (n *BinaryNode) Add(val *Edge) {
    if val.distance <= n.Value.distance {
        if n.Left != nil {
            n.Left.Add(val)
        } else {
            n.Left = NewBinaryNode(val)
        }
    } else {
        if n.Right != nil {
            n.Right.Add(val)
        } else {
            n.Right = NewBinaryNode(val)
        }
    }
}
func (n *BinaryTree) Delete(val *Edge) {
	leaf := n.Root.SearchDelete(val)
	if leaf != nil {
        n.Root = n.Root.DeleteLeafNode(leaf)
    }
}

func (n *BinaryNode) SearchDelete(val *Edge) *BinaryNode {
	if val == n.Value {
	    return n.Delete()
    }
    if val.distance <= n.Value.distance {
        return n.Left.SearchDelete(val)
    } else {
        return n.Right.SearchDelete(val)
    }
}

func (n *BinaryNode) Delete() *BinaryNode {
	maxNode := n.Left.GetMaxNode()
	if maxNode == nil {
		if n.Right != nil {
		    n.Value = n.Right.Value
            n.Left = n.Right.Left
            n.Right = n.Right.Right
            return nil
        }
		return n
    }
    n.Value = maxNode.Value
    return maxNode
}

func (n *BinaryNode) DeleteLeafNode(node *BinaryNode) *BinaryNode {
	if n == node {
		if n.Left != nil {
		    return n.Left
        }
	    return nil
    }
    if node.Value.distance <= n.Value.distance {
    	if n.Left != nil {
            n.Left = n.Left.DeleteLeafNode(node)
        }
    } else {
        if n.Right != nil {
            n.Right = n.Right.DeleteLeafNode(node)
        }
    }
	return n
}

func (n *BinaryTree) GetMin() *Edge {
    return n.Root.GetMin()
}


func (n *BinaryNode) GetMin() *Edge {
    var min *Edge
    if n.Left != nil {
        min = n.Left.GetMin()
    }
    if min == nil {
        min = n.Value
    }
    return min
}

func (n *BinaryNode) GetMaxNode() *BinaryNode {
    var max *BinaryNode
    if n == nil {
        return nil
    }
    if n.Right != nil {
        max = n.Right.GetMaxNode()
    }
    if max == nil {
        max = n
    }
    return max
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
            vals: []int{x, y},
        }
    }

    tree := createKdTree(points[1:], 0)

    total := 0
    connected := map[*Point]bool{}
    current := points[0]
    connected[current] = true
    bt := new(BinaryTree)
    for i := 0; i < n-1; i++ {
        minNode, minDistance := getNeighbor(tree, current)
        bt.Add(&Edge{minDistance, current, minNode.current})
        var min *Edge
        for {
            min = bt.GetMin()
            if _, ok := connected[min.to]; !ok {
               break
            }
            bt.Delete(min)
            minNode, minDistance := getNeighbor(tree, min.from)
            bt.Add(&Edge{minDistance, min.from, minNode.current})
        }
        total += min.distance
        current = min.to
        connected[min.to] = true
        deleteNode(tree, min.to)
    }

    fmt.Println(total)
}
