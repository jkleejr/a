using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Diagnostics;   // timer performance gain

public class Pathfinding : MonoBehaviour
{
    public Transform seeker, target;

    Grid grid;   // reference to Grid (for world positions as nodes)

    void Awake() {
        grid = GetComponent<Grid>();
    }

    void Update() {
        //if (Input.GetButtonDown("Jump")) {  
            FindPath(seeker.position, target.position);
        //}
    }

    // A* algorithm implementation
    void FindPath(Vector3 startPos, Vector3 targetPos) {

        Stopwatch sw = new Stopwatch();
        sw.Start();

        Node startNode = grid.NodeFromWorldPoint(startPos);

        Node targetNode = grid.NodeFromWorldPoint(targetPos);

        // create a new Heap of nodes for openset and closedset with grid maxsize
        Heap<Node> openSet = new Heap<Node>(grid.MaxSize);
        HashSet<Node> closedSet = new HashSet<Node>();

        // add starting node
        openSet.Add(startNode);

        // start loop
        while (openSet.Count > 0) {
            // get current node with lowest fcost and remove from openset
            Node currentNode = openSet.RemoveFirst();
            
            closedSet.Add(currentNode);

            if (currentNode == targetNode) {  // found the path 
                sw.Stop();
                print("path found: " + sw.ElapsedMilliseconds + " ms");

                RetracePath(startNode, targetNode);
                return; 
            }
            // loop through each neighboring node
            foreach (Node neighbor in grid.GetNeighbors(currentNode)) {
                if (!neighbor.walkable || closedSet.Contains(neighbor)) { 
                    continue;
                }
                // check if new path to neighbor is shorter or neighbor is not in open list
                int newMovementCostToNeighbor = currentNode.gCost + GetDistance(currentNode, neighbor);
                if (newMovementCostToNeighbor < neighbor.gCost || !openSet.Contains(neighbor)) {
                    // calculate g cost, h cost to find f cost
                    neighbor.gCost = newMovementCostToNeighbor;
                    neighbor.hCost = GetDistance(neighbor, targetNode);
                    // set parent of neighbor to current node
                    neighbor.parent = currentNode;
                    // check if neighbor is in open set, and add it if not
                    if(!openSet.Contains(neighbor))
                        openSet.Add(neighbor);
                
                }
            }
        }
    }
    // if current node = target node, need to retrace steps to get the path from start node to end node
    void RetracePath (Node startNode, Node endNode) {
        List<Node> path = new List<Node>();
        Node currentNode = endNode;

        while (currentNode != startNode) {
            path.Add(currentNode);
            currentNode = currentNode.parent;
        }
        path.Reverse();     // path.Reverse to get path in the right way

        //visualize path
        grid.path = path;
    }
    // first count on x axis, and y axis how many nodes away from target node
    // take lowest number (gives us how many diagonal moves it will take to be either horizontal or vertical alligned with end node)
    // subtract higher number - lower number = how many horizontal moves needed
    int GetDistance(Node nodeA, Node nodeB) {
        int dstX = Mathf.Abs(nodeA.gridX - nodeB.gridX);
        int dstY = Mathf.Abs(nodeA.gridY - nodeB.gridY);

        if (dstX > dstY)
            return 14 * dstY + 10 * (dstX-dstY);
        return 14 * dstX + 10 * (dstY-dstX);
        }
    
}

