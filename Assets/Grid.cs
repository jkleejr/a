using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Grid : MonoBehaviour {

    public bool onlyDisplayPathGizmos;
    public Transform player;
    public LayerMask unwalkableMask;
    public Vector2 gridWorldSize;  // area in coordinates 
    public float nodeRadius;  // individual node space

    Node[,] grid;  // 2D array of nodes representing grid

    float nodeDiameter;
    int gridSizeX, gridSizeY;

    void Start(){
        nodeDiameter = nodeRadius * 2;
        gridSizeX = Mathf.RoundToInt(gridWorldSize.x/nodeDiameter);  // gives us how many nodes we can fit in worldSize x
        gridSizeY = Mathf.RoundToInt(gridWorldSize.y/nodeDiameter);
        CreateGrid();

    }

    public int MaxSize {
        get {
            return gridSizeX * gridSizeY;
        }
    }

    void CreateGrid() {
        grid = new Node[gridSizeX, gridSizeY];
        // bottom left corner of world
        Vector3 worldBottomLeft = transform.position - Vector3.right * gridWorldSize.x/2 - Vector3.forward * gridWorldSize.y/2;

        // loop through node positions, do collision checks to see if walkable
        for (int x = 0; x < gridSizeX; x ++) {
            for (int y = 0; y < gridSizeY; y ++){
                // get world position
                Vector3 worldPoint = worldBottomLeft + Vector3.right * (x * nodeDiameter + nodeRadius) + Vector3.forward * (y * nodeDiameter + nodeRadius);
                // collision check for each point
                bool walkable = !(Physics.CheckSphere(worldPoint, nodeRadius, unwalkableMask));
                // populate grid w Nodes
                grid[x,y] = new Node(walkable, worldPoint, x, y);
            }
        }
    }

    public List<Node> GetNeighbors(Node node) {
        List<Node> neighbors = new List<Node>();

        // loop searches in 3x3 block around the node
        for (int x = -1; x <= 1; x ++){
            for (int y = -1; y <= 1; y ++) {
                if (x == 0 && y == 0)
                    continue;

                int checkX = node.gridX + x;
                int checkY = node.gridY + y;
                // check if inside of the grid
                if (checkX >= 0 && checkX < gridSizeX && checkY >= 0 && checkY < gridSizeY) {
                    neighbors.Add(grid[checkX, checkY]);
                }
            }
        }
        return neighbors;
    }

    // method finds the current node of object
    public Node NodeFromWorldPoint(Vector3 worldPosition){
        // converts world position into percentage of how far along it is
        float percentX = (worldPosition.x + gridWorldSize.x/2) / gridWorldSize.x;
        float percentY = (worldPosition.z + gridWorldSize.y/2) / gridWorldSize.y;
        // if object node is outside of grid, makes sure no invalid index in array by clamp
        percentX = Mathf.Clamp01(percentX);  
        percentY = Mathf.Clamp01(percentY);
        // x,y indices of 2D grid array
        int x = Mathf.RoundToInt((gridSizeX-1) * percentX);  // -1 so not outside of array
        int y = Mathf.RoundToInt((gridSizeY-1) * percentY); 
        // return node from grid
        return grid [x,y];
    }

    public List<Node> path;

    void OnDrawGizmos(){   // visualize grid
        Gizmos.DrawWireCube(transform.position, new Vector3(gridWorldSize.x,1,gridWorldSize.y));

        if (onlyDisplayPathGizmos) {
            if (path != null) {
                foreach (Node n in path) {
                    Gizmos.color = Color.blue;
                    Gizmos.DrawCube(n.worldPosition, Vector3.one * (nodeDiameter-.1f));
                }
            }
        }
        else{
            if (grid != null) {
                Node playerNode = NodeFromWorldPoint(player.position);
                foreach (Node n in grid) {
                    
                    Gizmos.color = (n.walkable)?Color.white:Color.red;

                    if (path != null)
                        if (path.Contains(n))   // if path contains current node n 
                            Gizmos.color = Color.blue;
                        Gizmos.DrawCube(n.worldPosition, Vector3.one * (nodeDiameter-.1f));
                }
            }
        }
    }

}

