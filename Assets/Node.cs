using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Node : IHeapItem<Node>  // implement IHeapItem interface type Node
{
    // 2 states, walkable or not
    public bool walkable; 
    public Vector3 worldPosition;
    public int gridX;
    public int gridY; 

    public int gCost;
    public int hCost;

    public Node parent;

    int heapIndex;

    public Node(bool _walkable, Vector3 _worldPos, int _gridX, int _gridY) {
        walkable = _walkable;
        worldPosition = _worldPos;
        gridX = _gridX;
        gridY = _gridY;
    }
    // calculates fCost
    public int fCost {
        get {
        return gCost + hCost;
        }
    }

    // implement IHeapItem interface
    public int HeapIndex {
        get{
            return heapIndex;
        }
        set{
            heapIndex = value;
        }
    }

    public int CompareTo(Node nodeToCompare) {
        int compare = fCost.CompareTo(nodeToCompare.fCost);
        if (compare == 0) {   // if the fcosts are equal
            compare = hCost.CompareTo(nodeToCompare.hCost);
        // returns 1 if current node has higher priority than compared node
        }
        // reversed for our nodes, want to return 1 if it is lower
        return -compare;
    }



}


