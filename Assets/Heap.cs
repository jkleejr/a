using System.Collections;
using System.Collections.Generic;
using System;       // for interface IComparable
using UnityEngine;

public class Heap<T> where T : IHeapItem<T>
{
    T[] items;      // specify array type T instead of array of nodes
    int currentItemCount;

    public Heap(int maxHeapSize) {
        items = new T[maxHeapSize];

    }

    public void Add(T item) {       // addingnew items to heap
        item.HeapIndex = currentItemCount;
        items[currentItemCount] = item;  // add to end of items array
        SortUp(item);
        currentItemCount++;
    }

    // removing 1st item from heap
    public T RemoveFirst() {
        T firstItem = items[0];  // save the first item
        currentItemCount--; 
        items[0] = items[currentItemCount];   // take item at end of heap and move it to 1st place
        items[0].HeapIndex = 0;  
        SortDown(items[0]);
        return firstItem;
    }
    // change priority/fcost of a node, update position in heap
    public void UpdateItem(T item) {
        SortUp(item);
    }

    public int Count {
        get {
            return currentItemCount;
        }
    }
    
    // check if heap contains specific node
    public bool Contains(T item) {
        return Equals(items[item.HeapIndex], item);
    }

    // sorting
    void SortDown (T item) {
        while (true) {
            int childIndexLeft = item.HeapIndex * 2 + 1;
            int childIndexRight = item.HeapIndex * 2 + 2;
            int swapIndex = 0;

            // check if item has child on left
            if (childIndexLeft < currentItemCount) {
                swapIndex = childIndexLeft;
            // check if child index right
                if (childIndexRight < currentItemCount) {
                    if (items[childIndexLeft].CompareTo(items[childIndexRight]) < 0) {
                    swapIndex = childIndexRight;
                    }
                }
                // now swap index is = child highest with highest priority 
                // check if parent has lower priority than highest priority child, if < 0  swap
                if (item.CompareTo(items[swapIndex]) < 0) {
                    Swap (item, items[swapIndex]);
                }
                else {
                    return;
                }
            }
            // if no children 
            else {
                return;
            }
        }
    }

    void SortUp(T item) {
        int parentIndex = (item.HeapIndex-1)/2;   // get parent index

        while (true) {
            T parentItem = items[parentIndex];
            if (item.CompareTo(parentItem) > 0) {   // implement CompareTo
            // if higher priority returns 1, same priority returns 0, lower priority returns -1
                Swap (item, parentItem);  // if item higher priority than parent item, swap
            }
            else {
                break;
            }
            // otherwise keep calculating parentIndex and comparing the item to its new parent
            parentIndex = (item.HeapIndex-1)/2;
            }
        }

        void Swap(T itemA, T itemB) {
            items[itemA.HeapIndex] = itemB;
            items[itemB.HeapIndex] = itemA;

            int itemAIndex = itemA.HeapIndex;
            itemA.HeapIndex = itemB.HeapIndex;
            itemB.HeapIndex = itemAIndex;
    }
}
    // each item to keep track of index in heap
    // compare 2 items for higher priority to sort
public interface IHeapItem<T> : IComparable<T> {
    int HeapIndex {
        get;
        set;
    }

}

