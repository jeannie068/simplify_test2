/**
 * Contour.hpp
 * 
 * This file defines the Contour class used for efficient packing in the ASF-B*-tree
 * placement algorithm. The contour data structure represents the skyline profile
 * of the currently placed modules, allowing for O(1) height queries and updates
 * using a doubly linked list implementation.
 */

#pragma once

#include <vector>
#include <utility>
#include <limits>
#include <memory>

/**
 * Contour segment representing a horizontal or vertical segment in the placement
 */
struct ContourSegment {
    int start;   // Start coordinate (x for horizontal, y for vertical)
    int end;     // End coordinate (x for horizontal, y for vertical)
    int height;  // Height (y for horizontal, x for vertical)
    
    ContourSegment(int start, int end, int height)
        : start(start), end(end), height(height) {}
        
    bool operator<(const ContourSegment& other) const {
        return start < other.start;
    }
};

/**
 * Contour class representing the skyline profile of the placement
 * Supports efficient queries and updates in O(1) time with a doubly linked list
 */
class Contour {
private:
    // Node structure for the doubly linked list
    struct ContourNode {
        int x;       // x-coordinate
        int height;  // height at this coordinate
        ContourNode* prev;  // Previous node
        ContourNode* next;  // Next node
        
        ContourNode(int x, int height) 
            : x(x), height(height), prev(nullptr), next(nullptr) {}
    };
    
    ContourNode* head;  // First node of the contour
    ContourNode* tail;  // Last node of the contour
    
    // Helper method to find the node that contains a specific x-coordinate
    ContourNode* findNode(int x) const;
    
    // Helper method to insert a node after a specific node
    void insertNodeAfter(ContourNode* node, int x, int height);
    
    // Helper method to delete a node
    void deleteNode(ContourNode* node);
    
public:
    /**
     * Constructor
     */
    Contour();
    
    /**
     * Copy constructor
     */
    Contour(const Contour& other);
    
    /**
     * Destructor
     */
    ~Contour();
    
    /**
     * Clears the contour
     */
    void clear();
    
    /**
     * Adds a segment to the contour
     * 
     * @param start Start coordinate of the segment
     * @param end End coordinate of the segment
     * @param height Height of the segment
     */
    void addSegment(int start, int end, int height);
    
    /**
     * Gets the height of the contour at a specific range
     * 
     * @param start Start coordinate of the range
     * @param end End coordinate of the range
     * @return Maximum height in the range
     */
    int getHeight(int start, int end) const;
    
    /**
     * Gets all contour segments
     * 
     * @return Vector of contour segments
     */
    std::vector<ContourSegment> getSegments() const;
    
    /**
     * Merges this contour with another contour
     * 
     * @param other Other contour to merge with
     */
    void merge(const Contour& other);
    
    /**
     * Gets the maximum coordinate value in the contour
     * 
     * @return Maximum coordinate value
     */
    int getMaxCoordinate() const;
    
    /**
     * Gets the maximum height value in the contour
     * 
     * @return Maximum height value
     */
    int getMaxHeight() const;
    
    /**
     * Checks if the contour is empty
     * 
     * @return True if the contour is empty, false otherwise
     */
    bool isEmpty() const;
};