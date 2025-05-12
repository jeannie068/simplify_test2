/**
 * Contour.cpp
 * 
 * Implementation of the Contour class for efficient packing in the ASF-B*-tree
 * placement algorithm. The contour data structure represents the skyline profile
 * of the currently placed modules, allowing for O(1) height queries and updates
 * using a doubly linked list implementation.
 */

#include "Contour.hpp"
#include <algorithm>
#include <cassert>

/**
 * Constructor
 */
Contour::Contour() : head(nullptr), tail(nullptr) {
    // Initialize with empty contour
}

/**
 * Copy constructor
 */
Contour::Contour(const Contour& other) : head(nullptr), tail(nullptr) {
    // Copy the contour segments from the other contour
    if (other.head) {
        // Create the first node
        head = new ContourNode(other.head->x, other.head->height);
        tail = head;
        
        // Copy the rest of the nodes
        ContourNode* current = other.head->next;
        while (current) {
            ContourNode* newNode = new ContourNode(current->x, current->height);
            newNode->prev = tail;
            tail->next = newNode;
            tail = newNode;
            current = current->next;
        }
    }
}

/**
 * Destructor
 */
Contour::~Contour() {
    clear();
}

/**
 * Clears the contour
 */
void Contour::clear() {
    ContourNode* current = head;
    while (current) {
        ContourNode* next = current->next;
        delete current;
        current = next;
    }
    head = nullptr;
    tail = nullptr;
}

/**
 * Helper method to find the node that contains a specific x-coordinate
 */
Contour::ContourNode* Contour::findNode(int x) const {
    if (!head) return nullptr;
    
    ContourNode* current = head;
    // Find the rightmost node whose x-coordinate is <= x
    while (current->next && current->next->x <= x) {
        current = current->next;
    }
    return current;
}

/**
 * Helper method to insert a node after a specific node
 */
void Contour::insertNodeAfter(ContourNode* node, int x, int height) {
    ContourNode* newNode = new ContourNode(x, height);
    
    if (!node) {
        // Insert at the beginning
        newNode->next = head;
        if (head) {
            head->prev = newNode;
        } else {
            tail = newNode;
        }
        head = newNode;
    } else {
        // Insert after the given node
        newNode->next = node->next;
        newNode->prev = node;
        
        if (node->next) {
            node->next->prev = newNode;
        } else {
            tail = newNode;
        }
        
        node->next = newNode;
    }
}

/**
 * Helper method to delete a node
 */
void Contour::deleteNode(ContourNode* node) {
    if (!node) return;
    
    if (node->prev) {
        node->prev->next = node->next;
    } else {
        // This is the head node
        head = node->next;
    }
    
    if (node->next) {
        node->next->prev = node->prev;
    } else {
        // This is the tail node
        tail = node->prev;
    }
    
    delete node;
}

/**
 * Adds a segment to the contour
 * 
 * This function updates the contour to incorporate a new module or segment.
 * It ensures the contour remains a valid skyline by merging overlapping segments.
 * 
 * Time complexity: O(k) where k is the number of contour segments affected.
 */
void Contour::addSegment(int start, int end, int height) {
    if (start >= end) return;  // Invalid segment
    
    // Special case: empty contour
    if (!head) {
        // Create two nodes for start and end
        head = new ContourNode(start, height);
        tail = new ContourNode(end, 0);
        head->next = tail;
        tail->prev = head;
        return;
    }
    
    // Find the first node whose x-coordinate is >= start
    ContourNode* current = head;
    while (current && current->x < start) {
        current = current->next;
    }
    
    // If we need to insert a new start point
    if (!current || current->x > start) {
        ContourNode* prevNode = current ? current->prev : tail;
        int prevHeight = prevNode ? prevNode->height : 0;
        
        // Insert the new start point
        insertNodeAfter(prevNode, start, height);
    } else if (current->x == start) {
        // Update the height of the existing node
        current->height = std::max(current->height, height);
    }
    
    // Process nodes between start and end
    current = head;
    while (current && current->x < end) {
        ContourNode* next = current->next;
        
        // If this node is within our segment range and after start
        if (current->x > start && current->x < end) {
            if (current->height <= height) {
                // If current height is not higher than our new segment, remove it
                deleteNode(current);
            }
        }
        
        current = next;
    }
    
    // If we need to insert a new end point
    if (!current || current->x > end) {
        ContourNode* prevNode = current ? current->prev : tail;
        
        // Find the height at the end point
        int endHeight = 0;
        if (current) {
            // There's a node after the end point
            endHeight = current->height;
        }
        
        // Insert the new end point
        insertNodeAfter(prevNode, end, endHeight);
    }
    
    // Cleanup: merge adjacent nodes with the same height
    current = head;
    while (current && current->next) {
        if (current->height == current->next->height) {
            deleteNode(current->next);
        } else {
            current = current->next;
        }
    }
}

/**
 * Gets the height of the contour at a specific range
 * 
 * This function returns the maximum height of the contour within
 * the specified coordinate range.
 * 
 * Time complexity: O(k) where k is the number of segments in the range
 */
int Contour::getHeight(int start, int end) const {
    if (start >= end) return 0;  // Invalid range
    if (!head) return 0;  // Empty contour
    
    // Find the first node that might contribute to the range
    ContourNode* current = findNode(start);
    if (!current) {
        // No nodes <= start, so return 0 or the height of the first node
        return head ? head->height : 0;
    }
    
    // If the found node is at a position < start, we need to use its height for the start position
    int maxHeight = current->height;
    
    // Traverse nodes in the range
    while (current && current->x < end) {
        maxHeight = std::max(maxHeight, current->height);
        current = current->next;
    }
    
    return maxHeight;
}

/**
 * Gets all contour segments
 * 
 * This function converts the internal representation to a vector
 * of ContourSegment objects for easier processing.
 * 
 * Time complexity: O(n) where n is the number of segments
 */
std::vector<ContourSegment> Contour::getSegments() const {
    std::vector<ContourSegment> segments;
    
    if (!head || !head->next) {
        return segments;
    }
    
    // Convert doubly linked list representation to segments
    ContourNode* current = head;
    while (current && current->next) {
        segments.emplace_back(current->x, current->next->x, current->height);
        current = current->next;
    }
    
    return segments;
}

/**
 * Merges this contour with another contour
 * 
 * This function combines two contours by taking the maximum height
 * at each coordinate.
 * 
 * Time complexity: O(n + m) where n and m are the sizes of the two contours
 */
void Contour::merge(const Contour& other) {
    if (other.isEmpty()) return;
    if (this->isEmpty()) {
        *this = other;
        return;
    }
    
    // Get all breakpoints from both contours
    std::vector<int> breakpoints;
    
    // Add breakpoints from this contour
    ContourNode* current = head;
    while (current) {
        breakpoints.push_back(current->x);
        current = current->next;
    }
    
    // Add breakpoints from the other contour
    current = other.head;
    while (current) {
        breakpoints.push_back(current->x);
        current = current->next;
    }
    
    // Sort and remove duplicates
    std::sort(breakpoints.begin(), breakpoints.end());
    breakpoints.erase(std::unique(breakpoints.begin(), breakpoints.end()), breakpoints.end());
    
    // Create a new contour with the merged heights
    Contour newContour;
    
    // For each breakpoint, take the maximum height from both contours
    for (size_t i = 0; i < breakpoints.size() - 1; ++i) {
        int start = breakpoints[i];
        int end = breakpoints[i + 1];
        int height1 = getHeight(start, start + 1);
        int height2 = other.getHeight(start, start + 1);
        int maxHeight = std::max(height1, height2);
        
        newContour.addSegment(start, end, maxHeight);
    }
    
    // If there's a last breakpoint, make sure it's included
    if (!breakpoints.empty()) {
        int lastPoint = breakpoints.back();
        newContour.addSegment(lastPoint, lastPoint + 1, 0);
    }
    
    // Replace this contour with the new one
    *this = newContour;
}

/**
 * Gets the maximum coordinate value in the contour
 */
int Contour::getMaxCoordinate() const {
    if (!tail) return 0;
    return tail->x;
}

/**
 * Gets the maximum height value in the contour
 */
int Contour::getMaxHeight() const {
    int maxHeight = 0;
    
    ContourNode* current = head;
    while (current) {
        maxHeight = std::max(maxHeight, current->height);
        current = current->next;
    }
    
    return maxHeight;
}

/**
 * Checks if the contour is empty
 */
bool Contour::isEmpty() const {
    return head == nullptr;
}