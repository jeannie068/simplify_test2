// ASFBStarTree.hpp
// Automatically Symmetric-Feasible B*-tree

#pragma once

#include "BStarTreeNode.hpp"
#include "SymmetryConstraint.hpp"
#include "Module.hpp"
#include "../utils/Contour.hpp"

#include <memory>
#include <vector>
#include <map>
#include <string>
#include <utility>
using namespace std;


class ASFBStarTree {
private:
    // Core tree structure
    shared_ptr<BStarTreeNode> root;
    
    // Module and symmetry information
    map<string, shared_ptr<Module>> modules;
    shared_ptr<SymmetryGroup> symmetryGroup;
    
    // Track representatives for symmetry pairs and self-symmetric modules
    map<string, string> representativeMap;  // Maps module to its representative
    map<string, string> symmetricPairMap;   // Maps a module to its symmetric pair
    vector<string> selfSymmetricModules;         // List of self-symmetric modules
    
    // Contour structure for packing
    shared_ptr<Contour> horizontalContour;
    shared_ptr<Contour> verticalContour;
    
    // Symmetry axis position
    double symmetryAxisPosition;
    
    // Internal helper methods
    bool isOnBoundary(const string& moduleName) const;
    bool canMoveNode(const shared_ptr<BStarTreeNode>& node, 
                     const shared_ptr<BStarTreeNode>& newParent, 
                     bool asLeftChild) const;
    void updateRepresentatives();
    
    // Packing helpers
    void initializeContours();
    void updateContourWithModule(const shared_ptr<Module>& module);
    void packNode(const shared_ptr<BStarTreeNode>& node);
    void calculateSymmetricModulePositions();
    
public:
    /**
     * Constructor
     * 
     * @param symmetryGroup The symmetry group to be modeled by this ASF-B*-tree
     */
    ASFBStarTree(shared_ptr<SymmetryGroup> symmetryGroup);
    ~ASFBStarTree();
    
    void addModule(shared_ptr<Module> module); // Adds a module to the tree
    
    // Constructs an initial ASF-B*-tree based on the symmetry group
    void constructInitialTree(); 

    bool validateSymmetryConstraints() const;
    bool isOnCorrectBranch(const string& moduleName) const;
    bool isSymmetryIslandValid() const;
    
    /**
     * Calculates the coordinates of all modules in the symmetry group
     * by packing the ASF-B*-tree
     * 
     * @return True if packing was successful, false otherwise
     */
    bool pack();
    
    /**
     * Checks if the tree satisfies the symmetric-feasible condition
     * 
     * @return True if the tree is symmetric-feasible, false otherwise
     */
    bool isSymmetricFeasible() const;
    
    /**
     * Returns the bounding rectangle area of the symmetry island
     * 
     * @return Area of the symmetry island
     */
    int getArea() const;
    
    /**
     * Gets the contour of the symmetry island (for HB*-tree)
     * 
     * @return Horizontal and vertical contours of the symmetry island
     */
    pair<shared_ptr<Contour>, shared_ptr<Contour>> getContours() const;
    
    // Perturbation operations for simulated annealing
    
    /**
     * Rotates a module in the symmetry group
     * 
     * @param moduleName Name of the module to rotate
     * @return True if rotation was successful, false otherwise
     */
    bool rotateModule(const string& moduleName);
    
    /**
     * Moves a node to a new position in the tree
     * 
     * @param nodeName Name of the node to move
     * @param newParentName Name of the new parent node
     * @param asLeftChild True if the node should be the left child, false for right child
     * @return True if the move was successful, false otherwise
     */
    bool moveNode(const string& nodeName, 
                 const string& newParentName, 
                 bool asLeftChild);
    
    /**
     * Swaps two nodes in the tree
     * 
     * @param nodeName1 Name of the first node
     * @param nodeName2 Name of the second node
     * @return True if the swap was successful, false otherwise
     */
    bool swapNodes(const string& nodeName1, const string& nodeName2);
    
    /**
     * Changes the representative of a symmetry pair
     * 
     * @param pairName Name of the symmetry pair
     * @return True if the change was successful, false otherwise
     */
    bool changeRepresentative(const string& moduleName);
    
    /**
     * Converts the symmetry type (vertical to horizontal or vice versa)
     * 
     * @return True if the conversion was successful, false otherwise
     */
    bool convertSymmetryType();
    
    // Getters
    
    /**
     * Gets the root node of the ASF-B*-tree
     * 
     * @return Root node
     */
    shared_ptr<BStarTreeNode> getRoot() const;
    
    /**
     * Gets all modules in the symmetry group
     * 
     * @return Map of module names to modules
     */
    const map<string, shared_ptr<Module>>& getModules() const;
    
    /**
     * Gets the symmetry group
     * 
     * @return Symmetry group
     */
    shared_ptr<SymmetryGroup> getSymmetryGroup() const;
    
    /**
     * Gets the position of the symmetry axis
     * 
     * @return Symmetry axis position
     */
    double getSymmetryAxisPosition() const;
    
    /**
     * Checks if a module is a representative
     * 
     * @param moduleName Name of the module to check
     * @return True if the module is a representative, false otherwise
     */
    bool isRepresentative(const string& moduleName) const;
    
    /**
     * Gets the representative of a module
     * 
     * @return Name of the representative module
     */
    string getRepresentative(const string& moduleName) const;
    
    shared_ptr<ASFBStarTree> clone() const;
};