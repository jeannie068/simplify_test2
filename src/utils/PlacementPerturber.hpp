/**
 * PlacementPerturber.hpp
 * 
 * This class handles all perturbation operations for the placement algorithm
 * including module rotation, node movement, swapping, and other operations
 * needed for simulated annealing.
 */

#pragma once

#include <memory>
#include <map>
#include <vector>
#include <string>
#include <random>
#include "../data_struct/Module.hpp"
#include "../data_struct/ASFBStarTree.hpp"
#include "../data_struct/BStarTreeNode.hpp"
#include "../data_struct/SymmetryConstraint.hpp"
#include "../data_struct/SymmetryIslandBlock.hpp"

class PlacementPerturber {
public:
    /**
     * Rotates a module
     * 
     * @param module The module to rotate
     * @return True if rotation succeeded
     */
    static bool rotateModule(std::shared_ptr<Module> module);
    
    /**
     * Rotates a module in a symmetry group (handles symmetry pairs)
     * 
     * @param asfTree The ASF-B*-tree containing the module
     * @param moduleName Name of the module to rotate
     * @return True if rotation succeeded
     */
    static bool rotateSymmetryModule(
        std::shared_ptr<ASFBStarTree> asfTree, 
        const std::string& moduleName);
    
    /**
     * Moves a module to a new position
     * 
     * @param module The module to move
     * @param x New x-coordinate
     * @param y New y-coordinate
     * @return True if move succeeded
     */
    static bool moveModule(
        std::shared_ptr<Module> module, 
        int x, int y);
    
    /**
     * Swaps the positions of two modules
     * 
     * @param module1 First module
     * @param module2 Second module
     * @return True if swap succeeded
     */
    static bool swapModules(
        std::shared_ptr<Module> module1, 
        std::shared_ptr<Module> module2);
    
    /**
     * Swaps two nodes in a B*-tree
     * 
     * @param node1 First node
     * @param node2 Second node
     * @return True if swap succeeded
     */
    static bool swapTreeNodes(
        std::shared_ptr<BStarTreeNode> node1, 
        std::shared_ptr<BStarTreeNode> node2);
    
    /**
     * Moves a node in a B*-tree to a new parent
     * 
     * @param nodeToMove Node to move
     * @param newParent New parent node
     * @param asLeftChild True to make left child, false for right child
     * @return True if move succeeded
     */
    static bool moveNodeInTree(
        std::shared_ptr<BStarTreeNode> nodeToMove, 
        std::shared_ptr<BStarTreeNode> newParent,
        bool asLeftChild);
    
    /**
     * Changes the representative of a symmetry pair
     * 
     * @param asfTree The ASF-B*-tree
     * @param moduleName Name of a module in the symmetry pair
     * @return True if change succeeded
     */
    static bool changeRepresentative(
        std::shared_ptr<ASFBStarTree> asfTree, 
        const std::string& moduleName);
    
    /**
     * Converts symmetry type (vertical to horizontal or vice versa)
     * 
     * @param asfTree The ASF-B*-tree
     * @return True if conversion succeeded
     */
    static bool convertSymmetryType(std::shared_ptr<ASFBStarTree> asfTree);
    
    /**
     * Performs a random perturbation on a global B*-tree
     * 
     * @param globalTree The global B*-tree
     * @param globalNodes Map of node names to module or island pointers
     * @param rng Random number generator
     * @return True if perturbation succeeded
     */
    static bool perturbGlobalTree(
        std::shared_ptr<BStarTreeNode>& globalTree,
        std::map<std::string, std::variant<std::shared_ptr<Module>, 
                                          std::shared_ptr<SymmetryIslandBlock>>>& globalNodes,
        std::mt19937& rng);
    
    /**
     * Rotates a node in the global tree
     * 
     * @param nodeName Name of the node to rotate
     * @param globalNodes Map of node names to module or island pointers
     * @return True if rotation succeeded
     */
    static bool rotateGlobalNode(
        const std::string& nodeName,
        std::map<std::string, std::variant<std::shared_ptr<Module>, 
                                          std::shared_ptr<SymmetryIslandBlock>>>& globalNodes);
    
    /**
     * Finds a node in a B*-tree by name
     * 
     * @param root Root of the B*-tree
     * @param name Name of the node to find
     * @return Pointer to the node if found, nullptr otherwise
     */
    static std::shared_ptr<BStarTreeNode> findNodeInTree(
        const std::shared_ptr<BStarTreeNode>& root, 
        const std::string& name);
    
    /**
     * Collect nodes in pre-order traversal
     * 
     * @param root Root of the tree
     * @param nodes Output vector of nodes
     */
    static void collectNodesPreOrder(
        const std::shared_ptr<BStarTreeNode>& root,
        std::vector<std::shared_ptr<BStarTreeNode>>& nodes);
    
    /**
     * Collect nodes in in-order traversal
     * 
     * @param root Root of the tree
     * @param nodes Output vector of nodes
     */
    static void collectNodesInOrder(
        const std::shared_ptr<BStarTreeNode>& root,
        std::vector<std::shared_ptr<BStarTreeNode>>& nodes);
};