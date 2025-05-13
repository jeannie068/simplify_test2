/**
 * PlacementPacker.hpp
 * 
 * This class handles all packing operations for the placement algorithm
 * including ASF-B*-tree packing, global B*-tree packing, and contour-based packing.
 */

#pragma once

#include <memory>
#include <map>
#include <vector>
#include <variant>
#include <string>
#include "../data_struct/Module.hpp"
#include "../data_struct/ASFBStarTree.hpp"
#include "../data_struct/BStarTreeNode.hpp"
#include "../data_struct/SymmetryConstraint.hpp"
#include "../data_struct/SymmetryIslandBlock.hpp"
#include "Contour.hpp"

class PlacementPacker {
public:
    /**
     * Packs a single ASF-B*-tree for a symmetry group
     * 
     * @param tree The ASF-B*-tree to pack
     * @return True if packing succeeded
     */
    static bool packASFBStarTree(const std::shared_ptr<ASFBStarTree>& tree);
    
    /**
     * Packs the global B*-tree containing regular modules and symmetry islands
     * 
     * @param globalTree The global B*-tree
     * @param globalNodes Map of node names to module or island pointers
     * @return True if packing succeeded
     */
    static bool packGlobalBTree(
        std::shared_ptr<BStarTreeNode>& globalTree,
        std::map<std::string, std::variant<std::shared_ptr<Module>, 
                                          std::shared_ptr<SymmetryIslandBlock>>>& globalNodes);
    
    /**
     * Packs all modules in a compacted placement
     * 
     * @param allModules All modules in the design
     * @param symmetryTrees ASF-B*-trees for symmetry groups
     * @param symmetryIslands Symmetry islands as blocks
     * @param globalTree Global B*-tree
     * @param globalNodes Map of global node names to modules or islands
     * @return True if packing succeeded without overlaps
     */
    static bool packSolution(
        std::map<std::string, std::shared_ptr<Module>>& allModules,
        const std::map<std::string, std::shared_ptr<ASFBStarTree>>& symmetryTrees,
        std::vector<std::shared_ptr<SymmetryIslandBlock>>& symmetryIslands,
        std::shared_ptr<BStarTreeNode>& globalTree,
        std::map<std::string, std::variant<std::shared_ptr<Module>, 
                                          std::shared_ptr<SymmetryIslandBlock>>>& globalNodes);
    
    /**
     * Packs regular modules using a B*-tree and contour-based approach
     * 
     * @param regularTree B*-tree for regular modules
     * @param regularModules Map of regular module names to module pointers
     * @param horizontalContour Horizontal contour for packing
     * @param verticalContour Vertical contour for packing
     * @return True if packing succeeded
     */
    static bool packRegularModules(
        std::shared_ptr<BStarTreeNode> regularTree,
        std::map<std::string, std::shared_ptr<Module>>& regularModules,
        std::shared_ptr<Contour> horizontalContour,
        std::shared_ptr<Contour> verticalContour);
    
    /**
     * Checks if there are any overlaps between modules
     * 
     * @param allModules All modules in the design 
     * @param moduleToGroup Mapping from module name to its symmetry group
     * @return True if there are overlaps
     */
    static bool hasOverlaps(
        const std::map<std::string, std::shared_ptr<Module>>& allModules,
        const std::map<std::string, std::shared_ptr<SymmetryGroup>>& moduleToGroup);
    
    /**
     * Validates symmetry constraints
     * 
     * @param allModules All modules in the design
     * @param symmetryTrees ASF-B*-trees for symmetry groups
     * @return True if all symmetry constraints are valid
     */
    static bool validateSymmetryConstraints(
        const std::map<std::string, std::shared_ptr<Module>>& allModules,
        const std::map<std::string, std::shared_ptr<ASFBStarTree>>& symmetryTrees);
    
    /**
     * Calculates the total area of the placement
     * 
     * @param allModules All modules in the design
     * @param solutionWidth Output parameter for solution width
     * @param solutionHeight Output parameter for solution height
     * @return Total area of the placement
     */
    static int calculateTotalArea(
        const std::map<std::string, std::shared_ptr<Module>>& allModules,
        int& solutionWidth,
        int& solutionHeight);
    
    /**
     * Checks if a position is valid (no overlaps)
     * 
     * @param x X-coordinate to check
     * @param y Y-coordinate to check
     * @param width Width of the module
     * @param height Height of the module
     * @param allModules All modules to check against
     * @return True if the position is valid
     */
    static bool isPositionValid(
        int x, int y, int width, int height,
        const std::map<std::string, std::shared_ptr<Module>>& allModules);
};