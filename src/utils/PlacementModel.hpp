/**
 * PlacementModel.hpp
 * 
 * This class encapsulates the entire placement model including all modules,
 * symmetry groups, ASF-B*-trees, and global B*-tree. It serves as the solution
 * representation for simulated annealing.
 */

#pragma once

#include <memory>
#include <map>
#include <vector>
#include <string>
#include <variant>
#include "../data_struct/Module.hpp"
#include "../data_struct/ASFBStarTree.hpp"
#include "../data_struct/BStarTreeNode.hpp"
#include "../data_struct/SymmetryConstraint.hpp"
#include "../data_struct/SymmetryIslandBlock.hpp"

/**
 * Encapsulates the entire placement state, serving as the solution type for SA
 */
class PlacementModel {
public:
    // Regular modules (not part of symmetry groups)
    std::map<std::string, std::shared_ptr<Module>> regularModules;
    
    // All modules (references to both regular and symmetry modules)
    std::map<std::string, std::shared_ptr<Module>> allModules;
    
    // Symmetry groups and their trees
    std::vector<std::shared_ptr<SymmetryGroup>> symmetryGroups;
    std::map<std::string, std::shared_ptr<ASFBStarTree>> symmetryTrees;
    
    // Symmetry islands as blocks for global placement
    std::vector<std::shared_ptr<SymmetryIslandBlock>> symmetryIslands;
    
    // Global B*-tree for overall placement
    std::shared_ptr<BStarTreeNode> globalTree;
    
    // Map from node name to either Module or SymmetryIslandBlock
    std::map<std::string, std::variant<std::shared_ptr<Module>, 
        std::shared_ptr<SymmetryIslandBlock>>> globalNodes;
    
    // B*-tree for regular modules
    std::shared_ptr<BStarTreeNode> regularTree;
    
    // Mapping from module name to its parent symmetry group
    std::map<std::string, std::shared_ptr<SymmetryGroup>> moduleToGroup;
    
    // Statistics
    int totalArea;
    int solutionWidth;
    int solutionHeight;
    
    /**
     * Default constructor
     */
    PlacementModel() : totalArea(0), solutionWidth(0), solutionHeight(0) {}
    
    /**
     * Constructor
     * 
     * @param allModules All modules
     * @param symmetryGroups All symmetry groups
     */
    PlacementModel(
        const std::map<std::string, std::shared_ptr<Module>>& allModules,
        const std::vector<std::shared_ptr<SymmetryGroup>>& symmetryGroups)
        : allModules(allModules),
          symmetryGroups(symmetryGroups),
          totalArea(0),
          solutionWidth(0),
          solutionHeight(0) {}
    
    /**
     * Creates a deep copy of the placement model
     * 
     * @return A new placement model that is a deep copy of this one
     */
    PlacementModel clone() const {
        PlacementModel copy;
        
        // Clone all modules
        for (const auto& pair : allModules) {
            copy.allModules[pair.first] = std::make_shared<Module>(*pair.second);
        }
        
        // Clone regular modules
        for (const auto& pair : regularModules) {
            copy.regularModules[pair.first] = copy.allModules[pair.first];
        }
        
        // Copy symmetry groups (no need to clone - they're immutable)
        copy.symmetryGroups = symmetryGroups;
        
        // Clone symmetry trees
        for (const auto& pair : symmetryTrees) {
            copy.symmetryTrees[pair.first] = pair.second->clone();
            
            // Update module references in the cloned tree
            for (const auto& modPair : pair.second->getModules()) {
                copy.symmetryTrees[pair.first]->addModule(copy.allModules[modPair.first]);
            }
        }
        
        // Clone symmetry islands
        for (const auto& island : symmetryIslands) {
            auto clonedTree = copy.symmetryTrees[island->getName()];
            auto clonedIsland = std::make_shared<SymmetryIslandBlock>(island->getName(), clonedTree);
            clonedIsland->setPosition(island->getX(), island->getY());
            copy.symmetryIslands.push_back(clonedIsland);
        }
        
        // Clone global nodes
        for (const auto& pair : globalNodes) {
            std::visit([&](auto&& arg) {
                using T = std::decay_t<decltype(arg)>;
                if constexpr (std::is_same_v<T, std::shared_ptr<Module>>) {
                    copy.globalNodes[pair.first] = copy.allModules[arg->getName()];
                } else if constexpr (std::is_same_v<T, std::shared_ptr<SymmetryIslandBlock>>) {
                    // Find corresponding island in copy
                    for (const auto& island : copy.symmetryIslands) {
                        if (island->getName() == arg->getName()) {
                            copy.globalNodes[pair.first] = island;
                            break;
                        }
                    }
                }
            }, pair.second);
        }
        
        // Clone B*-trees - this is a simplification, a full implementation 
        // would need to clone the tree structures
        // For now, we'll just copy the pointers
        copy.globalTree = globalTree;
        copy.regularTree = regularTree;
        
        // Copy module-to-group mapping
        copy.moduleToGroup = moduleToGroup;
        
        // Copy statistics
        copy.totalArea = totalArea;
        copy.solutionWidth = solutionWidth;
        copy.solutionHeight = solutionHeight;
        
        return copy;
    }
};