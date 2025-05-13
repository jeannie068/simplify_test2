/**
 * PlacementPacker.cpp
 * 
 * Implementation of the PlacementPacker class that handles all packing operations
 * for the placement algorithm.
 */

#include "PlacementPacker.hpp"
#include <iostream>
#include <limits>
#include <queue>
#include <algorithm>

bool PlacementPacker::packASFBStarTree(const std::shared_ptr<ASFBStarTree>& tree) {
    if (!tree) return false;
    
    // Pack the ASF-B*-tree to get the internal layout
    return tree->pack();
}

bool PlacementPacker::packGlobalBTree(
    std::shared_ptr<BStarTreeNode>& globalTree,
    std::map<std::string, std::variant<std::shared_ptr<Module>, 
                                      std::shared_ptr<SymmetryIslandBlock>>>& globalNodes) {
    
    if (!globalTree) {
        return true; // Nothing to pack
    }
    
    // Create contours for packing - now using BOTH horizontal and vertical contours
    std::shared_ptr<Contour> horizontalContour = std::make_shared<Contour>();
    std::shared_ptr<Contour> verticalContour = std::make_shared<Contour>();
    
    // Initialize contours with segments at 0
    horizontalContour->addSegment(0, std::numeric_limits<int>::max(), 0);
    verticalContour->addSegment(0, std::numeric_limits<int>::max(), 0);
    
    // Pack using pre-order DFS traversal instead of BFS
    return packTreeNodeDFS(globalTree, globalNodes, horizontalContour, verticalContour, 0, 0);
}

bool PlacementPacker::packSolution(
    std::map<std::string, std::shared_ptr<Module>>& allModules,
    const std::map<std::string, std::shared_ptr<ASFBStarTree>>& symmetryTrees,
    std::vector<std::shared_ptr<SymmetryIslandBlock>>& symmetryIslands,
    std::shared_ptr<BStarTreeNode>& globalTree,
    std::map<std::string, std::variant<std::shared_ptr<Module>, 
                                      std::shared_ptr<SymmetryIslandBlock>>>& globalNodes) {
    
    // Reset all module positions
    for (auto& pair : allModules) {
        pair.second->setPosition(0, 0);
    }
    
    // First, pack all symmetry islands
    for (auto& tree : symmetryTrees) {
        if (!packASFBStarTree(tree.second)) {
            return false;
        }
    }
    
    // Update symmetry island blocks
    for (auto& island : symmetryIslands) {
        island->updateBoundingBox();
    }
    
    // Then, pack the global B*-tree
    if (!packGlobalBTree(globalTree, globalNodes)) {
        return false;
    }
    
    return true;
}

bool PlacementPacker::packRegularModules(
    std::shared_ptr<BStarTreeNode> regularTree,
    std::map<std::string, std::shared_ptr<Module>>& regularModules,
    std::shared_ptr<Contour> horizontalContour,
    std::shared_ptr<Contour> verticalContour) {
    
    if (!regularTree) return true;
    
    // Process modules in order (BFS traversal of the B*-tree)
    std::queue<std::shared_ptr<BStarTreeNode>> nodeQueue;
    nodeQueue.push(regularTree);
    
    while (!nodeQueue.empty()) {
        auto currentNode = nodeQueue.front();
        nodeQueue.pop();
        
        const std::string& moduleName = currentNode->getModuleName();
        auto moduleIt = regularModules.find(moduleName);
        if (moduleIt == regularModules.end()) {
            continue;
        }
        
        auto module = moduleIt->second;
        
        int x = 0, y = 0;
        
        // Calculate position based on B*-tree rules and contour
        if (currentNode->getParent()) {
            auto parentName = currentNode->getParent()->getModuleName();
            auto parentIt = regularModules.find(parentName);
            if (parentIt != regularModules.end()) {
                auto parentModule = parentIt->second;
                
                if (currentNode->isLeftChild()) {
                    // Left child: place to the right of parent
                    x = parentModule->getX() + parentModule->getWidth();
                    // Get minimum y from the contour at this x position
                    y = horizontalContour->getHeight(x, x + module->getWidth());
                } else {
                    // Right child: place above parent
                    x = parentModule->getX();
                    // Get minimum y from the contour (must be at least above parent)
                    y = std::max(parentModule->getY() + parentModule->getHeight(),
                                horizontalContour->getHeight(x, x + module->getWidth()));
                }
            }
        } else {
            // For root node, get minimum y from the contour
            y = horizontalContour->getHeight(0, module->getWidth());
        }
        
        // Set the module's position
        module->setPosition(x, y);
        
        // Update the contour with this module
        horizontalContour->addSegment(x, x + module->getWidth(), y + module->getHeight());
        verticalContour->addSegment(y, y + module->getHeight(), x + module->getWidth());
        
        // Add children to the queue
        if (currentNode->getLeftChild()) {
            nodeQueue.push(currentNode->getLeftChild());
        }
        if (currentNode->getRightChild()) {
            nodeQueue.push(currentNode->getRightChild());
        }
    }
    
    return true;
}

bool PlacementPacker::packTreeNodeDFS(
    std::shared_ptr<BStarTreeNode> node,
    std::map<std::string, std::variant<std::shared_ptr<Module>, 
                                      std::shared_ptr<SymmetryIslandBlock>>>& globalNodes,
    std::shared_ptr<Contour> horizontalContour,
    std::shared_ptr<Contour> verticalContour,
    int parentX,
    int parentY) {

    if (!node) return true;

    const std::string& nodeName = node->getModuleName();
    auto it = globalNodes.find(nodeName);
    if (it == globalNodes.end()) {
        // Skip if node not found in global nodes
        return true;
    }

    int width = 0, height = 0;
    int x = parentX, y = parentY;

    // Get dimensions of the current node
    std::visit([&](auto&& arg) {
        using T = std::decay_t<decltype(arg)>;
        if constexpr (std::is_same_v<T, std::shared_ptr<Module>>) {
            width = arg->getWidth();
            height = arg->getHeight();
        } else if constexpr (std::is_same_v<T, std::shared_ptr<SymmetryIslandBlock>>) {
            width = arg->getWidth();
            height = arg->getHeight();
        }
    }, it->second);

    // Determine initial position based on B*-tree rules
    if (node->getParent()) {
        if (node->isLeftChild()) {
            // Left child: place to the right of parent
            x = parentX;
        } else {
            // Right child: place above parent with the same x-coordinate
            x = parentX;
            y = parentY;
        }
    } else {
        // Root node - place at origin
        x = 0;
        y = 0;
    }

    // CRITICAL FIX: Check BOTH contours to determine valid position
    // First check horizontal contour for vertical clearance
    y = std::max(y, horizontalContour->getHeight(x, x + width));
    
    // Then check vertical contour for horizontal clearance
    int xFromVerticalContour = verticalContour->getHeight(y, y + height);
    if (x < xFromVerticalContour) {
        // Need to move module to the right to avoid horizontal overlap
        x = xFromVerticalContour;
        
        // After moving horizontally, recheck vertical clearance at new x-position
        y = std::max(y, horizontalContour->getHeight(x, x + width));
    }

    // Set position
    std::visit([x, y](auto&& arg) {
        using T = std::decay_t<decltype(arg)>;
        if constexpr (std::is_same_v<T, std::shared_ptr<Module>>) {
            arg->setPosition(x, y);
        } else if constexpr (std::is_same_v<T, std::shared_ptr<SymmetryIslandBlock>>) {
            arg->setPosition(x, y);
        }
    }, it->second);

    // Update both contours after placing the module
    horizontalContour->addSegment(x, x + width, y + height);
    verticalContour->addSegment(y, y + height, x + width);

    // Pre-order DFS: Process this node, then left child, then right child
    // For left child: new parent x = current x + width (to the right)
    if (node->getLeftChild()) {
        packTreeNodeDFS(node->getLeftChild(), globalNodes, horizontalContour, verticalContour, 
                        x + width, y);
    }
    
    // For right child: new parent y = current y + height (above)
    if (node->getRightChild()) {
        packTreeNodeDFS(node->getRightChild(), globalNodes, horizontalContour, verticalContour, 
                        x, y + height);
    }

    return true;
}

void PlacementPacker::findAllOverlappingPairs(
    const std::map<std::string, std::shared_ptr<Module>>& allModules,
    const std::map<std::string, std::shared_ptr<SymmetryGroup>>& moduleToGroup,
    std::vector<std::pair<std::string, std::string>>& overlappingPairs) {
    
    overlappingPairs.clear();
    
    // 1. Create a map from module name to its parent symmetry island
    std::map<std::string, std::shared_ptr<SymmetryGroup>> moduleToIsland;
    for (const auto& pair : moduleToGroup) {
        moduleToIsland[pair.first] = pair.second;
    }
    
    // 2. Collect regular modules (not in any symmetry group)
    std::vector<std::string> regularModuleNames;
    for (const auto& pair : allModules) {
        // Skip modules that are part of a symmetry group
        if (moduleToGroup.find(pair.first) == moduleToGroup.end()) {
            regularModuleNames.push_back(pair.first);
        }
    }
    
    // 3. Check for overlaps between regular modules
    for (size_t i = 0; i < regularModuleNames.size(); ++i) {
        const auto& name1 = regularModuleNames[i];
        auto module1 = allModules.find(name1)->second;
        
        for (size_t j = i + 1; j < regularModuleNames.size(); ++j) {
            const auto& name2 = regularModuleNames[j];
            auto module2 = allModules.find(name2)->second;
            
            if (module1->overlaps(*module2)) {
                overlappingPairs.push_back({name1, name2});
            }
        }
    }
    
    // 4. Find overlaps between regular modules and symmetry island modules
    for (const auto& regularName : regularModuleNames) {
        auto regularModule = allModules.find(regularName)->second;
        
        for (const auto& modulePair : allModules) {
            // Skip if this is a regular module
            if (moduleToGroup.find(modulePair.first) == moduleToGroup.end()) {
                continue;
            }
            
            // This is a module in a symmetry group
            auto symmetryModule = modulePair.second;
            
            if (regularModule->overlaps(*symmetryModule)) {
                overlappingPairs.push_back({regularName, modulePair.first});
            }
        }
    }
    
    // Sort by frequency (how many overlaps each module has)
    std::map<std::string, int> overlapCount;
    for (const auto& pair : overlappingPairs) {
        overlapCount[pair.first]++;
        overlapCount[pair.second]++;
    }
    
    // Sort the overlapping pairs by overlap count (most problematic first)
    std::sort(overlappingPairs.begin(), overlappingPairs.end(),
              [&overlapCount](const auto& a, const auto& b) {
                  int countA = overlapCount[a.first] + overlapCount[a.second];
                  int countB = overlapCount[b.first] + overlapCount[b.second];
                  return countA > countB;
              });
}

bool PlacementPacker::iterativeImprovementPacking(
    std::map<std::string, std::shared_ptr<Module>>& allModules,
    const std::map<std::string, std::shared_ptr<SymmetryGroup>>& moduleToGroup,
    int maxIterations) {
    
    std::cout << "Starting iterative improvement packing..." << std::endl;
    
    // Find initial overlapping pairs
    std::vector<std::pair<std::string, std::string>> overlappingPairs;
    findAllOverlappingPairs(allModules, moduleToGroup, overlappingPairs);
    
    if (overlappingPairs.empty()) {
        std::cout << "No overlapping pairs found, placement is valid" << std::endl;
        return true;
    }
    
    std::cout << "Found " << overlappingPairs.size() << " initial overlapping pairs" << std::endl;
    
    // Perform iterative improvement
    for (int iteration = 0; iteration < maxIterations; ++iteration) {
        // Check if we've resolved all overlaps
        if (overlappingPairs.empty()) {
            std::cout << "All overlaps resolved after " << iteration << " iterations" << std::endl;
            return true;
        }
        
        // Process the most problematic overlap first
        const auto& [module1Name, module2Name] = overlappingPairs[0];
        
        // Check if modules exist in allModules
        auto it1 = allModules.find(module1Name);
        auto it2 = allModules.find(module2Name);
        
        if (it1 == allModules.end() || it2 == allModules.end()) {
            // Skip this pair if modules not found
            overlappingPairs.erase(overlappingPairs.begin());
            continue;
        }
        
        auto module1 = it1->second;
        auto module2 = it2->second;
        
        std::cout << "Iteration " << iteration << ": Resolving overlap between " 
                 << module1Name << " and " << module2Name << std::endl;
        
        // Determine which module to move (prefer regular modules)
        bool module1IsRegular = moduleToGroup.find(module1Name) == moduleToGroup.end();
        bool module2IsRegular = moduleToGroup.find(module2Name) == moduleToGroup.end();
        
        std::shared_ptr<Module> moduleToMove;
        std::string moduleToMoveName;
        
        if (module1IsRegular) {
            moduleToMove = module1;
            moduleToMoveName = module1Name;
        } else if (module2IsRegular) {
            moduleToMove = module2;
            moduleToMoveName = module2Name;
        } else {
            // Can't move symmetry modules directly
            // Skip this pair
            overlappingPairs.erase(overlappingPairs.begin());
            continue;
        }
        
        // Get the other module
        std::shared_ptr<Module> otherModule = (moduleToMove == module1) ? module2 : module1;
        
        // Store original position
        int origX = moduleToMove->getX();
        int origY = moduleToMove->getY();
        
        // Try several positions to resolve this overlap
        bool resolved = false;
        
        // 1. Try placing to the right
        int newX = otherModule->getX() + otherModule->getWidth() + 10;
        int newY = moduleToMove->getY();
        moduleToMove->setPosition(newX, newY);
        
        if (!moduleToMove->overlaps(*otherModule)) {
            std::cout << "  Resolved by moving right" << std::endl;
            resolved = true;
        } else {
            // 2. Try placing below
            newX = moduleToMove->getX();
            newY = otherModule->getY() + otherModule->getHeight() + 10;
            moduleToMove->setPosition(newX, newY);
            
            if (!moduleToMove->overlaps(*otherModule)) {
                std::cout << "  Resolved by moving down" << std::endl;
                resolved = true;
            } else {
                // 3. Try placing diagonally
                newX = otherModule->getX() + otherModule->getWidth() + 10;
                newY = otherModule->getY() + otherModule->getHeight() + 10;
                moduleToMove->setPosition(newX, newY);
                
                if (!moduleToMove->overlaps(*otherModule)) {
                    std::cout << "  Resolved by moving diagonally" << std::endl;
                    resolved = true;
                } else {
                    // 4. More aggressive move - place far away
                    // Find max coordinates of all modules
                    int maxX = 0, maxY = 0;
                    for (const auto& pair : allModules) {
                        auto mod = pair.second;
                        maxX = std::max(maxX, mod->getX() + mod->getWidth());
                        maxY = std::max(maxY, mod->getY() + mod->getHeight());
                    }
                    
                    newX = maxX + 100;
                    newY = 0;
                    moduleToMove->setPosition(newX, newY);
                    
                    std::cout << "  Placed module far away at (" << newX << ", " << newY << ")" << std::endl;
                    resolved = true;
                }
            }
        }
        
        if (!resolved) {
            // Restore original position if we couldn't resolve
            moduleToMove->setPosition(origX, origY);
            std::cout << "  Failed to resolve overlap" << std::endl;
        }
        
        // Remove this pair from the list regardless
        overlappingPairs.erase(overlappingPairs.begin());
        
        // Find new overlaps after this change
        findAllOverlappingPairs(allModules, moduleToGroup, overlappingPairs);
        std::cout << "  Remaining overlaps: " << overlappingPairs.size() << std::endl;
    }
    
    std::cout << "Maximum iterations reached, " << overlappingPairs.size() 
             << " overlaps remain" << std::endl;
    return overlappingPairs.empty();
}

bool PlacementPacker::hasOverlaps(
    const std::map<std::string, std::shared_ptr<Module>>& allModules,
    const std::map<std::string, std::shared_ptr<SymmetryGroup>>& moduleToGroup) {
    
    // Check for overlaps between all pairs of modules
    std::vector<std::shared_ptr<Module>> moduleList;
    
    // Collect all modules
    for (const auto& pair : allModules) {
        moduleList.push_back(pair.second);
    }
    
    // Check all pairs
    for (size_t i = 0; i < moduleList.size(); ++i) {
        const auto& module1 = moduleList[i];
        
        for (size_t j = i + 1; j < moduleList.size(); ++j) {
            const auto& module2 = moduleList[j];
            
            // Skip if modules are in the same symmetry group
            auto it1 = moduleToGroup.find(module1->getName());
            auto it2 = moduleToGroup.find(module2->getName());
            
            if (it1 != moduleToGroup.end() && it2 != moduleToGroup.end() && 
                it1->second == it2->second) {
                continue;
            }
            
            if (module1->overlaps(*module2)) {
                std::cerr << "Overlap detected between modules: " 
                         << module1->getName() << " and " 
                         << module2->getName() << std::endl;
                return true;
            }
        }
    }
    
    return false;
}

bool PlacementPacker::hasGlobalOverlaps(
    const std::map<std::string, std::shared_ptr<Module>>& allModules,
    const std::map<std::string, std::shared_ptr<SymmetryGroup>>& moduleToGroup,
    const std::vector<std::shared_ptr<SymmetryIslandBlock>>& symmetryIslands) {
    
    // 1. Create a map from module name to its parent symmetry island
    std::map<std::string, std::shared_ptr<SymmetryIslandBlock>> moduleToIsland;
    for (const auto& island : symmetryIslands) {
        auto asfTree = island->getASFBStarTree();
        if (!asfTree) continue;
        
        for (const auto& modPair : asfTree->getModules()) {
            moduleToIsland[modPair.first] = island;
        }
    }
    
    // 2. Collect regular modules (not in any symmetry group)
    std::vector<std::shared_ptr<Module>> regularModules;
    for (const auto& pair : allModules) {
        // Skip modules that are part of a symmetry group
        if (moduleToGroup.find(pair.first) != moduleToGroup.end()) {
            continue;
        }
        regularModules.push_back(pair.second);
    }
    
    // 3. Check for overlaps between regular modules
    for (size_t i = 0; i < regularModules.size(); ++i) {
        const auto& module1 = regularModules[i];
        
        for (size_t j = i + 1; j < regularModules.size(); ++j) {
            const auto& module2 = regularModules[j];
            
            if (module1->overlaps(*module2)) {
                std::cerr << "Global overlap detected between regular modules: " 
                         << module1->getName() << " and " 
                         << module2->getName() << std::endl;
                return true;
            }
        }
    }
    
    // 4. Check for overlaps between symmetry islands
    for (size_t i = 0; i < symmetryIslands.size(); ++i) {
        const auto& island1 = symmetryIslands[i];
        
        for (size_t j = i + 1; j < symmetryIslands.size(); ++j) {
            const auto& island2 = symmetryIslands[j];
            
            if (island1->overlaps(*island2)) {
                std::cerr << "Global overlap detected between symmetry islands: " 
                         << island1->getName() << " and " 
                         << island2->getName() << std::endl;
                return true;
            }
        }
    }
    
    // 5. Check for overlaps between regular modules and symmetry islands
    for (const auto& module : regularModules) {
        for (const auto& island : symmetryIslands) {
            // Check if the regular module overlaps with the island's bounding box
            int modLeft = module->getX();
            int modRight = module->getX() + module->getWidth();
            int modBottom = module->getY();
            int modTop = module->getY() + module->getHeight();
            
            int islandLeft = island->getX();
            int islandRight = island->getX() + island->getWidth();
            int islandBottom = island->getY();
            int islandTop = island->getY() + island->getHeight();
            
            // Check for overlap
            bool hasOverlap = !(modRight <= islandLeft || islandRight <= modLeft ||
                               modTop <= islandBottom || islandTop <= modBottom);
            
            if (hasOverlap) {
                std::cerr << "Global overlap detected between regular module " 
                         << module->getName() << " and symmetry island " 
                         << island->getName() << std::endl;
                return true;
            }
        }
    }
    
    return false;
}


bool PlacementPacker::validateSymmetryConstraints(
    const std::map<std::string, std::shared_ptr<Module>>& allModules,
    const std::map<std::string, std::shared_ptr<ASFBStarTree>>& symmetryTrees) {
    
    // [Keep the existing validation code for symmetry constraints]
    
    // Validate each symmetry group
    for (const auto& pair : symmetryTrees) {
        auto symGroup = pair.second->getSymmetryGroup();
        
        // Check symmetry axis
        double axis = pair.second->getSymmetryAxisPosition();
        
        if (symGroup->getType() == SymmetryType::VERTICAL) {
            // Check symmetry pairs
            for (const auto& symPair : symGroup->getSymmetryPairs()) {
                auto it1 = allModules.find(symPair.first);
                auto it2 = allModules.find(symPair.second);
                
                if (it1 == allModules.end() || it2 == allModules.end()) {
                    continue;
                }
                
                auto& mod1 = it1->second;
                auto& mod2 = it2->second;
                
                // Calculate centers
                double center1X = mod1->getX() + mod1->getWidth() / 2.0;
                double center2X = mod2->getX() + mod2->getWidth() / 2.0;
                
                // Verify X symmetry: x1 + x2 = 2 * axis
                if (std::abs((center1X + center2X) - 2 * axis) > 1e-6) {
                    std::cerr << "Vertical symmetry constraint violated for pair: " 
                             << symPair.first << " and " << symPair.second << std::endl;
                    return false;
                }
                
                // Verify Y position: y1 = y2
                if (mod1->getY() != mod2->getY()) {
                    std::cerr << "Y-coordinate mismatch for vertical symmetry pair: " 
                             << symPair.first << " and " << symPair.second << std::endl;
                    return false;
                }
            }
            
            // Check self-symmetric modules
            for (const auto& moduleName : symGroup->getSelfSymmetric()) {
                auto it = allModules.find(moduleName);
                if (it == allModules.end()) {
                    continue;
                }
                
                auto& module = it->second;
                
                // Calculate center
                double centerX = module->getX() + module->getWidth() / 2.0;
                
                // Verify module is centered on axis
                if (std::abs(centerX - axis) > 1e-6) {
                    std::cerr << "Self-symmetric module not centered on vertical axis: " 
                             << moduleName << std::endl;
                    return false;
                }
            }
        } 
        else { // HORIZONTAL
            // Check symmetry pairs
            for (const auto& symPair : symGroup->getSymmetryPairs()) {
                auto it1 = allModules.find(symPair.first);
                auto it2 = allModules.find(symPair.second);
                
                if (it1 == allModules.end() || it2 == allModules.end()) {
                    continue;
                }
                
                auto& mod1 = it1->second;
                auto& mod2 = it2->second;
                
                // Calculate centers
                double center1Y = mod1->getY() + mod1->getHeight() / 2.0;
                double center2Y = mod2->getY() + mod2->getHeight() / 2.0;
                
                // Verify Y symmetry: y1 + y2 = 2 * axis
                if (std::abs((center1Y + center2Y) - 2 * axis) > 1e-6) {
                    std::cerr << "Horizontal symmetry constraint violated for pair: " 
                             << symPair.first << " and " << symPair.second << std::endl;
                    return false;
                }
                
                // Verify X position: x1 = x2
                if (mod1->getX() != mod2->getX()) {
                    std::cerr << "X-coordinate mismatch for horizontal symmetry pair: " 
                             << symPair.first << " and " << symPair.second << std::endl;
                    return false;
                }
            }
            
            // Check self-symmetric modules
            for (const auto& moduleName : symGroup->getSelfSymmetric()) {
                auto it = allModules.find(moduleName);
                if (it == allModules.end()) {
                    continue;
                }
                
                auto& module = it->second;
                
                // Calculate center
                double centerY = module->getY() + module->getHeight() / 2.0;
                
                // Verify module is centered on axis
                if (std::abs(centerY - axis) > 1e-6) {
                    std::cerr << "Self-symmetric module not centered on horizontal axis: " 
                             << moduleName << std::endl;
                    return false;
                }
            }
        }
    }
    
    return true;
}

int PlacementPacker::calculateTotalArea(
    const std::map<std::string, std::shared_ptr<Module>>& allModules,
    int& solutionWidth,
    int& solutionHeight) {
    
    // Find the bounding rectangle of all modules
    int minX = std::numeric_limits<int>::max();
    int minY = std::numeric_limits<int>::max();
    int maxX = 0;
    int maxY = 0;
    
    // Check all modules
    for (const auto& pair : allModules) {
        const auto& module = pair.second;
        
        minX = std::min(minX, module->getX());
        minY = std::min(minY, module->getY());
        maxX = std::max(maxX, module->getX() + module->getWidth());
        maxY = std::max(maxY, module->getY() + module->getHeight());
    }
    
    // Set solution dimensions
    solutionWidth = maxX - minX;
    solutionHeight = maxY - minY;
    
    // Calculate total area
    return solutionWidth * solutionHeight;
}

bool PlacementPacker::isPositionValid(
    int x, int y, int width, int height,
    const std::map<std::string, std::shared_ptr<Module>>& allModules) {
    
    // Create a temporary module at the given position
    auto tempModule = std::make_shared<Module>("temp", width, height);
    tempModule->setPosition(x, y);
    
    // Check for overlaps with all other modules
    for (const auto& pair : allModules) {
        if (tempModule->overlaps(*pair.second)) {
            return false;
        }
    }
    
    return true;
}