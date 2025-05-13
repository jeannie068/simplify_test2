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
    
    // Create a contour for packing
    std::shared_ptr<Contour> contour = std::make_shared<Contour>();
    
    // Initialize contour with a segment at y=0
    contour->addSegment(0, std::numeric_limits<int>::max(), 0);
    
    // Pack using BFS traversal
    std::queue<std::shared_ptr<BStarTreeNode>> queue;
    queue.push(globalTree);
    
    while (!queue.empty()) {
        auto node = queue.front();
        queue.pop();
        
        const std::string& nodeName = node->getModuleName();
        auto it = globalNodes.find(nodeName);
        if (it == globalNodes.end()) {
            continue;
        }
        
        int width = 0, height = 0;
        
        // Calculate position based on B*-tree rules
        int x = 0, y = 0;
        
        if (node->getParent()) {
            auto parentName = node->getParent()->getModuleName();
            auto parentIt = globalNodes.find(parentName);
            if (parentIt != globalNodes.end()) {
                int parentX = 0, parentY = 0, parentWidth = 0, parentHeight = 0;
                
                // Get parent dimensions and position
                std::visit([&](auto&& arg) {
                    using T = std::decay_t<decltype(arg)>;
                    if constexpr (std::is_same_v<T, std::shared_ptr<Module>>) {
                        parentX = arg->getX();
                        parentY = arg->getY();
                        parentWidth = arg->getWidth();
                        parentHeight = arg->getHeight();
                    } else if constexpr (std::is_same_v<T, std::shared_ptr<SymmetryIslandBlock>>) {
                        parentX = arg->getX();
                        parentY = arg->getY();
                        parentWidth = arg->getWidth();
                        parentHeight = arg->getHeight();
                    }
                }, parentIt->second);
                
                if (node->isLeftChild()) {
                    // Left child: place to the right of parent
                    x = parentX + parentWidth;
                    y = parentY;
                } else {
                    // Right child: place above parent with same x
                    x = parentX;
                    y = parentY + parentHeight;
                }
            }
        }
        
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
        
        // Get minimum y from the contour at this x position
        y = std::max(y, contour->getHeight(x, x + width));
        
        // Set position
        std::visit([x, y](auto&& arg) {
            using T = std::decay_t<decltype(arg)>;
            if constexpr (std::is_same_v<T, std::shared_ptr<Module>>) {
                arg->setPosition(x, y);
            } else if constexpr (std::is_same_v<T, std::shared_ptr<SymmetryIslandBlock>>) {
                arg->setPosition(x, y);
            }
        }, it->second);
        
        // Update contour
        contour->addSegment(x, x + width, y + height);
        
        // Add children to queue
        if (node->getLeftChild()) {
            queue.push(node->getLeftChild());
        }
        if (node->getRightChild()) {
            queue.push(node->getRightChild());
        }
    }
    
    return true;
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

bool PlacementPacker::validateSymmetryConstraints(
    const std::map<std::string, std::shared_ptr<Module>>& allModules,
    const std::map<std::string, std::shared_ptr<ASFBStarTree>>& symmetryTrees) {
    
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