// ASFBStarTree.hpp
#pragma once

#include <memory>
#include <vector>
#include <map>
#include <unordered_map>
#include <algorithm>
#include <queue>
#include <cmath>
#include <string>
#include <random>
#include <iostream>

#include "Module.hpp"
#include "SymmetryConstraint.hpp"
#include "ASFBStarTree.hpp"
#include "../Logger.hpp"


/**
 * Packs the B*-tree to get the coordinates of all modules
 * This implementation uses level-order traversal (BFS)
 */
void ASFBStarTree::packBStarTree() {
    // Clear the contour
    clearContour();
    
    Logger::log("Starting to pack ASF-B*-tree");
    Logger::logTreeStructure("Pre-pack Tree", root);
    
    // Log all node names and their relationships
    std::function<void(BStarNode*)> logNodeRelationships = [&](BStarNode* node) {
        if (node == nullptr) return;
        
        if (node->left) {
            Logger::log(node->moduleName + " has left child: " + node->left->moduleName);
        }
        if (node->right) {
            Logger::log(node->moduleName + " has right child: " + node->right->moduleName);
        }
        
        logNodeRelationships(node->left);
        logNodeRelationships(node->right);
    };
    
    logNodeRelationships(root);
    
    // Create a parent-child relationship map for efficient lookup
    std::unordered_map<BStarNode*, BStarNode*> parentMap;
    std::function<void(BStarNode*, BStarNode*)> buildParentMap = [&](BStarNode* node, BStarNode* parent) {
        if (node == nullptr) return;
        parentMap[node] = parent;
        buildParentMap(node->left, node);
        buildParentMap(node->right, node);
    };
    buildParentMap(root, nullptr);
    
    // Initialize node positions
    std::unordered_map<BStarNode*, std::pair<int, int>> nodePositions;
    
    // Use level-order traversal (BFS) to ensure parents are processed before children
    std::queue<BStarNode*> bfsQueue;
    if (root != nullptr) {
        bfsQueue.push(root);
        // Root is placed at (0,0)
        nodePositions[root] = {0, 0};
        
        // Update the module position
        modules[root->moduleName]->setPosition(0, 0);
        
        // Update the contour
        updateContour(0, 0, modules[root->moduleName]->getWidth(), modules[root->moduleName]->getHeight());
        Logger::log("Placed root " + root->moduleName + " at (0, 0)");
    }
    
    int processedNodes = 0;
    
    try {
        while (!bfsQueue.empty()) {
            BStarNode* node = bfsQueue.front();
            bfsQueue.pop();
            processedNodes++;
            
            // Get current node position
            int nodeX = nodePositions[node].first;
            int nodeY = nodePositions[node].second;
            
            Logger::log("Processing node: " + node->moduleName + " at position (" + 
                        std::to_string(nodeX) + ", " + std::to_string(nodeY) + ")");
            
            // Process left child (placed to the right of current node)
            if (node->left) {
                int leftX = nodeX + modules[node->moduleName]->getWidth();
                int leftY = getContourHeight(leftX);
                
                // Store position of left child
                nodePositions[node->left] = {leftX, leftY};
                
                // Update module position
                modules[node->left->moduleName]->setPosition(leftX, leftY);
                
                // Update the contour
                updateContour(leftX, leftY, 
                                modules[node->left->moduleName]->getWidth(), 
                                modules[node->left->moduleName]->getHeight());
                
                Logger::log("Placed left child " + node->left->moduleName + 
                            " at (" + std::to_string(leftX) + ", " + std::to_string(leftY) + ")");
                
                // Add to queue for further processing
                bfsQueue.push(node->left);
            }
            
            // Process right child (placed at same x, above current node)
            if (node->right) {
                int rightX = nodeX;
                int rightY = nodeY + modules[node->moduleName]->getHeight();
                
                // Store position of right child
                nodePositions[node->right] = {rightX, rightY};
                
                // Update module position
                modules[node->right->moduleName]->setPosition(rightX, rightY);
                
                // Update the contour
                updateContour(rightX, rightY, 
                                modules[node->right->moduleName]->getWidth(), 
                                modules[node->right->moduleName]->getHeight());
                
                Logger::log("Placed right child " + node->right->moduleName + 
                            " at (" + std::to_string(rightX) + ", " + std::to_string(rightY) + ")");
                
                // Add to queue for further processing
                bfsQueue.push(node->right);
            }
        }
        
        Logger::log("Successfully processed " + std::to_string(processedNodes) + " nodes");
        
    } catch (const std::exception& e) {
        Logger::log("Exception during packing: " + std::string(e.what()));
        throw; // Re-throw the exception after logging
    }
}

/**
 * Updates the contour after placing a module
 * 
 * @param x X-coordinate of the module
 * @param y Y-coordinate of the module
 * @param width Width of the module
 * @param height Height of the module
 */
void ASFBStarTree::updateContour(int x, int y, int width, int height) {
    int right = x + width;
    int top = y + height;
    
    // Create a new contour point if contour is empty
    if (contourHead == nullptr) {
        contourHead = new ContourPoint(x, top);
        contourHead->next = new ContourPoint(right, 0);
        return;
    }
    
    // Find the position to insert or update
    ContourPoint* curr = contourHead;
    ContourPoint* prev = nullptr;
    
    // Skip points to the left of the module
    while (curr != nullptr && curr->x < x) {
        prev = curr;
        curr = curr->next;
    }
    
    // Handle case where we need to insert a point at x
    if (curr == nullptr || curr->x > x) {
        // Create a new point at the left edge of the module
        ContourPoint* newPoint = new ContourPoint(x, top);
        
        // Link the new point
        if (prev) {
            prev->next = newPoint;
        } else {
            contourHead = newPoint;
        }
        newPoint->next = curr;
        
        // Update for next operations
        prev = newPoint;
    } else if (curr->x == x) {
        // Update existing point at x
        curr->height = std::max(curr->height, top);
        prev = curr;
        curr = curr->next;
    }
    
    // Process or remove intermediate points up to right edge
    while (curr != nullptr && curr->x < right) {
        if (curr->height <= top) {
            // This point is below or at our new contour height, so remove it
            ContourPoint* temp = curr;
            curr = curr->next;
            delete temp;
            
            if (prev) {
                prev->next = curr;
            } else {
                contourHead = curr;
            }
        } else {
            // This point is above our new contour, keep it
            prev = curr;
            curr = curr->next;
        }
    }
    
    // Handle the right edge point
    if (curr == nullptr || curr->x > right) {
        // Add a new point at the right edge
        ContourPoint* newPoint = new ContourPoint(right, prev ? prev->height : 0);
        newPoint->next = curr;
        
        if (prev) {
            prev->next = newPoint;
        } else {
            contourHead = newPoint;
        }
    } else if (curr->x == right) {
        // Update existing point at the right edge
        curr->height = std::max(curr->height, prev ? prev->height : 0);
    }
}


/**
 * Calculates the position of the symmetry axis based on the current placement
 * Using the principles from the paper: for vertical symmetry, axis should be at leftmost point
 */
void ASFBStarTree::calculateSymmetryAxisPosition() {
    Logger::log("Calculating symmetry axis position");
    
    if (symmetryGroup->getType() == SymmetryType::VERTICAL) {
        // For vertical symmetry, find the leftmost edge of any representative module
        int minX = std::numeric_limits<int>::max();
        for (const auto& pair : representativeModules) {
            const auto& module = modules[pair.first];
            minX = std::min(minX, module->getX());
        }
        
        // Set axis at the leftmost edge to ensure all representative modules are on right side
        symmetryAxisPosition = minX;
        
        // Adjust self-symmetric modules to center on the axis
        for (const auto& moduleName : selfSymmetricModules) {
            auto module = modules[moduleName];
            // Align the center of self-symmetric module with the axis
            int width = module->getWidth();
            module->setPosition(minX - width/2, module->getY());
        }
        
        Logger::log("Set vertical symmetry axis at x=" + std::to_string(symmetryAxisPosition));
    } else {
        // For horizontal symmetry, find the lowest edge of any representative module
        int minY = std::numeric_limits<int>::max();
        for (const auto& pair : representativeModules) {
            const auto& module = modules[pair.first];
            minY = std::min(minY, module->getY());
        }
        
        // Set axis at the lowest edge to ensure all representative modules are above
        symmetryAxisPosition = minY;
        
        // Adjust self-symmetric modules to center on the axis
        for (const auto& moduleName : selfSymmetricModules) {
            auto module = modules[moduleName];
            // Align the center of self-symmetric module with the axis
            int height = module->getHeight();
            module->setPosition(module->getX(), minY - height/2);
        }
        
        Logger::log("Set horizontal symmetry axis at y=" + std::to_string(symmetryAxisPosition));
    }
    
    symmetryGroup->setAxisPosition(symmetryAxisPosition);
}

/**
 * Updates the positions of symmetric modules based on their representatives
 * Correctly implements the mirroring according to the paper's equations
 */
void ASFBStarTree::updateSymmetricModulePositions() {
    // Make sure the symmetry axis is set
    if (symmetryAxisPosition < 0) {
        calculateSymmetryAxisPosition();
    }
    
    Logger::log("Updating symmetric module positions with axis at " + std::to_string(symmetryAxisPosition));
    
    // Update positions for symmetry pairs
    for (const auto& pair : repToPairMap) {
        const std::string& repName = pair.first;
        const std::string& symName = pair.second;
        
        std::shared_ptr<Module> repModule = modules[repName];
        std::shared_ptr<Module> symModule = modules[symName];
        
        if (symmetryGroup->getType() == SymmetryType::VERTICAL) {
            // For vertical symmetry (equation 1 in the paper):
            // For proper mirroring, we need to reflect the entire module, not just its center
            int repRightEdge = repModule->getX() + repModule->getWidth();
            int distanceFromAxis = repRightEdge - static_cast<int>(symmetryAxisPosition);
            int symX = static_cast<int>(symmetryAxisPosition) - distanceFromAxis - symModule->getWidth();
            
            symModule->setPosition(symX, repModule->getY());
            
            Logger::log("Vertical symmetry: " + repName + " at (" + std::to_string(repModule->getX()) + 
                       ", " + std::to_string(repModule->getY()) + ") -> " + symName + " at (" + 
                       std::to_string(symX) + ", " + std::to_string(repModule->getY()) + ")");
        } else {
            // For horizontal symmetry (equation 2 in the paper):
            int repTopEdge = repModule->getY() + repModule->getHeight();
            int distanceFromAxis = repTopEdge - static_cast<int>(symmetryAxisPosition);
            int symY = static_cast<int>(symmetryAxisPosition) - distanceFromAxis - symModule->getHeight();
            
            symModule->setPosition(repModule->getX(), symY);
            
            Logger::log("Horizontal symmetry: " + repName + " at (" + std::to_string(repModule->getX()) + 
                       ", " + std::to_string(repModule->getY()) + ") -> " + symName + " at (" + 
                       std::to_string(repModule->getX()) + ", " + std::to_string(symY) + ")");
        }
        
        // Ensure the rotation is consistent for the symmetry pair
        symModule->setRotation(repModule->getRotated());
    }
    
    // Self-symmetric modules are already handled in calculateSymmetryAxisPosition
}

/**
 * Builds an initial B*-tree for the symmetry group with improved placement strategy
 * Places representative modules properly relative to symmetry axis
 */
void ASFBStarTree::buildInitialBStarTree() {
    Logger::log("Building initial ASF-B*-tree with improved placement strategy");
    
    // Clean up any existing tree
    cleanupTree(root);
    root = nullptr;
    
    // Get all representative modules
    std::vector<std::string> repModuleNames;
    for (const auto& pair : representativeModules) {
        repModuleNames.push_back(pair.first);
    }
    
    Logger::log("Total representative modules: " + std::to_string(repModuleNames.size()));
    
    // Separate self-symmetric and non-self-symmetric modules
    std::vector<std::string> nonSelfSymModules;
    std::vector<std::string> selfSymModules = selfSymmetricModules;
    
    for (const auto& name : repModuleNames) {
        if (!isSelfSymmetric(name)) {
            nonSelfSymModules.push_back(name);
        }
    }
    
    Logger::log("Self-symmetric modules: " + std::to_string(selfSymModules.size()));
    Logger::log("Non-self-symmetric modules: " + std::to_string(nonSelfSymModules.size()));
    
    // Randomize non-self-symmetric modules for variety
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(nonSelfSymModules.begin(), nonSelfSymModules.end(), g);
    
    // Create nodes for all modules
    std::unordered_map<std::string, BStarNode*> nodeMap;
    for (const auto& name : repModuleNames) {
        nodeMap[name] = new BStarNode(name);
        Logger::log("Created node for module: " + name);
    }
    
    // Build a tree that arranges modules with symmetry constraints in mind
    if (!repModuleNames.empty()) {
        // Choose the first root - prefer a module with smaller width/height for compactness
        std::string rootName;
        if (!nonSelfSymModules.empty()) {
            // Sort non-self-symmetric modules by width (for vertical symmetry)
            // or height (for horizontal symmetry)
            std::vector<std::pair<std::string, int>> sortedModules;
            for (const auto& name : nonSelfSymModules) {
                if (symmetryGroup->getType() == SymmetryType::VERTICAL) {
                    sortedModules.push_back({name, modules[name]->getWidth()});
                } else {
                    sortedModules.push_back({name, modules[name]->getHeight()});
                }
            }
            
            // Sort by dimension (ascending)
            std::sort(sortedModules.begin(), sortedModules.end(),
                     [](const auto& a, const auto& b) { return a.second < b.second; });
            
            rootName = sortedModules.front().first;
            
            // Remove from list
            auto it = std::find(nonSelfSymModules.begin(), nonSelfSymModules.end(), rootName);
            if (it != nonSelfSymModules.end()) {
                nonSelfSymModules.erase(it);
            }
            
            Logger::log("Using non-self-symmetric module as root: " + rootName);
        } else if (!selfSymModules.empty()) {
            rootName = selfSymModules.front();
            selfSymModules.erase(selfSymModules.begin());
            Logger::log("Using self-symmetric module as root: " + rootName);
        } else {
            Logger::log("ERROR: No modules to place in symmetry group");
            throw std::runtime_error("No modules to place in symmetry group");
        }
        
        root = nodeMap[rootName];
        
        // For self-symmetric modules, ensure they're on the proper boundary branch
        // For vertical symmetry: rightmost branch
        // For horizontal symmetry: leftmost branch
        BStarNode* currSelfSym = root;
        
        for (const auto& name : selfSymModules) {
            if (symmetryGroup->getType() == SymmetryType::VERTICAL) {
                // For vertical symmetry, self-symmetric modules on rightmost branch
                Logger::log("Placing self-symmetric module " + name + " as right child of " + currSelfSym->moduleName);
                currSelfSym->right = nodeMap[name];
                currSelfSym = currSelfSym->right;
            } else {
                // For horizontal symmetry, self-symmetric modules on leftmost branch
                Logger::log("Placing self-symmetric module " + name + " as left child of " + currSelfSym->moduleName);
                currSelfSym->left = nodeMap[name];
                currSelfSym = currSelfSym->left;
            }
        }
        
        // Build an optimized tree for the remaining non-self-symmetric modules
        // In vertical symmetry, we want a "to the right and upward" structure
        // In horizontal symmetry, we want a "downward and to the right" structure
        BStarNode* currNode = root;
        
        for (const auto& name : nonSelfSymModules) {
            if (symmetryGroup->getType() == SymmetryType::VERTICAL) {
                // Find a node with no left child (preference to place to the right)
                BStarNode* target = nullptr;
                std::function<void(BStarNode*)> findOpenLeftSlot = [&](BStarNode* node) {
                    if (node == nullptr) return;
                    if (node->left == nullptr) {
                        target = node;
                        return;
                    }
                    findOpenLeftSlot(node->left);  // Try left subtree first
                    if (target == nullptr) findOpenLeftSlot(node->right);
                };
                
                findOpenLeftSlot(root);
                
                if (target != nullptr) {
                    Logger::log("Placing module " + name + " as left child of " + target->moduleName);
                    target->left = nodeMap[name];
                } else {
                    // Fall back to placing as right child of leaf node
                    BStarNode* leafNode = currNode;
                    Logger::log("Placing module " + name + " as right child of " + leafNode->moduleName);
                    leafNode->right = nodeMap[name];
                    currNode = leafNode->right;  // Update current node
                }
            } else {
                // For horizontal symmetry, prefer right placements
                BStarNode* target = nullptr;
                std::function<void(BStarNode*)> findOpenRightSlot = [&](BStarNode* node) {
                    if (node == nullptr) return;
                    if (node->right == nullptr) {
                        target = node;
                        return;
                    }
                    findOpenRightSlot(node->right);  // Try right subtree first
                    if (target == nullptr) findOpenRightSlot(node->left);
                };
                
                findOpenRightSlot(root);
                
                if (target != nullptr) {
                    Logger::log("Placing module " + name + " as right child of " + target->moduleName);
                    target->right = nodeMap[name];
                } else {
                    // Fall back to placing as left child of leaf node
                    BStarNode* leafNode = currNode;
                    Logger::log("Placing module " + name + " as left child of " + leafNode->moduleName);
                    leafNode->left = nodeMap[name];
                    currNode = leafNode->left;  // Update current node
                }
            }
        }
    }
    
    // Log the resulting tree
    Logger::logTreeStructure("Initial ASF-B*-tree", root);
    
    // Validate the tree structure
    if (!validateTreeStructure(root)) {
        Logger::log("CRITICAL: Invalid tree structure after initialization");
        throw std::runtime_error("Invalid tree structure after initialization");
    }
    
    // Validate the symmetry constraints
    if (!validateSymmetryConstraints()) {
        Logger::log("CRITICAL: Tree does not meet symmetry constraints after initialization");
        throw std::runtime_error("Tree does not meet symmetry constraints after initialization");
    }
}

/**
 * Packs the ASF-B*-tree to get the coordinates of all modules
 * 
 * @return True if packing was successful
 */
bool ASFBStarTree::pack() {
    try {
        // Update the traversals for the current B*-tree
        preorderTraversal.clear();
        inorderTraversal.clear();
        preorder(root);
        inorder(root);
        
        Logger::log("Starting ASF-B*-tree packing with " + 
                   std::to_string(preorderTraversal.size()) + " nodes");
        
        // Pack the B*-tree to get coordinates for representatives
        packBStarTree();
        
        // First calculate the symmetry axis position
        calculateSymmetryAxisPosition();
        
        // Then update positions of symmetric modules
        updateSymmetricModulePositions();
        
        // Validate the resulting placement satisfies symmetry constraints
        if (!validateSymmetry()) {
            Logger::log("ERROR: Placement does not satisfy symmetry constraints");
            
            // Debug information to understand the problem
            for (const auto& pair : modules) {
                Logger::log("Module: " + pair.first + " at (" + 
                           std::to_string(pair.second->getX()) + ", " + 
                           std::to_string(pair.second->getY()) + ") with dimensions " +
                           std::to_string(pair.second->getWidth()) + "x" + 
                           std::to_string(pair.second->getHeight()));
            }
            
            return false;
        }
        
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error packing ASF-B*-tree: " << e.what() << std::endl;
        Logger::log("Exception during packing: " + std::string(e.what()));
        return false;
    }
}

/**
 * Validates the symmetric placement specifically checking for negative coordinates
 */
bool ASFBStarTree::validateSymmetry() const {
    // First check for negative coordinates which would invalidate the placement
    for (const auto& pair : modules) {
        const auto& module = pair.second;
        if (module->getX() < 0 || module->getY() < 0) {
            Logger::log("ERROR: Module " + pair.first + " has negative coordinates (" + 
                       std::to_string(module->getX()) + ", " + 
                       std::to_string(module->getY()) + ")");
            return false;
        }
    }
    
    // Then validate that the tree structure satisfies Property 1
    if (!const_cast<ASFBStarTree*>(this)->validateSymmetryConstraints()) {
        return false;
    }
    
    // Then validate that the placement satisfies the symmetry constraints
    return symmetryGroup->validateSymmetricPlacement(
        [this]() {
            std::unordered_map<std::string, std::pair<int, int>> positions;
            
            for (const auto& pair : modules) {
                const std::string& name = pair.first;
                const auto& module = pair.second;
                positions[name] = {module->getX(), module->getY()};
            }
            
            return positions;
        }(),
        [this]() {
            std::unordered_map<std::string, std::pair<int, int>> dimensions;
            
            for (const auto& pair : modules) {
                const std::string& name = pair.first;
                const auto& module = pair.second;
                dimensions[name] = {module->getWidth(), module->getHeight()};
            }
            
            return dimensions;
        }()
    );
}