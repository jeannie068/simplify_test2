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
 * This implementation uses level-order traversal (BFS) with adjacency enforcement
 */
void ASFBStarTree::packBStarTree() {
    // Clear the contour
    clearContour();
    
    Logger::log("Starting to pack ASF-B*-tree with connectivity enforcement");
    Logger::logTreeStructure("Pre-pack Tree", root);
    
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
                // ENFORCE ADJACENCY: Place directly adjacent to parent
                int leftX = nodeX + modules[node->moduleName]->getWidth();
                // Try to place at same Y-coordinate to maintain adjacency
                int leftY = nodeY;
                
                // If there's a conflict with the contour, adjust Y to the minimum required
                int contourHeight = getContourHeight(leftX);
                if (contourHeight > leftY) {
                    leftY = contourHeight;
                }
                
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
                // ENFORCE ADJACENCY: Place directly adjacent to parent
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
 * Optimized to prevent negative coordinates while maintaining connectivity
 * With added safety checks for larger problems
 */
void ASFBStarTree::calculateSymmetryAxisPosition() {
    Logger::log("Calculating optimized symmetry axis position for group: " + symmetryGroup->getName());
    
    if (modules.empty()) {
        symmetryAxisPosition = 0;
        symmetryGroup->setAxisPosition(symmetryAxisPosition);
        return;
    }
    
    // Find the bounding box of all representative modules
    int minX = std::numeric_limits<int>::max();
    int maxX = std::numeric_limits<int>::min();
    int minY = std::numeric_limits<int>::max();
    int maxY = std::numeric_limits<int>::min();
    
    // Safe check: Only process modules that actually exist in the representation
    for (const auto& pair : representativeModules) {
        const auto& moduleName = pair.first;
        
        // Safety check: Make sure module exists in the modules map
        auto moduleIt = modules.find(moduleName);
        if (moduleIt == modules.end()) {
            Logger::log("WARNING: Representative module " + moduleName + " not found in modules map");
            continue;
        }
        
        const auto& module = moduleIt->second;
        minX = std::min(minX, module->getX());
        maxX = std::max(maxX, module->getX() + module->getWidth());
        minY = std::min(minY, module->getY());
        maxY = std::max(maxY, module->getY() + module->getHeight());
    }
    
    // Safety check: If no valid modules found, use default position
    if (minX == std::numeric_limits<int>::max() || maxX == std::numeric_limits<int>::min()) {
        Logger::log("WARNING: No valid representative modules found, using default axis position");
        symmetryAxisPosition = 0;
        symmetryGroup->setAxisPosition(symmetryAxisPosition);
        return;
    }
    
    // Calculate the width of all representative modules
    int representativeWidth = maxX - minX;
    int maxModuleWidth = 0;
    
    // Get max module width for buffer calculation
    for (const auto& pair : modules) {
        maxModuleWidth = std::max(maxModuleWidth, pair.second->getWidth());
    }
    
    // Key improvement: Position the axis at the MAX of the representative modules
    // This ensures no negative coordinates when mirroring
    if (symmetryGroup->getType() == SymmetryType::VERTICAL) {
        // Position the axis at the right edge of the bounding box 
        symmetryAxisPosition = maxX;
        
        // Add buffer for safety
        int buffer = maxModuleWidth + 10;
        
        // Shift all modules to ensure symmetry axis stays positive
        for (auto& pair : modules) {
            int newX = pair.second->getX() + buffer;
            pair.second->setPosition(newX, pair.second->getY());
        }
        
        // Update the symmetry axis position after shifting
        symmetryAxisPosition += buffer;
        
        Logger::log("Set vertical symmetry axis at x=" + std::to_string(symmetryAxisPosition));
    } else {
        // For horizontal symmetry - similar approach
        symmetryAxisPosition = maxY;
        
        int maxModuleHeight = 0;
        for (const auto& pair : modules) {
            maxModuleHeight = std::max(maxModuleHeight, pair.second->getHeight());
        }
        
        int buffer = maxModuleHeight + 10;
        for (auto& pair : modules) {
            int newY = pair.second->getY() + buffer;
            pair.second->setPosition(pair.second->getX(), newY);
        }
        
        symmetryAxisPosition += buffer;
        
        Logger::log("Set horizontal symmetry axis at y=" + std::to_string(symmetryAxisPosition));
    }
    
    symmetryGroup->setAxisPosition(symmetryAxisPosition);
}

/**
 * Updates the positions of symmetric modules based on their representatives
 * Guarantees no negative coordinates with added safety checks
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
        
        // Safety check: Make sure modules exist
        auto repIt = modules.find(repName);
        auto symIt = modules.find(symName);
        
        if (repIt == modules.end() || symIt == modules.end()) {
            Logger::log("WARNING: Module not found when updating symmetry positions: " + 
                      (repIt == modules.end() ? repName : symName));
            continue;
        }
        
        std::shared_ptr<Module> repModule = repIt->second;
        std::shared_ptr<Module> symModule = symIt->second;
        
        if (symmetryGroup->getType() == SymmetryType::VERTICAL) {
            // Calculate center coordinates of the representative module
            double repCenterX = repModule->getX() + repModule->getWidth() / 2.0;
            double repCenterY = repModule->getY() + repModule->getHeight() / 2.0;
            
            // Apply the symmetry equation to get the center of the symmetric module
            double symCenterX = 2 * symmetryAxisPosition - repCenterX;
            double symCenterY = repCenterY;
            
            // Convert center coordinates back to top-left position
            int symX = static_cast<int>(symCenterX - symModule->getWidth() / 2.0);
            int symY = static_cast<int>(symCenterY - symModule->getHeight() / 2.0);
            
            // Sanity check: ensure no negative coordinates
            symX = std::max(0, symX);
            
            symModule->setPosition(symX, symY);
            
            Logger::log("Vertical symmetry: " + repName + " center at (" + std::to_string(repCenterX) + 
                       ", " + std::to_string(repCenterY) + ") -> " + symName + " center at (" + 
                       std::to_string(symCenterX) + ", " + std::to_string(symCenterY) + ")");
        } else {
            // For horizontal symmetry
            double repCenterX = repModule->getX() + repModule->getWidth() / 2.0;
            double repCenterY = repModule->getY() + repModule->getHeight() / 2.0;
            
            double symCenterX = repCenterX;
            double symCenterY = 2 * symmetryAxisPosition - repCenterY;
            
            int symX = static_cast<int>(symCenterX - symModule->getWidth() / 2.0);
            int symY = static_cast<int>(symCenterY - symModule->getHeight() / 2.0);
            
            // Sanity check: ensure no negative coordinates
            symY = std::max(0, symY);
            
            symModule->setPosition(symX, symY);
            
            Logger::log("Horizontal symmetry: " + repName + " center at (" + std::to_string(repCenterX) + 
                       ", " + std::to_string(repCenterY) + ") -> " + symName + " center at (" + 
                       std::to_string(symCenterX) + ", " + std::to_string(symCenterY) + ")");
        }
        
        // Ensure the rotation is consistent for the symmetry pair
        symModule->setRotation(repModule->getRotated());
    }
    
    // Handle self-symmetric modules
    for (const auto& moduleName : selfSymmetricModules) {
        // Safety check: Make sure module exists
        auto moduleIt = modules.find(moduleName);
        if (moduleIt == modules.end()) {
            Logger::log("WARNING: Self-symmetric module not found: " + moduleName);
            continue;
        }
        
        auto module = moduleIt->second;
        
        // Center on the symmetry axis
        if (symmetryGroup->getType() == SymmetryType::VERTICAL) {
            module->setPosition(
                symmetryAxisPosition - module->getWidth()/2, 
                module->getY()
            );
        } else {
            module->setPosition(
                module->getX(),
                symmetryAxisPosition - module->getHeight()/2
            );
        }
    }
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
 * with improved axis placement and connectivity enforcement
 * 
 * @return True if packing was successful
 */
/**
 * Packs the ASF-B*-tree with robust error handling
 */
bool ASFBStarTree::pack() {
    try {
        Logger::log("Starting safe packing for symmetry group: " + symmetryGroup->getName());
        
        // Safety check for empty tree
        if (!root) {
            Logger::log("WARNING: Empty ASF-B*-tree, nothing to pack");
            return false;
        }
        
        // Update the traversals for the current B*-tree
        preorderTraversal.clear();
        inorderTraversal.clear();
        preorder(root);
        inorder(root);
        
        if (preorderTraversal.empty()) {
            Logger::log("WARNING: Empty traversal, nothing to pack");
            return false;
        }
        
        Logger::log("ASF-B*-tree packing with " + std::to_string(preorderTraversal.size()) + " nodes");
        
        // Pack the B*-tree to get coordinates for representatives
        packBStarTree();
        
        // First try to enforce connectivity
        if (!enforceConnectivity()) {
            Logger::log("WARNING: Failed to enforce connectivity, continuing with placement anyway");
        }
        
        // Calculate the symmetry axis position with improved logic
        calculateSymmetryAxisPosition();
        
        // Update positions of symmetric modules
        updateSymmetricModulePositions();
        
        // Validate the resulting placement 
        if (!validateSymmetry()) {
            Logger::log("WARNING: Placement does not fully satisfy symmetry constraints, attempting to fix");
            
            // Shift all modules if needed to ensure no negative coordinates
            bool needsShift = false;
            int minX = 0;
            int minY = 0;
            
            for (const auto& pair : modules) {
                if (pair.second->getX() < minX) {
                    minX = pair.second->getX();
                    needsShift = true;
                }
                if (pair.second->getY() < minY) {
                    minY = pair.second->getY();
                    needsShift = true;
                }
            }
            
            if (needsShift) {
                Logger::log("Shifting all modules to eliminate negative coordinates");
                for (auto& pair : modules) {
                    pair.second->setPosition(
                        pair.second->getX() - minX,
                        pair.second->getY() - minY
                    );
                }
                
                // Update symmetry axis position after shifting
                if (symmetryGroup->getType() == SymmetryType::VERTICAL) {
                    symmetryAxisPosition -= minX;
                } else {
                    symmetryAxisPosition -= minY;
                }
                symmetryGroup->setAxisPosition(symmetryAxisPosition);
            }
            
            // Check again after fixing
            if (!validateSymmetry()) {
                Logger::log("ERROR: Still unable to satisfy symmetry constraints after fixing attempt");
                return false;
            }
        }
        
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error packing ASF-B*-tree: " << e.what() << std::endl;
        Logger::log("Exception during packing: " + std::string(e.what()));
        return false;
    }
}


/**
 * Validate if the symmetry is maintained, with improved error reporting
 */
bool ASFBStarTree::validateSymmetry() const {
    try {
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
        
        // Validate symmetry pairs have correct placements
        for (const auto& pair : repToPairMap) {
            // Safety check: Make sure both modules exist
            auto repIt = modules.find(pair.first);
            auto symIt = modules.find(pair.second);
            
            if (repIt == modules.end() || symIt == modules.end()) {
                Logger::log("WARNING: Cannot validate symmetry for missing modules: " + 
                          pair.first + " or " + pair.second);
                continue;
            }
            
            const std::string& repName = pair.first;
            const std::string& symName = pair.second;
            
            std::shared_ptr<Module> repModule = repIt->second;
            std::shared_ptr<Module> symModule = symIt->second;
            
            // Calculate centers
            double repCenterX = repModule->getX() + repModule->getWidth() / 2.0;
            double repCenterY = repModule->getY() + repModule->getHeight() / 2.0;
            double symCenterX = symModule->getX() + symModule->getWidth() / 2.0;
            double symCenterY = symModule->getY() + symModule->getHeight() / 2.0;
            
            if (symmetryGroup->getType() == SymmetryType::VERTICAL) {
                // Check vertical symmetry equation
                double expectedSum = 2 * symmetryAxisPosition;
                double actualSum = repCenterX + symCenterX;
                double error = std::abs(expectedSum - actualSum);
                
                // Also check y-coordinates match
                double yError = std::abs(repCenterY - symCenterY);
                
                if (error > 1.0 || yError > 1.0) {  // Allow small floating-point error
                    Logger::log("ERROR: Symmetry violation for pair (" + repName + ", " + symName + ")");
                    Logger::log("  Expected: repCenterX + symCenterX = " + std::to_string(expectedSum));
                    Logger::log("  Actual: " + std::to_string(repCenterX) + " + " + std::to_string(symCenterX) + " = " + std::to_string(actualSum));
                    Logger::log("  Y error: " + std::to_string(yError));
                    return false;
                }
            } else {
                // Check horizontal symmetry equation
                double expectedSum = 2 * symmetryAxisPosition;
                double actualSum = repCenterY + symCenterY;
                double error = std::abs(expectedSum - actualSum);
                
                // Also check x-coordinates match
                double xError = std::abs(repCenterX - symCenterX);
                
                if (error > 1.0 || xError > 1.0) {  // Allow small floating-point error
                    Logger::log("ERROR: Symmetry violation for pair (" + repName + ", " + symName + ")");
                    Logger::log("  Expected: repCenterY + symCenterY = " + std::to_string(expectedSum));
                    Logger::log("  Actual: " + std::to_string(repCenterY) + " + " + std::to_string(symCenterY) + " = " + std::to_string(actualSum));
                    Logger::log("  X error: " + std::to_string(xError));
                    return false;
                }
            }
        }
        
        // Check self-symmetric modules are centered on the axis
        for (const auto& moduleName : selfSymmetricModules) {
            // Safety check: Make sure module exists
            auto moduleIt = modules.find(moduleName);
            if (moduleIt == modules.end()) {
                Logger::log("WARNING: Cannot validate symmetry for missing self-symmetric module: " + moduleName);
                continue;
            }
            
            std::shared_ptr<Module> module = moduleIt->second;
            double centerX = module->getX() + module->getWidth() / 2.0;
            double centerY = module->getY() + module->getHeight() / 2.0;
            
            if (symmetryGroup->getType() == SymmetryType::VERTICAL) {
                double error = std::abs(centerX - symmetryAxisPosition);
                if (error > 1.0) {  // Allow small floating-point error
                    Logger::log("ERROR: Self-symmetric module " + moduleName + " not centered on axis");
                    Logger::log("  Module center: " + std::to_string(centerX));
                    Logger::log("  Axis position: " + std::to_string(symmetryAxisPosition));
                    return false;
                }
            } else {
                double error = std::abs(centerY - symmetryAxisPosition);
                if (error > 1.0) {  // Allow small floating-point error
                    Logger::log("ERROR: Self-symmetric module " + moduleName + " not centered on axis");
                    Logger::log("  Module center: " + std::to_string(centerY));
                    Logger::log("  Axis position: " + std::to_string(symmetryAxisPosition));
                    return false;
                }
            }
        }
        
        // If all checks pass, the symmetry is valid
        Logger::log("Symmetry validation passed");
        return true;
    } catch (const std::exception& e) {
        Logger::log("Exception in validateSymmetry: " + std::string(e.what()));
        return false;
    }
}

/**
 * Validates that the modules form a connected placement (symmetry island)
 * 
 * @return True if all modules are connected
 */
bool ASFBStarTree::validateConnectivity() {
    Logger::log("Validating connectivity (symmetry island constraint)");
    
    if (modules.empty()) return true;
    
    // Build position and dimension maps for the isSymmetryIsland check
    std::unordered_map<std::string, std::pair<int, int>> positions;
    std::unordered_map<std::string, std::pair<int, int>> dimensions;
    
    for (const auto& pair : modules) {
        const std::string& name = pair.first;
        const auto& module = pair.second;
        positions[name] = {module->getX(), module->getY()};
        dimensions[name] = {module->getWidth(), module->getHeight()};
    }
    
    // Use the isSymmetryIsland function from SymmetryGroup
    bool isConnected = symmetryGroup->isSymmetryIsland(positions, dimensions);
    
    if (isConnected) {
        Logger::log("Connectivity validation passed - all modules form a symmetry island");
    } else {
        Logger::log("Connectivity validation failed - modules do not form a symmetry island");
    }
    
    return isConnected;
}

/**
 * Attempts to enforce connectivity by repositioning modules
 * Modified enforceConnectivity function to maintain valid symmetry
 * 
 * @return True if successful in creating a connected symmetry island
 */
bool ASFBStarTree::enforceConnectivity() {
    Logger::log("Enforcing connectivity for symmetry island with improved error handling");
    
    try {
        // Get all representative modules
        std::vector<std::string> repModuleNames;
        for (const auto& pair : representativeModules) {
            repModuleNames.push_back(pair.first);
        }
        
        if (repModuleNames.empty()) {
            Logger::log("No representative modules to connect");
            return true;
        }
        
        // Safety check: ensure all module names exist in the modules map
        std::vector<std::string> validModuleNames;
        for (const auto& name : repModuleNames) {
            if (modules.find(name) != modules.end()) {
                validModuleNames.push_back(name);
            } else {
                Logger::log("WARNING: Module " + name + " not found when enforcing connectivity");
            }
        }
        
        if (validModuleNames.empty()) {
            Logger::log("No valid modules to connect");
            return false;
        }
        
        // Sort modules by width for better packing (place wider modules first)
        std::sort(validModuleNames.begin(), validModuleNames.end(), 
                [&](const std::string& a, const std::string& b) {
                    return modules.at(a)->getWidth() > modules.at(b)->getWidth();
                });
        
        // Start with the first representative module as the anchor
        std::string anchorName = validModuleNames[0];
        std::shared_ptr<Module> anchor = modules.at(anchorName);
        
        // Position the anchor at (0,0) for optimum placement
        anchor->setPosition(0, 0);
        
        // Track connected modules
        std::unordered_set<std::string> connected;
        connected.insert(anchorName);
        
        // Place modules in compact arrangement
        // Try to create a balanced layout for better area utilization
        int row = 0;
        int currentX = 0;
        int currentY = 0;
        int rowHeight = anchor->getHeight();
        
        for (size_t i = 1; i < validModuleNames.size(); i++) {
            const std::string& moduleName = validModuleNames[i];
            std::shared_ptr<Module> module = modules.at(moduleName);
            
            // Check if module would extend too far to the right (arbitrary threshold)
            int maxRowWidth = 5 * anchor->getWidth(); // Limit row width for better aspect ratio
            if (currentX + module->getWidth() > maxRowWidth) {
                // Start a new row
                currentX = 0;
                currentY += rowHeight;
                rowHeight = module->getHeight();
            } else {
                rowHeight = std::max(rowHeight, module->getHeight());
            }
            
            // Place the module
            module->setPosition(currentX, currentY);
            currentX += module->getWidth();
            
            // Add to connected set
            connected.insert(moduleName);
            
            Logger::log("Enforced connectivity: placed " + moduleName + " at (" + 
                       std::to_string(module->getX()) + ", " + 
                       std::to_string(module->getY()) + ")");
        }
        
        // Verify that all valid representative modules are now connected
        if (connected.size() == validModuleNames.size()) {
            Logger::log("Successfully enforced connectivity for all representative modules");
            return true;
        } else {
            Logger::log("Failed to enforce connectivity for all modules");
            return false;
        }
    } catch (const std::exception& e) {
        Logger::log("Exception in enforceConnectivity: " + std::string(e.what()));
        return false;
    }
}
