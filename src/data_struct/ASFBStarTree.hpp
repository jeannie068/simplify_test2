// ASFBStarTree.hpp
#pragma once

#include <memory>
#include <vector>
#include <map>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <string>
#include <random>
#include <iostream>

#include "Module.hpp"
#include "SymmetryConstraint.hpp"
#include "BStarTree.hpp"
#include "../Logger.hpp"

/**
 * @brief Automatically Symmetry-Feasible B*-tree (ASF-B*-tree)
 * 
 * This class implements a B*-tree variant that automatically enforces
 * symmetry constraints for a symmetry group. It represents a symmetry
 * island where all modules in the symmetry group form a connected
 * placement with proper symmetry relationships.
 */
class ASFBStarTree {
public:
    // Symmetry group represented by this ASF-B*-tree
    std::shared_ptr<SymmetryGroup> symmetryGroup;
    
    // Module map: module name -> module pointer
    std::map<std::string, std::shared_ptr<Module>> modules;
    
    // Map to store representative modules for each symmetry pair and self-symmetric module
    std::map<std::string, std::shared_ptr<Module>> representativeModules;
    
    // Map to store the relationship between representative and non-representative modules
    std::unordered_map<std::string, std::string> pairMap; // non-rep -> rep
    std::unordered_map<std::string, std::string> repToPairMap; // rep -> non-rep
    
    // Self-symmetric modules (center on symmetry axis)
    std::vector<std::string> selfSymmetricModules;
    
    // B*-tree representation (for representatives only)
    struct BStarNode {
        std::string moduleName;
        BStarNode* left;  // Left child: left-adjacent module
        BStarNode* right; // Right child: top-adjacent module
        
        BStarNode(const std::string& name) : moduleName(name), left(nullptr), right(nullptr) {}
    };
    
    
    // Root of the B*-tree
    BStarNode* root;

    // Backup storage for tree structure
    struct TreeBackup {
        std::vector<std::string> preorderTraversal;
        std::vector<std::string> inorderTraversal;
    };
    TreeBackup treeBackup;
    
    // Current symmetry axis position
    double symmetryAxisPosition;
    
    // Preorder and inorder traversals of the B*-tree
    std::vector<BStarNode*> preorderTraversal;
    std::vector<BStarNode*> inorderTraversal;
    
    // Contour data structure for packing
    struct ContourPoint {
        int x;
        int height;
        ContourPoint* next;
        
        ContourPoint(int x, int height) : x(x), height(height), next(nullptr) {}
    };
    
    ContourPoint* contourHead;
    
    /**
     * Clears the contour data structure
     */
    void clearContour() {
        while (contourHead != nullptr) {
            ContourPoint* temp = contourHead;
            contourHead = contourHead->next;
            delete temp;
        }
        contourHead = nullptr;
    }
    
    /**
     * Updates the contour after placing a module
     * 
     * @param x X-coordinate of the module
     * @param y Y-coordinate of the module
     * @param width Width of the module
     * @param height Height of the module
     */
    void updateContour(int x, int y, int width, int height) {
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
        
        // Update or insert points for the module
        if (curr == nullptr || curr->x > right) {
            // Module is beyond the current contour
            ContourPoint* newPoint = new ContourPoint(x, top);
            newPoint->next = new ContourPoint(right, prev ? prev->height : 0);
            newPoint->next->next = curr;
            
            if (prev) {
                prev->next = newPoint;
            } else {
                contourHead = newPoint;
            }
        } else {
            // Module intersects with existing contour
            if (curr->x > x) {
                // Insert a new point at the left edge of the module
                ContourPoint* newPoint = new ContourPoint(x, top);
                newPoint->next = curr;
                
                if (prev) {
                    prev->next = newPoint;
                } else {
                    contourHead = newPoint;
                }
                
                prev = newPoint;
            } else if (curr->x == x) {
                // Update the existing point
                curr->height = std::max(curr->height, top);
                prev = curr;
                curr = curr->next;
            }
            
            // Update or merge intermediate points
            while (curr != nullptr && curr->x <= right) {
                if (curr->x == right) {
                    curr->height = std::max(curr->height, top);
                    break;
                } else {
                    ContourPoint* temp = curr;
                    curr = curr->next;
                    delete temp;
                }
            }
            
            if (prev) {
                prev->next = curr;
            } else {
                contourHead = curr;
            }
            
            // If we reached the end without finding a point at 'right'
            if (curr == nullptr || curr->x > right) {
                ContourPoint* newPoint = new ContourPoint(right, prev ? prev->height : 0);
                newPoint->next = curr;
                
                if (prev) {
                    prev->next = newPoint;
                } else {
                    contourHead = newPoint;
                }
            }
        }
    }
    
    /**
     * Gets the height of the contour at a given x-coordinate
     * 
     * @param x X-coordinate
     * @return The height of the contour at x
     */
    int getContourHeight(int x) {
        if (contourHead == nullptr) return 0;
        
        ContourPoint* curr = contourHead;
        while (curr->next != nullptr && curr->next->x <= x) {
            curr = curr->next;
        }
        
        return curr->height;
    }
    
    /**
     * Performs a preorder traversal of the B*-tree
     * 
     * @param node The current node
     */
    void preorder(BStarNode* node) {
        if (node == nullptr) return;
        
        preorderTraversal.push_back(node);
        preorder(node->left);
        preorder(node->right);
    }
    
    /**
     * Performs an inorder traversal of the B*-tree
     * 
     * @param node The current node
     */
    void inorder(BStarNode* node) {
        if (node == nullptr) return;
        
        inorder(node->left);
        inorderTraversal.push_back(node);
        inorder(node->right);
    }
    
    /**
     * Packs the B*-tree to get the coordinates of all modules
     */
    void packBStarTree() {
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
        
        // Count nodes in the tree to verify tree size
        int nodeCount = 0;
        std::function<void(BStarNode*)> countNodes = [&](BStarNode* node) {
            if (node == nullptr) return;
            nodeCount++;
            countNodes(node->left);
            countNodes(node->right);
        };
        countNodes(root);
        
        Logger::log("Total nodes in tree: " + std::to_string(nodeCount) + ", Expected nodes: " + std::to_string(representativeModules.size()));
        
        // Traverse the tree in preorder
        std::vector<BStarNode*> nodeStack;
        if (root != nullptr) {
            nodeStack.push_back(root);
        }
        
        int processedNodes = 0;
        
        try {
            while (!nodeStack.empty()) {
                BStarNode* node = nodeStack.back();
                nodeStack.pop_back();
                processedNodes++;
                
                Logger::log("Processing node: " + node->moduleName);
                
                std::shared_ptr<Module> module = modules[node->moduleName];
                
                // Determine x-coordinate from the parent-child relationship
                int x = 0;
                if (node != root) {
                    bool found = false;
                    
                    Logger::log("Looking for parent of " + node->moduleName + " in stack of size " + std::to_string(nodeStack.size()));
                    
                    for (BStarNode* parent : nodeStack) {
                        if (parent->left == node) {
                            // Left child: placed to the right of parent
                            std::shared_ptr<Module> parentModule = modules[parent->moduleName];
                            x = parentModule->getX() + parentModule->getWidth();
                            Logger::log("Found parent: " + parent->moduleName + " (node is left child)");
                            found = true;
                            break;
                        } else if (parent->right == node) {
                            // Right child: same x-coordinate as parent
                            std::shared_ptr<Module> parentModule = modules[parent->moduleName];
                            x = parentModule->getX();
                            Logger::log("Found parent: " + parent->moduleName + " (node is right child)");
                            found = true;
                            break;
                        }
                    }
                    
                    if (!found) {
                        // Detailed diagnostic information
                        Logger::log("ERROR: Cannot find parent for node " + node->moduleName);
                        Logger::log("Stack contents:");
                        for (BStarNode* stackNode : nodeStack) {
                            Logger::log("- " + stackNode->moduleName);
                            if (stackNode->left) Logger::log("  Left child: " + stackNode->left->moduleName);
                            if (stackNode->right) Logger::log("  Right child: " + stackNode->right->moduleName);
                        }
                        
                        // Verify the tree structure integrity
                        Logger::log("Verifying tree structure integrity...");
                        std::unordered_set<BStarNode*> allNodes;
                        std::function<void(BStarNode*)> collectNodes = [&](BStarNode* n) {
                            if (n == nullptr) return;
                            allNodes.insert(n);
                            collectNodes(n->left);
                            collectNodes(n->right);
                        };
                        collectNodes(root);
                        
                        bool nodeFound = false;
                        for (BStarNode* n : allNodes) {
                            if (n->left == node || n->right == node) {
                                nodeFound = true;
                                Logger::log("Node found as child of " + n->moduleName + 
                                        (n->left == node ? " (left)" : " (right)"));
                                
                                // This should not happen - the node has a parent but wasn't found in the traversal
                                throw std::runtime_error("Node has parent but wasn't found in traversal - possible cycle in tree");
                            }
                        }
                        
                        if (!nodeFound) {
                            Logger::log("Node is not a child of any node in the tree - orphaned node detected");
                        }
                        
                        throw std::runtime_error("Invalid B*-tree structure: Parent not found for node " + node->moduleName);
                    }
                } else {
                    Logger::log("Root node, starting at x=0");
                }
                
                // Determine y-coordinate from the contour
                int y = getContourHeight(x);
                
                // Update the module's position
                module->setPosition(x, y);
                Logger::log("Placed " + node->moduleName + " at (" + std::to_string(x) + ", " + std::to_string(y) + ")");
                
                // Update the contour
                updateContour(x, y, module->getWidth(), module->getHeight());
                
                // Add children to the stack (right child first for proper DFS order)
                if (node->right != nullptr) {
                    Logger::log("Adding right child " + node->right->moduleName + " to stack");
                    nodeStack.push_back(node->right);
                }
                if (node->left != nullptr) {
                    Logger::log("Adding left child " + node->left->moduleName + " to stack");
                    nodeStack.push_back(node->left);
                }
            }
            
            Logger::log("Successfully processed " + std::to_string(processedNodes) + " nodes");
            
        } catch (const std::exception& e) {
            Logger::log("Exception during packing: " + std::string(e.what()));
            throw; // Re-throw the exception after logging
        }
    }
    
    /**
     * Cleans up the B*-tree recursively
     * 
     * @param node The current node
     */
    void cleanupTree(BStarNode* node) {
        if (node == nullptr) return;
        
        cleanupTree(node->left);
        cleanupTree(node->right);
        delete node;
    }
    
    /**
     * Checks if a node represents a self-symmetric module
     * 
     * @param nodeName Name of the module
     * @return True if the module is self-symmetric
     */
    bool isSelfSymmetric(const std::string& nodeName) const {
        return std::find(selfSymmetricModules.begin(), selfSymmetricModules.end(), nodeName) != selfSymmetricModules.end();
    }
    
    /**
     * Updates the positions of symmetric modules based on their representatives
     */
    void updateSymmetricModulePositions() {
        // Calculate symmetry axis position if not set
        if (symmetryAxisPosition < 0) {
            calculateSymmetryAxisPosition();
        }
        
        // Update positions for symmetry pairs
        for (const auto& pair : repToPairMap) {
            const std::string& repName = pair.first;
            const std::string& symName = pair.second;
            
            std::shared_ptr<Module> repModule = modules[repName];
            std::shared_ptr<Module> symModule = modules[symName];
            
            if (symmetryGroup->getType() == SymmetryType::VERTICAL) {
                // For vertical symmetry:
                // - x-coordinate is mirrored around the axis
                // - y-coordinate is the same
                int newX = 2 * static_cast<int>(symmetryAxisPosition) - (repModule->getX() + repModule->getWidth());
                symModule->setPosition(newX, repModule->getY());
            } else {
                // For horizontal symmetry:
                // - x-coordinate is the same
                // - y-coordinate is mirrored around the axis
                int newY = 2 * static_cast<int>(symmetryAxisPosition) - (repModule->getY() + repModule->getHeight());
                symModule->setPosition(repModule->getX(), newY);
            }
            
            // Ensure the rotation is consistent for the symmetry pair
            symModule->setRotation(repModule->getRotated());
        }
        
        // Update positions for self-symmetric modules
        for (const std::string& moduleName : selfSymmetricModules) {
            std::shared_ptr<Module> module = modules[moduleName];
            
            if (symmetryGroup->getType() == SymmetryType::VERTICAL) {
                // For vertical symmetry, center the module on the axis
                int width = module->getWidth();
                int newX = static_cast<int>(symmetryAxisPosition) - width / 2;
                module->setPosition(newX, module->getY());
            } else {
                // For horizontal symmetry, center the module on the axis
                int height = module->getHeight();
                int newY = static_cast<int>(symmetryAxisPosition) - height / 2;
                module->setPosition(module->getX(), newY);
            }
        }
    }
    
    /**
     * Calculates the position of the symmetry axis based on the current placement
     */
    void calculateSymmetryAxisPosition() {
        double sum = 0.0;
        int count = 0;
        
        // Use symmetry pairs to determine the axis
        for (const auto& pair : repToPairMap) {
            const std::string& repName = pair.first;
            const std::string& symName = pair.second;
            
            std::shared_ptr<Module> repModule = modules[repName];
            std::shared_ptr<Module> symModule = modules[symName];
            
            if (symmetryGroup->getType() == SymmetryType::VERTICAL) {
                // For vertical symmetry, axis is at the middle x-coordinate
                double repCenterX = repModule->getX() + repModule->getWidth() / 2.0;
                double symCenterX = symModule->getX() + symModule->getWidth() / 2.0;
                sum += (repCenterX + symCenterX) / 2.0;
            } else {
                // For horizontal symmetry, axis is at the middle y-coordinate
                double repCenterY = repModule->getY() + repModule->getHeight() / 2.0;
                double symCenterY = symModule->getY() + symModule->getHeight() / 2.0;
                sum += (repCenterY + symCenterY) / 2.0;
            }
            count++;
        }
        
        // If no symmetry pairs, use self-symmetric modules
        if (count == 0 && !selfSymmetricModules.empty()) {
            for (const std::string& moduleName : selfSymmetricModules) {
                std::shared_ptr<Module> module = modules[moduleName];
                
                if (symmetryGroup->getType() == SymmetryType::VERTICAL) {
                    sum += module->getX() + module->getWidth() / 2.0;
                } else {
                    sum += module->getY() + module->getHeight() / 2.0;
                }
                count++;
            }
        }
        
        // Update the symmetry axis position
        if (count > 0) {
            symmetryAxisPosition = sum / count;
            symmetryGroup->setAxisPosition(symmetryAxisPosition);
        } else {
            // Default to 0 if no modules
            symmetryAxisPosition = 0;
            symmetryGroup->setAxisPosition(0);
        }
    }
    
    /**
     * Builds a random initial B*-tree for the symmetry group
     * This ensures self-symmetric modules are on the left-most (vertical) or top-most (horizontal) branch
     */
    void buildInitialBStarTree() {
        Logger::log("Building initial ASF-B*-tree");
        
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
        
        std::random_device rd;
        std::mt19937 g(rd());
        std::shuffle(nonSelfSymModules.begin(), nonSelfSymModules.end(), g);
        
        // Create nodes for all modules
        std::unordered_map<std::string, BStarNode*> nodeMap;
        for (const auto& name : repModuleNames) {
            nodeMap[name] = new BStarNode(name);
            Logger::log("Created node for module: " + name);
        }
        
        // Build a tree that ensures self-symmetric modules are on the proper branch
        if (!repModuleNames.empty()) {
            // Start with a module (preferably self-symmetric if any)
            std::string rootName;
            if (!selfSymModules.empty()) {
                rootName = selfSymModules.front();
                selfSymModules.erase(selfSymModules.begin());
                Logger::log("Using self-symmetric module as root: " + rootName);
            } else if (!nonSelfSymModules.empty()) {
                rootName = nonSelfSymModules.front();
                nonSelfSymModules.erase(nonSelfSymModules.begin());
                Logger::log("Using non-self-symmetric module as root: " + rootName);
            } else {
                // Should not happen
                Logger::log("ERROR: No modules to place in symmetry group");
                throw std::runtime_error("No modules to place in symmetry group");
            }
            
            root = nodeMap[rootName];
            
            // If there are self-symmetric modules, place them on the rightmost branch (vertical) or leftmost branch (horizontal)
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
            
            // Place remaining non-self-symmetric modules with proper insertion logic
            for (const auto& name : nonSelfSymModules) {
                // Randomly select an existing node to be the parent
                std::vector<BStarNode*> potentialParents;
                std::function<void(BStarNode*)> collectNodes = [&](BStarNode* node) {
                    if (node == nullptr) return;
                    potentialParents.push_back(node);
                    collectNodes(node->left);
                    collectNodes(node->right);
                };
                
                collectNodes(root);
                
                // Try to find a parent with an available child slot
                bool placed = false;
                // Shuffle potential parents to avoid bias
                std::shuffle(potentialParents.begin(), potentialParents.end(), g);
                
                Logger::log("Trying to place module " + name + " with " + std::to_string(potentialParents.size()) + " potential parents");
                
                for (BStarNode* parent : potentialParents) {
                    // Try left child first if it's empty
                    if (parent->left == nullptr) {
                        Logger::log("Placing module " + name + " as left child of " + parent->moduleName);
                        parent->left = nodeMap[name];
                        placed = true;
                        break;
                    }
                    // Try right child if it's empty
                    else if (parent->right == nullptr) {
                        Logger::log("Placing module " + name + " as right child of " + parent->moduleName);
                        parent->right = nodeMap[name];
                        placed = true;
                        break;
                    }
                }
                
                // If we couldn't place the node (all parents have both children),
                // create a new level by adding to a leaf node
                if (!placed) {
                    Logger::log("Couldn't find parent with empty child slot, looking for leaf nodes");
                    
                    // Find leaf nodes (nodes with at least one nullptr child)
                    std::vector<BStarNode*> leafNodes;
                    for (BStarNode* parent : potentialParents) {
                        if (parent->left == nullptr || parent->right == nullptr) {
                            leafNodes.push_back(parent);
                        }
                    }
                    
                    if (!leafNodes.empty()) {
                        // Select a random leaf node
                        BStarNode* leafNode = leafNodes[std::rand() % leafNodes.size()];
                        
                        // Add to whichever child pointer is nullptr
                        if (leafNode->left == nullptr) {
                            Logger::log("Placing module " + name + " as left child of leaf node " + leafNode->moduleName);
                            leafNode->left = nodeMap[name];
                            placed = true;
                        } else { // right must be nullptr
                            Logger::log("Placing module " + name + " as right child of leaf node " + leafNode->moduleName);
                            leafNode->right = nodeMap[name];
                            placed = true;
                        }
                    } else {
                        // This should not happen in theory, but provide a fallback just in case
                        Logger::log("WARNING: No leaf nodes found, adding to root!");
                        
                        // Add as new right child of first potential parent
                        if (potentialParents[0]->right == nullptr) {
                            potentialParents[0]->right = nodeMap[name];
                        } else {
                            // CRITICAL: We must not create cycles or lose nodes
                            Logger::log("CRITICAL: Cannot place module " + name + " without overwriting existing node!");
                            
                            // Create special case handling - add as a child of the rightmost node
                            BStarNode* rightmost = root;
                            while (rightmost->right != nullptr) {
                                rightmost = rightmost->right;
                            }
                            
                            Logger::log("Adding as left child of rightmost node " + rightmost->moduleName);
                            rightmost->left = nodeMap[name];
                        }
                        placed = true;
                    }
                }
                
                if (!placed) {
                    Logger::log("CRITICAL: Failed to place module " + name);
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
    }

    bool validateTreeStructure(BStarNode* node) {
        if (node == nullptr) return true;
        
        Logger::log("Validating tree structure starting at " + (node == root ? "root" : node->moduleName));
        
        // Set to keep track of visited nodes
        std::unordered_set<BStarNode*> visited;
        
        // Function to check for cycles in the tree
        std::function<bool(BStarNode*, std::unordered_set<BStarNode*>&, std::string)> hasNoCycles =
            [&](BStarNode* current, std::unordered_set<BStarNode*>& path, std::string pathStr) -> bool {
                if (current == nullptr) return true;
                
                // Log the current path
                std::string currentPathStr = pathStr + " -> " + current->moduleName;
                
                // If we've seen this node in the current path, we have a cycle
                if (path.find(current) != path.end()) {
                    Logger::log("CYCLE DETECTED: " + currentPathStr);
                    return false;
                }
                
                // Add this node to the current path
                path.insert(current);
                visited.insert(current);
                
                // Check children
                bool leftValid = hasNoCycles(current->left, path, currentPathStr);
                bool rightValid = hasNoCycles(current->right, path, currentPathStr);
                
                // Remove this node from the current path (backtracking)
                path.erase(current);
                
                return leftValid && rightValid;
            };
        
        // Start DFS from the root to check for cycles
        std::unordered_set<BStarNode*> path;
        bool noCycles = hasNoCycles(node, path, "START");
        
        if (!noCycles) {
            Logger::log("Tree has cycles!");
            return false;
        }
        
        // Make sure each node in the tree is reachable from the root
        // First count all nodes in the tree
        size_t totalNodes = 0;
        std::function<void(BStarNode*)> countNodes = [&](BStarNode* n) {
            if (n == nullptr) return;
            totalNodes++;
            countNodes(n->left);
            countNodes(n->right);
        };
        countNodes(node);
        
        Logger::log("Total nodes in tree: " + std::to_string(totalNodes));
        Logger::log("Visited nodes during cycle check: " + std::to_string(visited.size()));
        
        // Make sure we visited all nodes in the cycle check
        if (visited.size() != totalNodes) {
            Logger::log("Some nodes are unreachable from the root!");
            return false;
        }
        
        // Also verify that all nodes in the tree are actually in our modules map
        std::function<bool(BStarNode*)> verifyModuleExists = [&](BStarNode* n) -> bool {
            if (n == nullptr) return true;
            
            if (modules.find(n->moduleName) == modules.end()) {
                Logger::log("Node " + n->moduleName + " doesn't exist in modules map!");
                return false;
            }
            
            return verifyModuleExists(n->left) && verifyModuleExists(n->right);
        };
        
        bool allModulesExist = verifyModuleExists(node);
        if (!allModulesExist) {
            Logger::log("Some nodes reference non-existent modules!");
            return false;
        }
        
        Logger::log("Tree structure is valid");
        return true;
    }

    // New methods for backing up and restoring tree structure
    // Backup the current tree structure
    void backupTreeStructure() {
        // Store the current tree structure to enable complete restoration
        treeBackup.preorderTraversal.clear();
        treeBackup.inorderTraversal.clear();
        
        // Populate the backup traversals
        std::function<void(BStarNode*)> preorderBackup = [&](BStarNode* node) {
            if (node == nullptr) return;
            treeBackup.preorderTraversal.push_back(node->moduleName);
            preorderBackup(node->left);
            preorderBackup(node->right);
        };
        
        std::function<void(BStarNode*)> inorderBackup = [&](BStarNode* node) {
            if (node == nullptr) return;
            inorderBackup(node->left);
            treeBackup.inorderTraversal.push_back(node->moduleName);
            inorderBackup(node->right);
        };
        
        preorderBackup(root);
        inorderBackup(root);
    }

    // Restore from the backed up tree structure
    void restoreTreeStructure() {
        if (treeBackup.preorderTraversal.empty() || treeBackup.inorderTraversal.empty()) {
            // No backup available
            return;
        }
        
        // Clean up existing tree
        cleanupTree(root);
        root = nullptr;
        
        // Create nodes for all modules
        std::unordered_map<std::string, BStarNode*> nodeMap;
        for (const std::string& name : treeBackup.preorderTraversal) {
            if (nodeMap.find(name) == nodeMap.end()) {
                nodeMap[name] = new BStarNode(name);
            }
        }
        
        // Create mapping from module name to inorder index
        std::unordered_map<std::string, size_t> inorderMap;
        for (size_t i = 0; i < treeBackup.inorderTraversal.size(); i++) {
            inorderMap[treeBackup.inorderTraversal[i]] = i;
        }
        
        // Recursive function to rebuild the tree
        std::function<BStarNode*(size_t&, size_t, size_t)> rebuildTree = 
            [&](size_t& preIdx, size_t inStart, size_t inEnd) -> BStarNode* {
                if (inStart > inEnd || preIdx >= treeBackup.preorderTraversal.size()) {
                    return nullptr;
                }
                
                // Get the root of current subtree from preorder traversal
                std::string moduleName = treeBackup.preorderTraversal[preIdx++];
                BStarNode* node = nodeMap[moduleName];
                
                // If this is the only element in this subtree
                if (inStart == inEnd) {
                    return node;
                }
                
                // Find the index of current node in inorder traversal
                size_t inIndex = inorderMap[moduleName];
                
                // Recursively build left and right subtrees
                if (inIndex > inStart) {
                    node->left = rebuildTree(preIdx, inStart, inIndex - 1);
                }
                if (inIndex < inEnd) {
                    node->right = rebuildTree(preIdx, inIndex + 1, inEnd);
                }
                
                return node;
            };
        
        // Rebuild the tree
        size_t preIdx = 0;
        root = rebuildTree(preIdx, 0, treeBackup.inorderTraversal.size() - 1);
        
        // Validate the restored structure
        if (!validateTreeStructure(root)) {
            throw std::runtime_error("Invalid tree structure after restoration");
        }
    }

    /**
     * Constructor
     * 
     * @param symmetryGroup The symmetry group to represent
     * @param modules Map of all modules
     */
    ASFBStarTree(std::shared_ptr<SymmetryGroup> symmetryGroup, const std::map<std::string, std::shared_ptr<Module>>& modules)
        : symmetryGroup(symmetryGroup), modules(modules), root(nullptr), contourHead(nullptr), symmetryAxisPosition(-1) {
        
        // Initialize module relationships
        initializeRepresentatives();
        
        // Build an initial B*-tree
        buildInitialBStarTree();
    }
    
    /**
     * Destructor
     */
    ~ASFBStarTree() {
        cleanupTree(root);
        clearContour();
    }
    
    /**
     * Initializes representative modules for the symmetry group
     */
    void initializeRepresentatives() {
        // Clear existing data
        representativeModules.clear();
        pairMap.clear();
        repToPairMap.clear();
        selfSymmetricModules.clear();
        
        // Process symmetry pairs
        for (const auto& pair : symmetryGroup->getSymmetryPairs()) {
            const std::string& module1 = pair.first;
            const std::string& module2 = pair.second;
            
            // Choose one module as the representative
            const std::string& rep = module2; // As defined in the paper
            const std::string& sym = module1;
            
            representativeModules[rep] = modules[rep];
            pairMap[sym] = rep;
            repToPairMap[rep] = sym;
        }
        
        // Process self-symmetric modules
        for (const auto& moduleName : symmetryGroup->getSelfSymmetric()) {
            selfSymmetricModules.push_back(moduleName);
            representativeModules[moduleName] = modules[moduleName];
        }
    }
    
    /**
     * Packs the ASF-B*-tree to get the coordinates of all modules
     * 
     * @return True if packing was successful
     */
    bool pack() {
        try {
            // Update the traversals for the current B*-tree
            preorderTraversal.clear();
            inorderTraversal.clear();
            preorder(root);
            inorder(root);
            
            // Pack the B*-tree to get coordinates for representatives
            packBStarTree();
            
            // Calculate the symmetry axis position
            calculateSymmetryAxisPosition();
            
            // Update positions of symmetric modules
            updateSymmetricModulePositions();
            
            return true;
        } catch (const std::exception& e) {
            std::cerr << "Error packing ASF-B*-tree: " << e.what() << std::endl;
            return false;
        }
    }
    
    /**
     * Gets the symmetry axis position
     * 
     * @return The current symmetry axis position
     */
    double getSymmetryAxisPosition() const {
        return symmetryAxisPosition;
    }
    
    /**
     * Sets the symmetry axis position
     * 
     * @param position The new symmetry axis position
     */
    void setSymmetryAxisPosition(double position) {
        symmetryAxisPosition = position;
        symmetryGroup->setAxisPosition(position);
    }
    
    /**
     * Gets the symmetry group
     * 
     * @return The symmetry group
     */
    std::shared_ptr<SymmetryGroup> getSymmetryGroup() const {
        return symmetryGroup;
    }
    
    /**
     * Gets all modules in the symmetry group
     * 
     * @return Map of module names to module pointers
     */
    const std::map<std::string, std::shared_ptr<Module>>& getModules() const {
        return modules;
    }
    
    /**
     * Gets the bounding rectangle of the symmetry island
     * 
     * @return Pair of (width, height)
     */
    std::pair<int, int> getBoundingBox() const {
        int minX = std::numeric_limits<int>::max();
        int minY = std::numeric_limits<int>::max();
        int maxX = std::numeric_limits<int>::min();
        int maxY = std::numeric_limits<int>::min();
        
        for (const auto& pair : modules) {
            const auto& module = pair.second;
            minX = std::min(minX, module->getX());
            minY = std::min(minY, module->getY());
            maxX = std::max(maxX, module->getX() + module->getWidth());
            maxY = std::max(maxY, module->getY() + module->getHeight());
        }
        
        return {maxX - minX, maxY - minY};
    }
    
    /**
     * Gets the area of the symmetry island
     * 
     * @return The area of the bounding rectangle
     */
    int getArea() const {
        auto [width, height] = getBoundingBox();
        return width * height;
    }
    
    /**
     * Validates the symmetric placement
     * 
     * @return True if the placement satisfies all symmetry constraints
     */
    bool validateSymmetry() const {
        return symmetryGroup->validateSymmetricPlacement(
            [this]() {
                std::unordered_map<std::string, std::pair<int, int>> positions;
                std::unordered_map<std::string, std::pair<int, int>> dimensions;
                
                for (const auto& pair : modules) {
                    const std::string& name = pair.first;
                    const auto& module = pair.second;
                    positions[name] = {module->getX(), module->getY()};
                    dimensions[name] = {module->getWidth(), module->getHeight()};
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
    
    /**
     * Rotates all modules in the symmetry group
     * 
     * @param rotate True to rotate, false to revert to original orientation
     */
    void rotateAll(bool rotate) {
        // Rotate all modules
        for (auto& pair : modules) {
            pair.second->setRotation(rotate);
        }
        
        // Rebuild the B*-tree to adjust to new dimensions
        buildInitialBStarTree();
        
        // Re-pack to update positions
        pack();
    }
    
    /**
     * Rotates a specific module and its symmetric pair
     * 
     * @param moduleName Name of the module to rotate
     */
    void rotateModule(const std::string& moduleName) {
        // Check if the module is in this symmetry group
        if (modules.find(moduleName) == modules.end()) {
            return;
        }
        
        // Get the module to rotate
        std::shared_ptr<Module> module = modules[moduleName];
        
        // Rotate the module
        module->rotate();
        
        // If it's part of a symmetry pair, rotate the other module too
        if (repToPairMap.find(moduleName) != repToPairMap.end()) {
            modules[repToPairMap[moduleName]]->rotate();
        } else if (pairMap.find(moduleName) != pairMap.end()) {
            modules[pairMap[moduleName]]->rotate();
        }
        
        // Rebuild the B*-tree to adjust to new dimensions
        buildInitialBStarTree();
        
        // Re-pack to update positions
        pack();
    }
    
    /**
     * Changes the representative module for a symmetry pair
     * 
     * @param moduleName Name of the module in the pair
     * @return True if the representative was successfully changed
     */
    bool changeRepresentative(const std::string& moduleName) {
        // Check if the module is in this symmetry group
        if (modules.find(moduleName) == modules.end()) {
            return false;
        }
        
        // Check if it's part of a symmetry pair
        if (repToPairMap.find(moduleName) != repToPairMap.end()) {
            // Current rep -> non-rep
            std::string rep = moduleName;
            std::string nonRep = repToPairMap[rep];
            
            // Swap the representative
            representativeModules.erase(rep);
            representativeModules[nonRep] = modules[nonRep];
            
            repToPairMap.erase(rep);
            repToPairMap[nonRep] = rep;
            
            pairMap.erase(nonRep);
            pairMap[rep] = nonRep;
            
            // Rebuild the B*-tree with the new representative
            buildInitialBStarTree();
            
            // Re-pack to update positions
            pack();
            
            return true;
        } else if (pairMap.find(moduleName) != pairMap.end()) {
            // Current non-rep -> rep
            std::string nonRep = moduleName;
            std::string rep = pairMap[nonRep];
            
            // Swap the representative
            representativeModules.erase(rep);
            representativeModules[nonRep] = modules[nonRep];
            
            repToPairMap.erase(rep);
            repToPairMap[nonRep] = rep;
            
            pairMap.erase(nonRep);
            pairMap[rep] = nonRep;
            
            // Rebuild the B*-tree with the new representative
            buildInitialBStarTree();
            
            // Re-pack to update positions
            pack();
            
            return true;
        }
        
        // Not part of a symmetry pair
        return false;
    }
    
    /**
     * Converts the symmetry type between vertical and horizontal
     * 
     * @return True if the conversion was successful
     */
    bool convertSymmetryType() {
        // Toggle the symmetry type
        if (symmetryGroup->getType() == SymmetryType::VERTICAL) {
            symmetryGroup->setType(SymmetryType::HORIZONTAL);
        } else {
            symmetryGroup->setType(SymmetryType::VERTICAL);
        }
        
        // Rebuild the B*-tree to adjust to the new symmetry type
        buildInitialBStarTree();
        
        // Re-pack to update positions
        pack();
        
        return true;
    }
    
    /**
     * Performs a random perturbation on the B*-tree
     * 
     * @param perturbationType 0: rotate, 1: move, 2: swap, 3: change representative, 4: convert symmetry type
     * @return True if perturbation was successful
     */
    bool perturb(int perturbationType) {
        Logger::log("ASF-B*-tree perturbation type " + std::to_string(perturbationType));
        
        // Backup tree structure before perturbation
        backupTreeStructure();
        
        bool success = false;
        
        try {
            switch (perturbationType) {
                case 0: // Rotate
                    {
                        // Rotate a random module
                        std::vector<std::string> moduleNames;
                        for (const auto& pair : modules) {
                            moduleNames.push_back(pair.first);
                        }
                        
                        if (!moduleNames.empty()) {
                            std::string randomModule = moduleNames[std::rand() % moduleNames.size()];
                            Logger::log("Rotating module " + randomModule);
                            rotateModule(randomModule);
                            success = true;
                        }
                    }
                    break;
                    
                case 1: // Move (rebuild tree with different topology)
                    {
                        Logger::log("Rebuilding tree with different topology");
                        // Just rebuild the tree randomly
                        buildInitialBStarTree();
                        pack();
                        success = true;
                    }
                    break;
                    
                case 2: // Swap
                    {
                        // Get all representative modules
                        std::vector<std::string> repModules;
                        for (const auto& pair : representativeModules) {
                            if (!isSelfSymmetric(pair.first)) {
                                repModules.push_back(pair.first);
                            }
                        }
                        
                        // Need at least 2 non-self-symmetric modules to swap
                        if (repModules.size() >= 2) {
                            // Select two random modules
                            int idx1 = std::rand() % repModules.size();
                            int idx2;
                            do {
                                idx2 = std::rand() % repModules.size();
                            } while (idx1 == idx2);
                            
                            Logger::log("Swapping " + repModules[idx1] + " and " + repModules[idx2]);
                            
                            // Locate the nodes in the B*-tree
                            BStarNode* node1 = nullptr;
                            BStarNode* node2 = nullptr;
                            
                            std::function<void(BStarNode*)> findNodes = [&](BStarNode* node) {
                                if (node == nullptr) return;
                                
                                if (node->moduleName == repModules[idx1]) {
                                    node1 = node;
                                } else if (node->moduleName == repModules[idx2]) {
                                    node2 = node;
                                }
                                
                                findNodes(node->left);
                                findNodes(node->right);
                            };
                            
                            findNodes(root);
                            
                            if (node1 != nullptr && node2 != nullptr) {
                                Logger::log("Found both nodes, swapping module names");
                                // Swap the module names
                                std::swap(node1->moduleName, node2->moduleName);
                                
                                // Re-pack to update positions
                                Logger::log("Re-packing after swap");
                                pack();
                                success = true;
                            } else {
                                Logger::log("Failed to find one or both nodes in the tree");
                            }
                        }
                    }
                    break;
                    
                case 3: // Change representative
                    {
                        // Get all symmetry pairs
                        std::vector<std::string> symPairs;
                        for (const auto& pair : repToPairMap) {
                            symPairs.push_back(pair.first);
                        }
                        
                        if (!symPairs.empty()) {
                            // Select a random symmetry pair
                            std::string randomRep = symPairs[std::rand() % symPairs.size()];
                            Logger::log("Changing representative for " + randomRep);
                            success = changeRepresentative(randomRep);
                        }
                    }
                    break;
                    
                case 4: // Convert symmetry type
                    {
                        Logger::log("Converting symmetry type");
                        success = convertSymmetryType();
                    }
                    break;
            }
        } catch (const std::exception& e) {
            Logger::log("Exception during perturbation: " + std::string(e.what()));
            
            // Attempt to restore the tree structure
            Logger::log("Attempting to restore tree structure after exception");
            restoreTreeStructure();
            return false;
        }
        
        // Validate tree structure after perturbation
        if (success) {
            Logger::log("Validating tree structure after perturbation");
            if (!validateTreeStructure(root)) {
                Logger::log("Invalid tree structure after perturbation, restoring");
                restoreTreeStructure();
                success = false;
            } else {
                Logger::log("Tree structure valid after perturbation");
            }
        }
        
        return success;
    }
};
