#include "solver.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

// Constructor
PlacementSolver::PlacementSolver()
    : bstarRoot(nullptr), contourHead(nullptr),
      solutionArea(0), solutionWirelength(0),
      bestSolutionArea(std::numeric_limits<int>::max()), bestSolutionWirelength(0),
      initialTemperature(1000.0), finalTemperature(0.1),
      coolingRate(0.95), iterationsPerTemperature(100), noImprovementLimit(1000),
      rotateProb(0.3), moveProb(0.3), swapProb(0.3),
      changeRepProb(0.05), convertSymProb(0.05),
      areaWeight(1.0), wirelengthWeight(0.0),
      timeLimit(300) {
    
    // Initialize random number generator
    std::random_device rd;
    rng = std::mt19937(rd());
}

// Destructor
PlacementSolver::~PlacementSolver() {
    cleanupBStarTree(bstarRoot);
    clearContour();
}

// Cleanup B*-tree
void PlacementSolver::cleanupBStarTree(BStarNode* node) {
    if (node == nullptr) return;
    
    cleanupBStarTree(node->left);
    cleanupBStarTree(node->right);
    delete node;
}

// Clear contour data structure
void PlacementSolver::clearContour() {
    while (contourHead != nullptr) {
        ContourPoint* temp = contourHead;
        contourHead = contourHead->next;
        delete temp;
    }
    contourHead = nullptr;
}

// Update contour after placing a module
void PlacementSolver::updateContour(int x, int y, int width, int height) {
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

// Get height of contour at x-coordinate
int PlacementSolver::getContourHeight(int x) {
    if (contourHead == nullptr) return 0;
    
    ContourPoint* curr = contourHead;
    while (curr->next != nullptr && curr->next->x <= x) {
        curr = curr->next;
    }
    
    return curr->height;
}

// Build initial B*-tree for global placement
void PlacementSolver::buildInitialBStarTree() {
    // Clean up any existing tree
    cleanupBStarTree(bstarRoot);
    bstarRoot = nullptr;
    
    // Get all module and island names
    std::vector<std::string> entities;
    
    // Add symmetry islands
    for (size_t i = 0; i < symmetryIslands.size(); i++) {
        entities.push_back("island_" + std::to_string(i));
    }
    
    // Add regular modules
    for (const auto& pair : regularModules) {
        entities.push_back(pair.first);
    }
    
    // Shuffle the entities for randomness
    std::shuffle(entities.begin(), entities.end(), rng);
    
    // Create nodes for all entities
    std::unordered_map<std::string, BStarNode*> nodeMap;
    for (const auto& name : entities) {
        bool isIsland = (name.substr(0, 6) == "island_");
        nodeMap[name] = new BStarNode(name, isIsland);
    }
    
    // Build a random tree
    if (!entities.empty()) {
        // Start with a random entity as the root
        std::string rootName = entities[0];
        bstarRoot = nodeMap[rootName];
        
        // Place remaining entities randomly
        for (size_t i = 1; i < entities.size(); i++) {
            // Randomly select an existing node to be the parent
            std::vector<BStarNode*> potentialParents;
            std::function<void(BStarNode*)> collectNodes = [&](BStarNode* node) {
                if (node == nullptr) return;
                potentialParents.push_back(node);
                collectNodes(node->left);
                collectNodes(node->right);
            };
            
            collectNodes(bstarRoot);
            
            BStarNode* parent = potentialParents[std::rand() % potentialParents.size()];
            
            // Randomly choose left or right child
            if (std::rand() % 2 == 0 && parent->left == nullptr) {
                parent->left = nodeMap[entities[i]];
            } else if (parent->right == nullptr) {
                parent->right = nodeMap[entities[i]];
            } else {
                // Both children occupied, try left
                parent->left = nodeMap[entities[i]];
            }
        }
    }
}

// Preorder traversal of B*-tree
void PlacementSolver::preorder(BStarNode* node) {
    if (node == nullptr) return;
    
    preorderTraversal.push_back(node);
    preorder(node->left);
    preorder(node->right);
}

// Inorder traversal of B*-tree
void PlacementSolver::inorder(BStarNode* node) {
    if (node == nullptr) return;
    
    inorder(node->left);
    inorderTraversal.push_back(node);
    inorder(node->right);
}

// Pack B*-tree to get coordinates
void PlacementSolver::packBStarTree() {
    // Clear the contour
    clearContour();
    
    // Update the traversals for the current B*-tree
    preorderTraversal.clear();
    inorderTraversal.clear();
    preorder(bstarRoot);
    inorder(bstarRoot);
    
    // Traverse the tree in preorder
    std::vector<BStarNode*> nodeStack;
    if (bstarRoot != nullptr) {
        nodeStack.push_back(bstarRoot);
    }
    
    while (!nodeStack.empty()) {
        BStarNode* node = nodeStack.back();
        nodeStack.pop_back();
        
        int width, height;
        if (node->isSymmetryIsland) {
            // Extract island index from name (format: "island_X")
            size_t islandIndex = std::stoi(node->name.substr(7));
            if (islandIndex >= symmetryIslands.size()) {
                continue; // Invalid island index
            }
            
            // Get dimensions from the symmetry island
            width = symmetryIslands[islandIndex]->getWidth();
            height = symmetryIslands[islandIndex]->getHeight();
        } else {
            // Regular module
            if (regularModules.find(node->name) == regularModules.end()) {
                continue; // Module not found
            }
            
            std::shared_ptr<Module> module = regularModules[node->name];
            width = module->getWidth();
            height = module->getHeight();
        }
        
        // Determine x-coordinate from the parent-child relationship
        int x = 0;
        if (node != bstarRoot) {
            bool found = false;
            for (BStarNode* parent : nodeStack) {
                if (parent->left == node) {
                    // Left child: placed to the right of parent
                    if (parent->isSymmetryIsland) {
                        size_t parentIslandIndex = std::stoi(parent->name.substr(7));
                        x = symmetryIslands[parentIslandIndex]->getX() + symmetryIslands[parentIslandIndex]->getWidth();
                    } else {
                        std::shared_ptr<Module> parentModule = regularModules[parent->name];
                        x = parentModule->getX() + parentModule->getWidth();
                    }
                    found = true;
                    break;
                } else if (parent->right == node) {
                    // Right child: same x-coordinate as parent
                    if (parent->isSymmetryIsland) {
                        size_t parentIslandIndex = std::stoi(parent->name.substr(7));
                        x = symmetryIslands[parentIslandIndex]->getX();
                    } else {
                        std::shared_ptr<Module> parentModule = regularModules[parent->name];
                        x = parentModule->getX();
                    }
                    found = true;
                    break;
                }
            }
            
            if (!found) {
                // This should not happen in a valid B*-tree
                continue;
            }
        }
        
        // Determine y-coordinate from the contour
        int y = getContourHeight(x);
        
        // Update the position
        if (node->isSymmetryIsland) {
            size_t islandIndex = std::stoi(node->name.substr(7));
            symmetryIslands[islandIndex]->setPosition(x, y);
        } else {
            regularModules[node->name]->setPosition(x, y);
        }
        
        // Update the contour
        updateContour(x, y, width, height);
        
        // Add children to the stack (right child first for proper DFS order)
        if (node->right != nullptr) {
            nodeStack.push_back(node->right);
        }
        if (node->left != nullptr) {
            nodeStack.push_back(node->left);
        }
    }
}

// Calculate bounding box area
int PlacementSolver::calculateArea() {
    int minX = 0;
    int minY = 0;
    int maxX = 0;
    int maxY = 0;
    
    // Check symmetry islands
    for (const auto& island : symmetryIslands) {
        maxX = std::max(maxX, island->getX() + island->getWidth());
        maxY = std::max(maxY, island->getY() + island->getHeight());
    }
    
    // Check regular modules
    for (const auto& pair : regularModules) {
        const auto& module = pair.second;
        maxX = std::max(maxX, module->getX() + module->getWidth());
        maxY = std::max(maxY, module->getY() + module->getHeight());
    }
    
    return maxX * maxY;
}

// Calculate half-perimeter wirelength (placeholder)
double PlacementSolver::calculateWirelength() {
    // This is a placeholder - in a real implementation, you would calculate
    // the wirelength based on module connectivity information
    
    // For this simplified version, we'll use a proxy: sum of distances from modules to the origin
    double wirelength = 0.0;
    
    // Include symmetry islands
    for (const auto& island : symmetryIslands) {
        wirelength += island->getX() + island->getY();
    }
    
    // Include regular modules
    for (const auto& pair : regularModules) {
        const auto& module = pair.second;
        wirelength += module->getX() + module->getY();
    }
    
    return wirelength;
}

// Calculate cost of current solution
double PlacementSolver::calculateCost() {
    int area = calculateArea();
    double wirelength = calculateWirelength();
    
    return areaWeight * area + wirelengthWeight * wirelength;
}

// Perform random perturbation
bool PlacementSolver::perturb() {
    double rand = static_cast<double>(std::rand()) / RAND_MAX;
    double cumulativeProb = 0.0;
    
    // Backup the tree structure before any perturbation
    backupBStarTree();
    
    bool success = false;
    
    // Select perturbation type based on probabilities
    if ((cumulativeProb += rotateProb) > rand) {
        // Rotate a module
        BStarNode* node = findRandomNode();
        if (node) {
            success = rotateModule(node->isSymmetryIsland, node->name);
        }
    } else if ((cumulativeProb += moveProb) > rand) {
        // Move a node
        BStarNode* node = findRandomNode();
        if (node) {
            success = moveNode(node);
        }
    } else if ((cumulativeProb += swapProb) > rand) {
        // Swap two nodes
        success = swapNodes();
    } else if ((cumulativeProb += changeRepProb) > rand) {
        // Change representative
        success = changeRepresentative();
    } else if ((cumulativeProb += convertSymProb) > rand) {
        // Convert symmetry type
        success = convertSymmetryType();
    }
    
    // Validate the tree after perturbation
    if (success && !validateBStarTree()) {
        // If validation fails, restore from backup
        restoreBStarTree();
        success = false;
    }
    
    return success;
}

// Rotate a module
bool PlacementSolver::rotateModule(bool isSymmetryIsland, const std::string& name) {
    if (isSymmetryIsland) {
        // Extract island index
        size_t islandIndex = std::stoi(name.substr(7));
        if (islandIndex < symmetryIslands.size()) {
            // Rotate the symmetry island
            symmetryIslands[islandIndex]->rotate();
            return true;
        }
    } else {
        // Rotate a regular module
        if (regularModules.find(name) != regularModules.end()) {
            regularModules[name]->rotate();
            return true;
        }
    }
    
    return false;
}

// Move a node in the B*-tree
bool PlacementSolver::moveNode(BStarNode* node) {
    if (node == nullptr || node == bstarRoot) {
        return false;
    }
    
    // Before perturbation, backup the tree structure
    backupBStarTree();
    
    // Find the parent of the node to move
    BStarNode* parent = nullptr;
    BStarNode* current = bstarRoot;
    std::queue<BStarNode*> queue;
    queue.push(current);
    
    while (!queue.empty()) {
        current = queue.front();
        queue.pop();
        
        if (current->left == node || current->right == node) {
            parent = current;
            break;
        }
        
        if (current->left) queue.push(current->left);
        if (current->right) queue.push(current->right);
    }
    
    if (parent == nullptr) {
        return false;
    }
    
    // Detach node from its parent
    if (parent->left == node) {
        parent->left = nullptr;
    } else {
        parent->right = nullptr;
    }
    
    // Find potential new parents (exclude descendants of node to prevent cycles)
    std::vector<BStarNode*> potentialParents;
    std::unordered_set<BStarNode*> descendants;
    
    // First collect all descendants of the node
    std::function<void(BStarNode*)> collectDescendants = [&](BStarNode* n) {
        if (n == nullptr) return;
        descendants.insert(n);
        collectDescendants(n->left);
        collectDescendants(n->right);
    };
    collectDescendants(node);
    
    // Now collect all nodes that are not descendants
    std::function<void(BStarNode*)> collectNonDescendants = [&](BStarNode* n) {
        if (n == nullptr) return;
        if (descendants.find(n) == descendants.end()) {
            potentialParents.push_back(n);
        }
        collectNonDescendants(n->left);
        collectNonDescendants(n->right);
    };
    collectNonDescendants(bstarRoot);
    
    if (potentialParents.empty()) {
        // Restore original connection and return failure
        if (parent->left == nullptr) {
            parent->left = node;
        } else {
            parent->right = node;
        }
        return false;
    }
    
    // Try to find a parent with an available child slot
    bool placed = false;
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(potentialParents.begin(), potentialParents.end(), g);
    
    for (BStarNode* newParent : potentialParents) {
        // Try left child first if it's empty
        if (newParent->left == nullptr) {
            newParent->left = node;
            placed = true;
            break;
        }
        // Try right child if it's empty
        else if (newParent->right == nullptr) {
            newParent->right = node;
            placed = true;
            break;
        }
    }
    
    // If we couldn't place the node (all potential parents have both children),
    // find a leaf node to attach to
    if (!placed) {
        // Find leaf nodes (nodes with at least one nullptr child)
        std::vector<BStarNode*> leafNodes;
        for (BStarNode* potential : potentialParents) {
            if (potential->left == nullptr || potential->right == nullptr) {
                leafNodes.push_back(potential);
            }
        }
        
        if (!leafNodes.empty()) {
            // Select a random leaf node
            BStarNode* leafNode = leafNodes[std::rand() % leafNodes.size()];
            
            // Add to whichever child pointer is nullptr
            if (leafNode->left == nullptr) {
                leafNode->left = node;
                placed = true;
            } else if (leafNode->right == nullptr) {
                leafNode->right = node;
                placed = true;
            }
        }
    }
    
    // If still not placed, restore original position and return failure
    if (!placed) {
        if (parent->left == nullptr) {
            parent->left = node;
        } else {
            parent->right = node;
        }
        return false;
    }
    
    // Validate the resulting tree structure
    if (!validateBStarTree()) {
        // If invalid, restore from backup and return failure
        restoreBStarTree();
        return false;
    }
    
    return true;
}

// Swap two nodes in the B*-tree
bool PlacementSolver::swapNodes() {
    // Need at least 2 nodes
    if (preorderTraversal.size() < 2) {
        return false;
    }
    
    // Select two random nodes
    int idx1 = std::rand() % preorderTraversal.size();
    int idx2;
    do {
        idx2 = std::rand() % preorderTraversal.size();
    } while (idx1 == idx2);
    
    BStarNode* node1 = preorderTraversal[idx1];
    BStarNode* node2 = preorderTraversal[idx2];
    
    // Swap node contents (name and isSymmetryIsland flag)
    std::swap(node1->name, node2->name);
    std::swap(node1->isSymmetryIsland, node2->isSymmetryIsland);
    
    return true;
}

// Change representative for a symmetry pair
bool PlacementSolver::changeRepresentative() {
    // Select a random symmetry island
    if (symmetryIslands.empty()) {
        return false;
    }
    
    size_t islandIndex = std::rand() % symmetryIslands.size();
    auto island = symmetryIslands[islandIndex];
    
    // Get the ASF-B*-tree from the island
    auto asfBStarTree = island->getASFBStarTree();
    
    // Perturb the ASF-B*-tree by changing a representative
    return asfBStarTree->perturb(3); // Type 3 is "change representative"
}

// Convert symmetry type for a symmetry group
bool PlacementSolver::convertSymmetryType() {
    // Select a random symmetry island
    if (symmetryIslands.empty()) {
        return false;
    }
    
    size_t islandIndex = std::rand() % symmetryIslands.size();
    auto island = symmetryIslands[islandIndex];
    
    // Get the ASF-B*-tree from the island
    auto asfBStarTree = island->getASFBStarTree();
    
    // Perturb the ASF-B*-tree by converting symmetry type
    return asfBStarTree->perturb(4); // Type 4 is "convert symmetry type"
}

// Added tree validation for PlacementSolver
bool PlacementSolver::validateBStarTree() {
    if (bstarRoot == nullptr) return true;
    
    // Set to keep track of visited nodes
    std::unordered_set<BStarNode*> visited;
    
    // Function to check for cycles in the tree
    std::function<bool(BStarNode*, std::unordered_set<BStarNode*>&)> hasNoCycles =
        [&](BStarNode* current, std::unordered_set<BStarNode*>& path) -> bool {
            if (current == nullptr) return true;
            
            // If we've seen this node in the current path, we have a cycle
            if (path.find(current) != path.end()) {
                return false;
            }
            
            // Add this node to the current path
            path.insert(current);
            visited.insert(current);
            
            // Check children
            bool leftValid = hasNoCycles(current->left, path);
            bool rightValid = hasNoCycles(current->right, path);
            
            // Remove this node from the current path (backtracking)
            path.erase(current);
            
            return leftValid && rightValid;
        };
    
    // Start DFS from the root to check for cycles
    std::unordered_set<BStarNode*> path;
    bool noCycles = hasNoCycles(bstarRoot, path);
    
    // Make sure each node in the tree is reachable from the root
    // First count all nodes in the tree
    size_t totalNodes = 0;
    std::function<void(BStarNode*)> countNodes = [&](BStarNode* n) {
        if (n == nullptr) return;
        totalNodes++;
        countNodes(n->left);
        countNodes(n->right);
    };
    countNodes(bstarRoot);
    
    // Make sure we visited all nodes in the cycle check
    return noCycles && (visited.size() == totalNodes);
}

// Methods for backing up and restoring B*-tree structure in PlacementSolver
void PlacementSolver::backupBStarTree() {
    // Store the current tree structure
    bstarTreeBackup.preorderNodes.clear();
    bstarTreeBackup.inorderNodes.clear();
    
    // Populate the backup traversals
    std::function<void(BStarNode*)> preorderBackup = [&](BStarNode* node) {
        if (node == nullptr) return;
        bstarTreeBackup.preorderNodes.push_back({node->name, node->isSymmetryIsland});
        preorderBackup(node->left);
        preorderBackup(node->right);
    };
    
    std::function<void(BStarNode*)> inorderBackup = [&](BStarNode* node) {
        if (node == nullptr) return;
        inorderBackup(node->left);
        bstarTreeBackup.inorderNodes.push_back({node->name, node->isSymmetryIsland});
        inorderBackup(node->right);
    };
    
    preorderBackup(bstarRoot);
    inorderBackup(bstarRoot);
}

// Restore B*-tree from backup
void PlacementSolver::restoreBStarTree() {
    if (bstarTreeBackup.preorderNodes.empty() || bstarTreeBackup.inorderNodes.empty()) {
        return; // No backup available
    }
    
    // Clean up existing tree
    cleanupBStarTree(bstarRoot);
    bstarRoot = nullptr;
    
    // Create nodes for all entities
    std::unordered_map<std::string, BStarNode*> nodeMap;
    for (const auto& info : bstarTreeBackup.preorderNodes) {
        if (nodeMap.find(info.name) == nodeMap.end()) {
            nodeMap[info.name] = new BStarNode(info.name, info.isIsland);
        }
    }
    
    // Create mapping from name to inorder index
    std::unordered_map<std::string, size_t> inorderMap;
    for (size_t i = 0; i < bstarTreeBackup.inorderNodes.size(); i++) {
        inorderMap[bstarTreeBackup.inorderNodes[i].name] = i;
    }
    
    // Recursive function to rebuild the tree
    std::function<BStarNode*(size_t&, size_t, size_t)> rebuildTree = 
        [&](size_t& preIdx, size_t inStart, size_t inEnd) -> BStarNode* {
            if (inStart > inEnd || preIdx >= bstarTreeBackup.preorderNodes.size()) {
                return nullptr;
            }
            
            // Get the root of current subtree from preorder traversal
            const auto& nodeInfo = bstarTreeBackup.preorderNodes[preIdx++];
            BStarNode* node = nodeMap[nodeInfo.name];
            
            // If this is the only element in this subtree
            if (inStart == inEnd) {
                return node;
            }
            
            // Find the index of current node in inorder traversal
            size_t inIndex = inorderMap[nodeInfo.name];
            
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
    bstarRoot = rebuildTree(preIdx, 0, bstarTreeBackup.inorderNodes.size() - 1);
}

// Check for module overlaps
bool PlacementSolver::hasOverlaps() {
    // Check overlaps between regular modules
    for (const auto& pair1 : regularModules) {
        const auto& module1 = pair1.second;
        
        // Check against other regular modules
        for (const auto& pair2 : regularModules) {
            if (pair1.first == pair2.first) continue; // Skip self
            
            const auto& module2 = pair2.second;
            if (module1->overlaps(*module2)) {
                return true;
            }
        }
        
        // Check against symmetry islands
        for (const auto& island : symmetryIslands) {
            if (island->overlaps(*module1)) {
                return true;
            }
        }
    }
    
    // Check overlaps between symmetry islands
    for (size_t i = 0; i < symmetryIslands.size(); i++) {
        for (size_t j = i + 1; j < symmetryIslands.size(); j++) {
            if (symmetryIslands[i]->overlaps(*symmetryIslands[j])) {
                return true;
            }
        }
    }
    
    return false;
}

// Deep copy of module data
std::map<std::string, std::shared_ptr<Module>> PlacementSolver::copyModules(
    const std::map<std::string, std::shared_ptr<Module>>& source) {
    
    std::map<std::string, std::shared_ptr<Module>> result;
    
    for (const auto& pair : source) {
        result[pair.first] = std::make_shared<Module>(*pair.second);
    }
    
    return result;
}

// Find a random node in the B*-tree
PlacementSolver::BStarNode* PlacementSolver::findRandomNode() {
    if (preorderTraversal.empty()) {
        return nullptr;
    }
    
    return preorderTraversal[std::rand() % preorderTraversal.size()];
}

// Copy current solution to best solution
void PlacementSolver::updateBestSolution() {
    bestSolutionArea = solutionArea;
    bestSolutionWirelength = solutionWirelength;
    
    // Update all modules in the solution
    bestSolutionModules.clear();
    
    // Add regular modules
    for (const auto& pair : regularModules) {
        bestSolutionModules[pair.first] = std::make_shared<Module>(*pair.second);
    }
    
    // Add modules from symmetry islands
    for (const auto& island : symmetryIslands) {
        for (const auto& pair : island->getASFBStarTree()->getModules()) {
            bestSolutionModules[pair.first] = std::make_shared<Module>(*pair.second);
        }
    }
}

// Copy best solution to current solution
void PlacementSolver::restoreBestSolution() {
    // Restore module positions from best solution
    for (const auto& pair : bestSolutionModules) {
        const std::string& name = pair.first;
        const auto& module = pair.second;
        
        // Check if it's a regular module
        if (regularModules.find(name) != regularModules.end()) {
            regularModules[name]->setPosition(module->getX(), module->getY());
            regularModules[name]->setRotation(module->getRotated());
        } else {
            // It's in a symmetry island - find and update it
            for (const auto& island : symmetryIslands) {
                const auto& islandModules = island->getASFBStarTree()->getModules();
                if (islandModules.find(name) != islandModules.end()) {
                    islandModules.at(name)->setPosition(module->getX(), module->getY());
                    islandModules.at(name)->setRotation(module->getRotated());
                    break;
                }
            }
        }
    }
    
    // Update solution metrics
    solutionArea = bestSolutionArea;
    solutionWirelength = bestSolutionWirelength;
}

// Load the problem data
bool PlacementSolver::loadProblem(
    const std::map<std::string, std::shared_ptr<Module>>& modules,
    const std::vector<std::shared_ptr<SymmetryGroup>>& symmetryGroups) {
    
    // Clear current data
    this->modules = modules;
    this->symmetryGroups = symmetryGroups;
    
    regularModules.clear();
    symmetryIslands.clear();
    
    // Create symmetry islands
    for (size_t i = 0; i < symmetryGroups.size(); i++) {
        auto symmetryGroup = symmetryGroups[i];
        
        // Collect modules in this symmetry group
        std::map<std::string, std::shared_ptr<Module>> groupModules;
        
        // Add symmetry pairs
        for (const auto& pair : symmetryGroup->getSymmetryPairs()) {
            if (modules.find(pair.first) != modules.end()) {
                groupModules[pair.first] = modules.at(pair.first);
            }
            if (modules.find(pair.second) != modules.end()) {
                groupModules[pair.second] = modules.at(pair.second);
            }
        }
        
        // Add self-symmetric modules
        for (const auto& name : symmetryGroup->getSelfSymmetric()) {
            if (modules.find(name) != modules.end()) {
                groupModules[name] = modules.at(name);
            }
        }
        
        // Create ASF-B*-tree for this symmetry group
        auto asfBStarTree = std::make_shared<ASFBStarTree>(symmetryGroup, groupModules);
        
        // Pack the ASF-B*-tree to get initial layout
        asfBStarTree->pack();
        
        // Create symmetry island
        auto island = std::make_shared<SymmetryIslandBlock>("sg_" + std::to_string(i), asfBStarTree);
        
        // Update bounding box
        island->updateBoundingBox();
        
        symmetryIslands.push_back(island);
    }
    
    // Collect regular modules (not in any symmetry group)
    std::unordered_set<std::string> symmetryModules;
    
    for (const auto& group : symmetryGroups) {
        for (const auto& pair : group->getSymmetryPairs()) {
            symmetryModules.insert(pair.first);
            symmetryModules.insert(pair.second);
        }
        for (const auto& name : group->getSelfSymmetric()) {
            symmetryModules.insert(name);
        }
    }
    
    for (const auto& pair : modules) {
        if (symmetryModules.find(pair.first) == symmetryModules.end()) {
            regularModules[pair.first] = pair.second;
        }
    }
    
    // Build initial B*-tree for global placement
    buildInitialBStarTree();
    
    return true;
}

// Set simulated annealing parameters
void PlacementSolver::setAnnealingParameters(
    double initialTemp, double finalTemp, double cooling,
    int iterations, int noImprovementLimit) {
    
    initialTemperature = initialTemp;
    finalTemperature = finalTemp;
    coolingRate = cooling;
    iterationsPerTemperature = iterations;
    this->noImprovementLimit = noImprovementLimit;
}

// Set perturbation probabilities
void PlacementSolver::setPerturbationProbabilities(
    double rotate, double move, double swap,
    double changeRep, double convertSym) {
    
    rotateProb = rotate;
    moveProb = move;
    swapProb = swap;
    changeRepProb = changeRep;
    convertSymProb = convertSym;
}

// Set cost weights
void PlacementSolver::setCostWeights(double areaWeight, double wirelengthWeight) {
    this->areaWeight = areaWeight;
    this->wirelengthWeight = wirelengthWeight;
}

// Set random seed
void PlacementSolver::setRandomSeed(unsigned int seed) {
    rng.seed(seed);
    std::srand(seed);
}

// Set time limit
void PlacementSolver::setTimeLimit(int seconds) {
    timeLimit = seconds;
}

// Solve the placement problem
bool PlacementSolver::solve() {
    // Record start time
    startTime = std::chrono::steady_clock::now();
    
    // Initial packing
    packBStarTree();
    
    // Calculate initial solution metrics
    solutionArea = calculateArea();
    solutionWirelength = calculateWirelength();
    
    // Initialize best solution
    bestSolutionArea = solutionArea;
    bestSolutionWirelength = solutionWirelength;
    updateBestSolution();
    
    // Also backup the initial tree structure
    backupBStarTree();
    bestBStarTreeBackup = bstarTreeBackup;
    
    std::cout << "Initial solution - Area: " << solutionArea << ", Cost: " << calculateCost() << std::endl;
    
    // Simulated annealing
    double temperature = initialTemperature;
    int iterations = 0;
    int noImprovementCount = 0;
    
    while (temperature > finalTemperature && noImprovementCount < noImprovementLimit) {
        bool improved = false;
        
        for (int i = 0; i < iterationsPerTemperature; i++) {
            // Check time limit
            auto currentTime = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                currentTime - startTime).count();
            
            if (elapsed >= timeLimit) {
                std::cout << "Time limit reached." << std::endl;
                break;
            }
            
            // Save current state and metrics
            int oldArea = solutionArea;
            double oldWirelength = solutionWirelength;
            double oldCost = calculateCost();
            
            // Backup current tree structure
            backupBStarTree();
            
            // Create backup of module positions
            std::map<std::string, std::pair<int, int>> oldPositions;
            std::map<std::string, bool> oldRotations;
            
            for (const auto& pair : regularModules) {
                oldPositions[pair.first] = {pair.second->getX(), pair.second->getY()};
                oldRotations[pair.first] = pair.second->getRotated();
            }
            
            // Create backup of islands
            std::vector<std::pair<int, int>> oldIslandPositions;
            for (const auto& island : symmetryIslands) {
                oldIslandPositions.push_back({island->getX(), island->getY()});
            }
            
            // Perturb the solution
            bool perturbSuccess = perturb();
            
            if (perturbSuccess) {
                // Re-pack
                packBStarTree();
                
                // Calculate new metrics
                int newArea = calculateArea();
                double newWirelength = calculateWirelength();
                double newCost = areaWeight * newArea + wirelengthWeight * newWirelength;
                
                // Check if we should accept this solution
                double costDelta = newCost - oldCost;
                bool accept = false;
                
                if (costDelta <= 0) {
                    // Accept improvement
                    accept = true;
                    
                    if (newArea < bestSolutionArea) {
                        // Found a new best solution
                        bestSolutionArea = newArea;
                        bestSolutionWirelength = newWirelength;
                        updateBestSolution();
                        bestBStarTreeBackup = bstarTreeBackup; // Save tree structure too
                        improved = true;
                        noImprovementCount = 0;
                        
                        std::cout << "New best solution - Area: " << newArea 
                                  << ", Cost: " << newCost 
                                  << ", Temp: " << temperature << std::endl;
                    }
                } else {
                    // Accept with probability based on temperature
                    double acceptProb = exp(-costDelta / temperature);
                    if (static_cast<double>(std::rand()) / RAND_MAX < acceptProb) {
                        accept = true;
                    }
                }
                
                if (accept) {
                    // Update solution metrics
                    solutionArea = newArea;
                    solutionWirelength = newWirelength;
                } else {
                    // Restore the tree structure
                    restoreBStarTree();
                    
                    // Restore old positions
                    for (const auto& pair : oldPositions) {
                        if (regularModules.find(pair.first) != regularModules.end()) {
                            regularModules[pair.first]->setPosition(pair.second.first, pair.second.second);
                            regularModules[pair.first]->setRotation(oldRotations[pair.first]);
                        }
                    }
                    
                    // Restore islands
                    for (size_t i = 0; i < symmetryIslands.size(); i++) {
                        symmetryIslands[i]->setPosition(oldIslandPositions[i].first, oldIslandPositions[i].second);
                    }
                    
                    // Restore metrics
                    solutionArea = oldArea;
                    solutionWirelength = oldWirelength;
                }
            }
            
            iterations++;
        }
        
        // Update temperature
        temperature *= coolingRate;
        
        if (!improved) {
            noImprovementCount++;
        }
    }
    
    std::cout << "Annealing complete - " << iterations << " iterations" << std::endl;
    
    // Restore best solution including tree structure
    bstarTreeBackup = bestBStarTreeBackup;
    restoreBStarTree();
    restoreBestSolution();
    
    std::cout << "Final solution - Area: " << solutionArea << std::endl;
    
    return true;
}

// Get solution area
int PlacementSolver::getSolutionArea() const {
    return solutionArea;
}

// Get solution modules
const std::map<std::string, std::shared_ptr<Module>>& PlacementSolver::getSolutionModules() const {
    return bestSolutionModules;
}