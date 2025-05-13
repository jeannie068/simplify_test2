/**
 * PlacementPerturber.cpp
 * 
 * Implementation of the PlacementPerturber class that handles all perturbation
 * operations for the placement algorithm.
 */

#include "PlacementPerturber.hpp"
#include <iostream>
#include <queue>
#include <algorithm>
#include <random>

bool PlacementPerturber::rotateModule(std::shared_ptr<Module> module) {
    if (!module) return false;
    
    module->rotate();
    return true;
}

bool PlacementPerturber::rotateSymmetryModule(
    std::shared_ptr<ASFBStarTree> asfTree, 
    const std::string& moduleName) {
    
    if (!asfTree) return false;
    
    // Let the ASF-B*-tree handle the rotation (it manages symmetry pairs)
    return asfTree->rotateModule(moduleName);
}

bool PlacementPerturber::moveModule(
    std::shared_ptr<Module> module, 
    int x, int y) {
    
    if (!module) return false;
    
    module->setPosition(x, y);
    return true;
}

bool PlacementPerturber::swapModules(
    std::shared_ptr<Module> module1, 
    std::shared_ptr<Module> module2) {
    
    if (!module1 || !module2) return false;
    
    // Swap positions
    int tmpX = module1->getX();
    int tmpY = module1->getY();
    
    module1->setPosition(module2->getX(), module2->getY());
    module2->setPosition(tmpX, tmpY);
    
    return true;
}

bool PlacementPerturber::swapTreeNodes(
    std::shared_ptr<BStarTreeNode> node1, 
    std::shared_ptr<BStarTreeNode> node2) {
    
    if (!node1 || !node2 || node1 == node2) return false;
    
    // Instead of restructuring the tree, we'll swap the module names
    // To actually swap tree nodes, we would need to update parents and children
    // which is more complex and can create cycles
    
    std::string tempName = node1->getModuleName();
    
    // This is a simplification - in a real implementation we would need to 
    // either update the module name in the node (not possible with current API)
    // or swap the entire subtrees while maintaining tree structure
    
    // Since we can't modify the node module name directly, we would need to
    // update external references to these nodes, or create new nodes with swapped names
    
    return false;  // Current implementation limitation
}

bool PlacementPerturber::moveNodeInTree(
    std::shared_ptr<BStarTreeNode> nodeToMove, 
    std::shared_ptr<BStarTreeNode> newParent,
    bool asLeftChild) {
    
    if (!nodeToMove || !newParent || nodeToMove == newParent) return false;
    
    // Ensure we're not creating a cycle
    auto current = newParent;
    while (current) {
        if (current == nodeToMove) return false;
        current = current->getParent();
    }
    
    // Remove the node from its current parent
    auto oldParent = nodeToMove->getParent();
    if (oldParent) {
        if (oldParent->getLeftChild() == nodeToMove) {
            oldParent->setLeftChild(nullptr);
        } else if (oldParent->getRightChild() == nodeToMove) {
            oldParent->setRightChild(nullptr);
        }
    }
    
    nodeToMove->setParent(newParent);
    
    // Attach to new parent
    if (asLeftChild) {
        auto oldChild = newParent->getLeftChild();
        newParent->setLeftChild(nodeToMove);
        
        // If there was already a child there, make it a child of the moved node
        if (oldChild) {
            if (!nodeToMove->getLeftChild()) {
                nodeToMove->setLeftChild(oldChild);
                oldChild->setParent(nodeToMove);
            } else if (!nodeToMove->getRightChild()) {
                nodeToMove->setRightChild(oldChild);
                oldChild->setParent(nodeToMove);
            }
        }
    } else {
        auto oldChild = newParent->getRightChild();
        newParent->setRightChild(nodeToMove);
        
        // If there was already a child there, make it a child of the moved node
        if (oldChild) {
            if (!nodeToMove->getLeftChild()) {
                nodeToMove->setLeftChild(oldChild);
                oldChild->setParent(nodeToMove);
            } else if (!nodeToMove->getRightChild()) {
                nodeToMove->setRightChild(oldChild);
                oldChild->setParent(nodeToMove);
            }
        }
    }
    
    return true;
}

bool PlacementPerturber::changeRepresentative(
    std::shared_ptr<ASFBStarTree> asfTree, 
    const std::string& moduleName) {
    
    if (!asfTree) return false;
    
    return asfTree->changeRepresentative(moduleName);
}

bool PlacementPerturber::convertSymmetryType(std::shared_ptr<ASFBStarTree> asfTree) {
    if (!asfTree) return false;
    
    return asfTree->convertSymmetryType();
}

bool PlacementPerturber::perturbGlobalTree(
    std::shared_ptr<BStarTreeNode>& globalTree,
    std::map<std::string, std::variant<std::shared_ptr<Module>, 
                                      std::shared_ptr<SymmetryIslandBlock>>>& globalNodes,
    std::mt19937& rng) {
    
    if (!globalTree || globalNodes.empty()) {
        return false;
    }
    
    // Choose a random perturbation type
    std::uniform_real_distribution<double> uniformDist(0.0, 1.0);
    double randVal = uniformDist(rng);
    
    if (randVal < 0.3) {
        // Rotate a node
        std::vector<std::string> nodeNames;
        for (const auto& pair : globalNodes) {
            nodeNames.push_back(pair.first);
        }
        
        if (nodeNames.empty()) return false;
        
        int idx = static_cast<int>(uniformDist(rng) * nodeNames.size());
        std::string nodeName = nodeNames[idx];
        
        return rotateGlobalNode(nodeName, globalNodes);
    } 
    else if (randVal < 0.65) {
        // Swap two nodes in the tree
        std::vector<std::string> nodeNames;
        for (const auto& pair : globalNodes) {
            nodeNames.push_back(pair.first);
        }
        
        if (nodeNames.size() < 2) return false;
        
        // Choose two random nodes
        int idx1 = static_cast<int>(uniformDist(rng) * nodeNames.size());
        int idx2;
        do {
            idx2 = static_cast<int>(uniformDist(rng) * nodeNames.size());
        } while (idx2 == idx1);
        
        std::string name1 = nodeNames[idx1];
        std::string name2 = nodeNames[idx2];
        
        // Find the nodes in the tree
        std::shared_ptr<BStarTreeNode> node1 = findNodeInTree(globalTree, name1);
        std::shared_ptr<BStarTreeNode> node2 = findNodeInTree(globalTree, name2);
        
        if (node1 && node2) {
            // Swap the objects in globalNodes
            if (globalNodes.find(node1->getModuleName()) != globalNodes.end() &&
                globalNodes.find(node2->getModuleName()) != globalNodes.end()) {
                std::swap(globalNodes[node1->getModuleName()], globalNodes[node2->getModuleName()]);
                return true;
            }
        }
    }
    else {
        // Move a subtree
        std::vector<std::string> nodeNames;
        for (const auto& pair : globalNodes) {
            nodeNames.push_back(pair.first);
        }
        
        if (nodeNames.size() < 2) return false;
        
        // Choose two random nodes
        int idx1 = static_cast<int>(uniformDist(rng) * nodeNames.size());
        int idx2;
        do {
            idx2 = static_cast<int>(uniformDist(rng) * nodeNames.size());
        } while (idx2 == idx1);
        
        std::string name1 = nodeNames[idx1];
        std::string name2 = nodeNames[idx2];
        
        // Find the nodes in the tree
        std::shared_ptr<BStarTreeNode> nodeToMove = findNodeInTree(globalTree, name1);
        std::shared_ptr<BStarTreeNode> newParent = findNodeInTree(globalTree, name2);
        
        if (nodeToMove && newParent && nodeToMove != globalTree) {
            return moveNodeInTree(nodeToMove, newParent, uniformDist(rng) < 0.5);
        }
    }
    
    return false;
}

bool PlacementPerturber::rotateGlobalNode(
    const std::string& nodeName,
    std::map<std::string, std::variant<std::shared_ptr<Module>, 
                                      std::shared_ptr<SymmetryIslandBlock>>>& globalNodes) {
    
    auto it = globalNodes.find(nodeName);
    
    if (it == globalNodes.end()) return false;
    
    // Rotate the module or island
    return std::visit([](auto&& arg) -> bool {
        using T = std::decay_t<decltype(arg)>;
        if constexpr (std::is_same_v<T, std::shared_ptr<Module>>) {
            arg->rotate();
            return true;
        } else if constexpr (std::is_same_v<T, std::shared_ptr<SymmetryIslandBlock>>) {
            arg->rotate();
            return true;
        } else {
            return false;
        }
    }, it->second);
}

std::shared_ptr<BStarTreeNode> PlacementPerturber::findNodeInTree(
    const std::shared_ptr<BStarTreeNode>& root, 
    const std::string& name) {
    
    if (!root) return nullptr;
    if (root->getModuleName() == name) return root;
    
    auto leftResult = findNodeInTree(root->getLeftChild(), name);
    if (leftResult) return leftResult;
    
    return findNodeInTree(root->getRightChild(), name);
}

void PlacementPerturber::collectNodesPreOrder(
    const std::shared_ptr<BStarTreeNode>& root,
    std::vector<std::shared_ptr<BStarTreeNode>>& nodes) {
    
    if (!root) return;
    
    // Pre-order: Root, Left, Right
    nodes.push_back(root);
    
    if (root->getLeftChild()) {
        collectNodesPreOrder(root->getLeftChild(), nodes);
    }
    
    if (root->getRightChild()) {
        collectNodesPreOrder(root->getRightChild(), nodes);
    }
}

void PlacementPerturber::collectNodesInOrder(
    const std::shared_ptr<BStarTreeNode>& root,
    std::vector<std::shared_ptr<BStarTreeNode>>& nodes) {
    
    if (!root) return;
    
    // In-order: Left, Root, Right
    if (root->getLeftChild()) {
        collectNodesInOrder(root->getLeftChild(), nodes);
    }
    
    nodes.push_back(root);
    
    if (root->getRightChild()) {
        collectNodesInOrder(root->getRightChild(), nodes);
    }
}