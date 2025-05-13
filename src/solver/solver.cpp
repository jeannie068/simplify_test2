/**
 * solver.cpp
 * 
 * Implementation of the PlacementSolver class which orchestrates the overall
 * placement algorithm using the new modular architecture.
 */

#include "solver.hpp"
#include "../utils/PlacementPacker.hpp"
#include "../utils/PlacementPerturber.hpp"
#include <iostream>
#include <ctime>
#include <algorithm>
#include <limits>
#include <queue>
#include <random>

PlacementSolver::PlacementSolver()
    : initialTemperature(1000.0),
      finalTemperature(0.1),
      coolingRate(0.98),
      iterationsPerTemperature(300),
      noImprovementLimit(3000),
      probRotate(0.4),
      probMove(0.3),
      probSwap(0.2),
      probChangeRep(0.05),
      probConvertSym(0.05),
      areaWeight(1.0),
      wirelengthWeight(0.0),
      randomSeed(static_cast<unsigned int>(std::time(nullptr))),
      uniformDist(0.0, 1.0),
      timeLimit(290) {
    
    // Initialize random number generator
    rng.seed(randomSeed);
}

PlacementSolver::~PlacementSolver() {
    // Smart pointers handle cleanup
}

void PlacementSolver::loadProblem(const std::map<std::string, std::shared_ptr<Module>>& modules,
                                const std::vector<std::shared_ptr<SymmetryGroup>>& symmetryGroups) {
    // Initialize the placement model
    model = PlacementModel(modules, symmetryGroups);
    
    // Initialize module grouping
    initializeModuleGrouping();
}

void PlacementSolver::initializeModuleGrouping() {
    // Clear existing groupings
    model.regularModules.clear();
    model.symmetryTrees.clear();
    model.moduleToGroup.clear();
    
    // Track which modules are part of symmetry groups
    std::unordered_set<std::string> symmetryModules;
    
    // Process symmetry groups
    for (const auto& group : model.symmetryGroups) {
        // Create a new ASF-B*-tree for this symmetry group
        auto asfTree = std::make_shared<ASFBStarTree>(group);
        model.symmetryTrees[group->getName()] = asfTree;
        
        // Process symmetry pairs
        for (const auto& pair : group->getSymmetryPairs()) {
            // Add modules to the symmetry group
            if (model.allModules.find(pair.first) != model.allModules.end()) {
                asfTree->addModule(model.allModules[pair.first]);
                symmetryModules.insert(pair.first);
                model.moduleToGroup[pair.first] = group;
            }
            
            if (model.allModules.find(pair.second) != model.allModules.end()) {
                asfTree->addModule(model.allModules[pair.second]);
                symmetryModules.insert(pair.second);
                model.moduleToGroup[pair.second] = group;
            }
        }
        
        // Process self-symmetric modules
        for (const auto& moduleName : group->getSelfSymmetric()) {
            if (model.allModules.find(moduleName) != model.allModules.end()) {
                asfTree->addModule(model.allModules[moduleName]);
                symmetryModules.insert(moduleName);
                model.moduleToGroup[moduleName] = group;
            }
        }
    }
    
    // Identify regular modules (not part of symmetry groups)
    for (const auto& pair : model.allModules) {
        if (symmetryModules.find(pair.first) == symmetryModules.end()) {
            model.regularModules[pair.first] = pair.second;
        }
    }
    
    std::cout << "Initialized: " << model.regularModules.size() << " regular modules, "
              << model.symmetryGroups.size() << " symmetry groups" << std::endl;
}

void PlacementSolver::createInitialSolution() {
    // Initialize ASF-B*-trees for symmetry groups
    for (auto& pair : model.symmetryTrees) {
        createInitialASFBTree(pair.second);
    }
    
    // Create B*-tree for regular modules (if any)
    model.regularTree = nullptr;
    if (!model.regularModules.empty()) {
        // Create a basic B*-tree structure
        // Sort regular modules by area (largest first)
        std::vector<std::pair<std::string, std::shared_ptr<Module>>> sortedModules;
        for (const auto& pair : model.regularModules) {
            sortedModules.push_back(pair);
        }
        
        std::sort(sortedModules.begin(), sortedModules.end(), 
                 [](const auto& a, const auto& b) {
                     return a.second->getArea() > b.second->getArea();
                 });
        
        // Create the tree
        if (!sortedModules.empty()) {
            model.regularTree = std::make_shared<BStarTreeNode>(sortedModules[0].first);
            
            // Add other nodes in a more balanced tree structure
            for (size_t i = 1; i < sortedModules.size(); ++i) {
                auto newNode = std::make_shared<BStarTreeNode>(sortedModules[i].first);
                
                // Find a suitable parent node using BFS
                std::queue<std::shared_ptr<BStarTreeNode>> queue;
                queue.push(model.regularTree);
                
                bool placed = false;
                while (!queue.empty() && !placed) {
                    auto current = queue.front();
                    queue.pop();
                    
                    if (!current->getLeftChild()) {
                        current->setLeftChild(newNode);
                        newNode->setParent(current);
                        placed = true;
                    } else if (!current->getRightChild()) {
                        current->setRightChild(newNode);
                        newNode->setParent(current);
                        placed = true;
                    } else {
                        queue.push(current->getLeftChild());
                        queue.push(current->getRightChild());
                    }
                }
            }
        }
    }
    
    // Create symmetry islands from ASF-B*-trees
    model.symmetryIslands.clear();
    for (const auto& pair : model.symmetryTrees) {
        auto islandBlock = std::make_shared<SymmetryIslandBlock>(
            pair.first, pair.second);
        model.symmetryIslands.push_back(islandBlock);
    }
    
    // Create global B*-tree
    createGlobalBTree();
    
    // Initial packing
    bool packed = PlacementPacker::packSolution(
        model.allModules, model.symmetryTrees, model.symmetryIslands,
        model.globalTree, model.globalNodes);
    
    if (!packed) {
        std::cerr << "Warning: Initial packing failed" << std::endl;
    }
    
    // Calculate initial area
    model.totalArea = PlacementPacker::calculateTotalArea(
        model.allModules, model.solutionWidth, model.solutionHeight);
}

void PlacementSolver::createInitialASFBTree(std::shared_ptr<ASFBStarTree> asfTree) {
    if (!asfTree) return;
    
    // Construct an initial tree that satisfies symmetry constraints
    asfTree->constructInitialTree();
    
    // Pack the ASF-B*-tree
    if (!PlacementPacker::packASFBStarTree(asfTree)) {
        std::cerr << "Warning: Failed to pack initial ASF-B*-tree" << std::endl;
    }
}

void PlacementSolver::createGlobalBTree() {
    // Clear existing tree
    model.globalTree = nullptr;
    model.globalNodes.clear();
    
    // Combine regular modules and symmetry islands
    std::vector<std::pair<std::string, int>> allNodes;
    
    // Add regular modules
    for (const auto& pair : model.regularModules) {
        std::string nodeName = "reg_" + pair.first;
        model.globalNodes[nodeName] = pair.second;
        allNodes.push_back({nodeName, pair.second->getArea()});
    }
    
    // Add symmetry islands
    for (const auto& island : model.symmetryIslands) {
        std::string nodeName = "island_" + island->getName();
        model.globalNodes[nodeName] = island;
        allNodes.push_back({nodeName, island->getArea()});
    }
    
    // Sort by area (largest first)
    std::sort(allNodes.begin(), allNodes.end(),
              [](const auto& a, const auto& b) {
                  return a.second > b.second;
              });
    
    // Create B*-tree in a balanced way
    if (!allNodes.empty()) {
        // Start with the root node (largest area)
        model.globalTree = std::make_shared<BStarTreeNode>(allNodes[0].first);
        
        // Add other nodes level by level
        for (size_t i = 1; i < allNodes.size(); ++i) {
            // Find a suitable parent using BFS
            std::queue<std::shared_ptr<BStarTreeNode>> queue;
            queue.push(model.globalTree);
            
            while (!queue.empty()) {
                auto current = queue.front();
                queue.pop();
                
                if (!current->getLeftChild()) {
                    auto newNode = std::make_shared<BStarTreeNode>(allNodes[i].first);
                    current->setLeftChild(newNode);
                    newNode->setParent(current);
                    break;
                } else if (!current->getRightChild()) {
                    auto newNode = std::make_shared<BStarTreeNode>(allNodes[i].first);
                    current->setRightChild(newNode);
                    newNode->setParent(current);
                    break;
                } else {
                    queue.push(current->getLeftChild());
                    queue.push(current->getRightChild());
                }
            }
        }
    }
}

bool PlacementSolver::performRandomPerturbation() {
    // Choose a perturbation type based on probabilities
    double randVal = uniformDist(rng);
    
    if (randVal < probRotate) {
        // Rotate a module
        if (!model.regularModules.empty() && uniformDist(rng) < 0.5) {
            // Rotate a regular module
            auto it = model.regularModules.begin();
            std::advance(it, uniformDist(rng) * model.regularModules.size());
            
            return PlacementPerturber::rotateModule(it->second);
        } 
        else if (!model.symmetryTrees.empty()) {
            // Rotate a module in a symmetry group
            auto it = model.symmetryTrees.begin();
            std::advance(it, uniformDist(rng) * model.symmetryTrees.size());
            
            // Get a random module from the symmetry group
            auto& modules = it->second->getModules();
            if (!modules.empty()) {
                auto moduleIt = modules.begin();
                std::advance(moduleIt, uniformDist(rng) * modules.size());
                
                return PlacementPerturber::rotateSymmetryModule(it->second, moduleIt->first);
            }
        }
    } 
    else if (randVal < probRotate + probMove) {
        // Try to perturb the global tree
        return PlacementPerturber::perturbGlobalTree(
            model.globalTree, model.globalNodes, rng);
    } 
    else if (randVal < probRotate + probMove + probSwap) {
        // Swap modules
        if (!model.regularModules.empty() && model.regularModules.size() >= 2) {
            // Swap two regular modules
            std::vector<std::string> regularModuleNames;
            for (const auto& pair : model.regularModules) {
                regularModuleNames.push_back(pair.first);
            }
            
            int idx1 = uniformDist(rng) * regularModuleNames.size();
            int idx2;
            do {
                idx2 = uniformDist(rng) * regularModuleNames.size();
            } while (idx2 == idx1);
            
            std::string name1 = regularModuleNames[idx1];
            std::string name2 = regularModuleNames[idx2];
            
            auto module1 = model.regularModules[name1];
            auto module2 = model.regularModules[name2];
            
            return PlacementPerturber::swapModules(module1, module2);
        }
    } 
    else if (randVal < probRotate + probMove + probSwap + probChangeRep) {
        // Change representative in symmetry group
        if (!model.symmetryTrees.empty()) {
            auto it = model.symmetryTrees.begin();
            std::advance(it, uniformDist(rng) * model.symmetryTrees.size());
            
            // Get a random symmetry pair
            auto group = it->second->getSymmetryGroup();
            const auto& pairs = group->getSymmetryPairs();
            if (!pairs.empty()) {
                int pairIndex = uniformDist(rng) * pairs.size();
                const auto& pair = pairs[pairIndex];
                
                // Choose one of the modules in the pair
                std::string moduleName = (uniformDist(rng) < 0.5) ? pair.first : pair.second;
                
                return PlacementPerturber::changeRepresentative(it->second, moduleName);
            }
        }
    } 
    else {
        // Convert symmetry type
        if (!model.symmetryTrees.empty()) {
            auto it = model.symmetryTrees.begin();
            std::advance(it, uniformDist(rng) * model.symmetryTrees.size());
            
            return PlacementPerturber::convertSymmetryType(it->second);
        }
    }
    
    return false;
}

int PlacementSolver::calculateCost(const PlacementModel& model) {
    // For now, just use the area as the cost
    // In a more complex implementation, this would include wirelength
    return model.totalArea;
}

void PlacementSolver::saveSolution(const PlacementModel& source, PlacementModel& destination) {
    // Create a deep copy of the solution
    destination = source.clone();
}

void PlacementSolver::restoreSolution(PlacementModel& destination, const PlacementModel& source) {
    // Restore solution from a saved copy
    destination = source.clone();
}

bool PlacementSolver::validateSolution(const PlacementModel& model) {
    // Check for overlaps
    if (PlacementPacker::hasOverlaps(model.allModules, model.moduleToGroup)) {
        return false;
    }
    
    // Validate symmetry constraints
    if (!PlacementPacker::validateSymmetryConstraints(model.allModules, model.symmetryTrees)) {
        return false;
    }
    
    return true;
}

void PlacementSolver::setAnnealingParameters(double initialTemp, double finalTemp, double coolRate, 
                                           int iterations, int noImprovementLimit) {
    initialTemperature = initialTemp;
    finalTemperature = finalTemp;
    coolingRate = coolRate;
    iterationsPerTemperature = iterations;
    this->noImprovementLimit = noImprovementLimit;
}

void PlacementSolver::setPerturbationProbabilities(double rotate, double move, double swap, 
                                                 double changeRep, double convertSym) {
    // Check if probabilities sum to 1.0
    double sum = rotate + move + swap + changeRep + convertSym;
    if (std::abs(sum - 1.0) > 1e-6) {
        // Normalize probabilities to sum to 1.0
        if (sum <= 0.0) {
            // Default values if all probabilities are zero or negative
            probRotate = 0.4;
            probMove = 0.3;
            probSwap = 0.2;
            probChangeRep = 0.05;
            probConvertSym = 0.05;
            return;
        }
        
        probRotate = rotate / sum;
        probMove = move / sum;
        probSwap = swap / sum;
        probChangeRep = changeRep / sum;
        probConvertSym = convertSym / sum;
    } else {
        probRotate = rotate;
        probMove = move;
        probSwap = swap;
        probChangeRep = changeRep;
        probConvertSym = convertSym;
    }
}

void PlacementSolver::setCostWeights(double area, double wirelength) {
    areaWeight = area;
    wirelengthWeight = wirelength;
}

void PlacementSolver::setRandomSeed(unsigned int seed) {
    randomSeed = seed;
    rng.seed(randomSeed);
}

void PlacementSolver::setTimeLimit(int seconds) {
    timeLimit = seconds;
}

bool PlacementSolver::solve() {
    // Create initial solution
    createInitialSolution();
    
    if (model.allModules.empty()) {
        std::cerr << "Error: No modules to place." << std::endl;
        return false;
    }
    
    // Setup simulated annealing
    std::cout << "Starting simulated annealing..." << std::endl;
    
    // Define cost function
    auto costFunction = [](const PlacementModel& model) -> int {
        // Calculate area (we could add wirelength here in the future)
        int width, height;
        return PlacementPacker::calculateTotalArea(
            model.allModules, width, height);
    };
    
    // Define perturbation function
    auto perturbFunction = [this](PlacementModel& model) -> bool {
        // Perform a random perturbation
        bool success = this->performRandomPerturbation();
        
        // If perturbation succeeded, repack the solution
        if (success) {
            PlacementPacker::packSolution(
                model.allModules, model.symmetryTrees, model.symmetryIslands,
                model.globalTree, model.globalNodes);
        }
        
        return success;
    };
    
    // Define solution save/restore functions
    auto saveFunction = [this](const PlacementModel& source, PlacementModel& destination) {
        this->saveSolution(source, destination);
    };
    
    auto restoreFunction = [this](PlacementModel& destination, const PlacementModel& source) {
        this->restoreSolution(destination, source);
    };
    
    // Define validation function
    auto validateFunction = [this](const PlacementModel& model) -> bool {
        return this->validateSolution(model);
    };
    
    // Create simulated annealing object
    SimulatedAnnealing<PlacementModel, int> sa(
        model, costFunction, perturbFunction, saveFunction, restoreFunction, validateFunction,
        initialTemperature, finalTemperature, coolingRate, 
        iterationsPerTemperature, noImprovementLimit, timeLimit);
    
    // Set random seed
    sa.setSeed(randomSeed);
    
    // Run simulated annealing
    PlacementModel bestSolution = sa.optimize();
    
    // Apply the best solution
    model = bestSolution;
    
    // Final validation
    bool valid = validateSolution(model);
    
    if (!valid) {
        std::cerr << "Warning: Final solution has constraints violations" << std::endl;
    }
    
    // Recalculate area
    model.totalArea = PlacementPacker::calculateTotalArea(
        model.allModules, model.solutionWidth, model.solutionHeight);
    
    std::cout << "Final area: " << model.totalArea << std::endl;
    
    return valid;
}

int PlacementSolver::getSolutionArea() const {
    return model.totalArea;
}

std::map<std::string, std::shared_ptr<Module>> PlacementSolver::getSolutionModules() const {
    return model.allModules;
}

std::map<std::string, int> PlacementSolver::getStatistics() const {
    std::map<std::string, int> stats;
    stats["totalArea"] = model.totalArea;
    stats["width"] = model.solutionWidth;
    stats["height"] = model.solutionHeight;
    stats["numModules"] = model.allModules.size();
    stats["numSymmetryGroups"] = model.symmetryGroups.size();
    stats["numRegularModules"] = model.regularModules.size();
    return stats;
}