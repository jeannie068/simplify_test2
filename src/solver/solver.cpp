#include "solver.hpp"
#include <iostream>
#include <ctime>
#include <algorithm>
#include <limits>
#include <queue>
#include <random>
#include <cmath>
#include <set>

PlacementSolver::PlacementSolver()
    : regularTree(nullptr),
      initialTemperature(1000.0),
      finalTemperature(0.1),
      coolingRate(0.98), // Slower cooling rate for better convergence
      iterationsPerTemperature(300), // More iterations per temperature
      noImprovementLimit(3000), // Longer no improvement limit
      probRotate(0.4),
      probMove(0.3),
      probSwap(0.2),
      probChangeRep(0.05),
      probConvertSym(0.05),
      areaWeight(1.0),
      wirelengthWeight(0.0),
      randomSeed(static_cast<unsigned int>(std::time(nullptr))),
      totalArea(0),
      solutionWidth(0),
      solutionHeight(0),
      timeLimit(290) { // 4 minutes 50 seconds time limit
}

PlacementSolver::~PlacementSolver() {
    // Smart pointers handle cleanup
}

void PlacementSolver::loadProblem(const std::map<std::string, std::shared_ptr<Module>>& modules,
                                const std::vector<std::shared_ptr<SymmetryGroup>>& symmetryGroups) {
    // Store all modules for reference
    allModules = modules;
    this->symmetryGroups = symmetryGroups;
    
    // Initialize module grouping
    initializeModuleGrouping();
}

void PlacementSolver::initializeModuleGrouping() {
    // Clear existing groupings
    regularModules.clear();
    symmetryTrees.clear();
    moduleToGroup.clear();
    
    // Track which modules are part of symmetry groups
    std::unordered_set<std::string> symmetryModules;
    
    // Process symmetry groups
    for (const auto& group : symmetryGroups) {
        // Create a new ASF-B*-tree for this symmetry group
        auto asfTree = std::make_shared<ASFBStarTree>(group);
        symmetryTrees[group->getName()] = asfTree;
        
        // Process symmetry pairs
        for (const auto& pair : group->getSymmetryPairs()) {
            // Add modules to the symmetry group
            if (allModules.find(pair.first) != allModules.end()) {
                asfTree->addModule(allModules[pair.first]);
                symmetryModules.insert(pair.first);
                moduleToGroup[pair.first] = group;
            }
            
            if (allModules.find(pair.second) != allModules.end()) {
                asfTree->addModule(allModules[pair.second]);
                symmetryModules.insert(pair.second);
                moduleToGroup[pair.second] = group;
            }
        }
        
        // Process self-symmetric modules
        for (const auto& moduleName : group->getSelfSymmetric()) {
            if (allModules.find(moduleName) != allModules.end()) {
                asfTree->addModule(allModules[moduleName]);
                symmetryModules.insert(moduleName);
                moduleToGroup[moduleName] = group;
            }
        }
    }
    
    // Identify regular modules (not part of symmetry groups)
    for (const auto& pair : allModules) {
        if (symmetryModules.find(pair.first) == symmetryModules.end()) {
            regularModules[pair.first] = pair.second;
        }
    }
    
    std::cout << "Initialized: " << regularModules.size() << " regular modules, "
              << symmetryGroups.size() << " symmetry groups" << std::endl;
}


int PlacementSolver::calculateTotalArea() {
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
    totalArea = solutionWidth * solutionHeight;
    
    return totalArea;
}

bool PlacementSolver::hasOverlaps() const {
    // Get all modules for overlap checking
    std::vector<std::shared_ptr<Module>> moduleList;
    
    // Collect all modules
    for (const auto& pair : allModules) {
        moduleList.push_back(pair.second);
    }
    
    // Check all pairs for overlaps using a more robust method
    for (size_t i = 0; i < moduleList.size(); ++i) {
        const auto& module1 = moduleList[i];
        
        for (size_t j = i + 1; j < moduleList.size(); ++j) {
            const auto& module2 = moduleList[j];
            
            // Check for overlap using more explicit calculations
            int x1 = module1->getX();
            int y1 = module1->getY();
            int w1 = module1->getWidth();
            int h1 = module1->getHeight();
            
            int x2 = module2->getX();
            int y2 = module2->getY();
            int w2 = module2->getWidth();
            int h2 = module2->getHeight();
            
            // Check if rectangles overlap
            bool overlap = (x1 < x2 + w2) && (x2 < x1 + w1) && 
                          (y1 < y2 + h2) && (y2 < y1 + h1);
            
            if (overlap) {
                std::cerr << "Overlap detected between modules: " 
                         << module1->getName() << " (" << x1 << "," << y1 << "," << w1 << "," << h1 << ") and " 
                         << module2->getName() << " (" << x2 << "," << y2 << "," << w2 << "," << h2 << ")" 
                         << std::endl;
                return true;
            }
        }
    }
    
    return false;
}

bool PlacementSolver::validateSymmetryConstraints() const {
    // Validate each symmetry group
    for (const auto& pair : symmetryTrees) {
        auto symGroup = pair.second->getSymmetryGroup();
        auto& tree = pair.second;
        
        // Check symmetry axis
        double axis = tree->getSymmetryAxisPosition();
        
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
                
                // Verify dimensions are the same
                if (mod1->getWidth() != mod2->getWidth() || mod1->getHeight() != mod2->getHeight()) {
                    std::cerr << "Dimension mismatch for symmetry pair: " 
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
                
                // Verify dimensions are the same
                if (mod1->getWidth() != mod2->getWidth() || mod1->getHeight() != mod2->getHeight()) {
                    std::cerr << "Dimension mismatch for symmetry pair: " 
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
}

void PlacementSolver::setTimeLimit(int seconds) {
    timeLimit = seconds;
}

bool PlacementSolver::solve() {
    // Initialize random generator with provided seed
    std::mt19937 rng(randomSeed);
    std::uniform_real_distribution<double> uniformDist(0.0, 1.0);

    // Start timing
    auto startTime = std::chrono::steady_clock::now();
    
    std::cout << "Starting hierarchical placement..." << std::endl;
    
    // -------------------------------------------------------------------------
    // STEP 1: Process each symmetry group internally with SA
    // -------------------------------------------------------------------------
    std::cout << "Phase 1: Processing symmetry groups internally..." << std::endl;
    
    for (auto& pair : symmetryTrees) {
        std::string groupName = pair.first;
        auto& asfTree = pair.second;
        
        std::cout << "Processing symmetry group: " << groupName << std::endl;
        
        // Run SA just for this symmetry group
        if (!processSymmetryGroupInternally(asfTree, startTime)) {
            std::cerr << "Failed to process symmetry group: " << groupName << std::endl;
            // Continue with next group instead of failing completely
        }
        
        // Validate the symmetry group placement
        if (!validateSymmetryGroup(asfTree)) {
            std::cerr << "Warning: Symmetry group " << groupName << " has validation issues" << std::endl;
            
            // Try to fix the symmetry group by re-packing it
            asfTree->constructInitialTree();
            if (!asfTree->pack()) {
                std::cerr << "Error: Failed to re-pack symmetry group " << groupName << std::endl;
            }
        }
        
        // Finalize the symmetry group's contour
        finalizeSymmetryGroupContour(asfTree);
    }
    
    // -------------------------------------------------------------------------
    // STEP 2: Place regular modules and finalized symmetry groups
    // -------------------------------------------------------------------------
    std::cout << "Phase 2: Combined placement of symmetry groups and regular modules..." << std::endl;
    
    // Create initial solution for regular modules
    createInitialRegularSolution();
    
    // Calculate initial area
    int currentArea = calculateTotalArea();
    int bestArea = currentArea;
    
    // Save best solution
    std::map<std::string, std::pair<int, int>> bestPositions;
    std::map<std::string, bool> bestRotations;
    
    // Save positions and rotations of regular modules only
    for (const auto& pair : regularModules) {
        const auto& module = pair.second;
        bestPositions[pair.first] = {module->getX(), module->getY()};
        bestRotations[pair.first] = module->getRotated();
    }
    
    // Save positions of symmetry groups (as single units)
    for (const auto& pair : symmetryGroups) {
        const auto& group = pair;
        
        // Find a representative module from the group
        for (const auto& symPair : group->getSymmetryPairs()) {
            auto it = allModules.find(symPair.first);
            if (it != allModules.end()) {
                bestPositions[group->getName()] = {it->second->getX(), it->second->getY()};
                break;
            }
        }
    }
    
    std::cout << "Initial combined area: " << currentArea << std::endl;
    
    // SA parameters
    double temperature = initialTemperature;
    int noImprovementCount = 0;
    int totalIterations = 0;
    int acceptedMoves = 0;
    int rejectedMoves = 0;
    
    // Main SA loop for combined placement
    while (temperature > finalTemperature && 
           noImprovementCount < noImprovementLimit) {
        
        // Check time limit
        auto currentTime = std::chrono::steady_clock::now();
        auto elapsedSeconds = std::chrono::duration_cast<std::chrono::seconds>(
            currentTime - startTime).count();
        
        if (elapsedSeconds >= timeLimit) {
            std::cout << "Time limit reached, stopping SA..." << std::endl;
            break;
        }
        
        bool improved = false;
        int acceptedInPassCount = 0;
        
        // Perform iterations at current temperature
        for (int i = 0; i < iterationsPerTemperature; ++i) {
            totalIterations++;
            
            // Save current state before perturbation
            std::map<std::string, std::pair<int, int>> currentPositions;
            std::map<std::string, bool> currentRotations;
            
            // Store the current state of regular modules
            for (const auto& pair : regularModules) {
                const auto& module = pair.second;
                currentPositions[pair.first] = {module->getX(), module->getY()};
                currentRotations[pair.first] = module->getRotated();
            }
            
            // Store current positions of symmetry groups
            for (const auto& pair : symmetryTrees) {
                // Get the bounding box of the symmetry group
                int minX = std::numeric_limits<int>::max();
                int minY = std::numeric_limits<int>::max();
                
                for (const auto& modPair : pair.second->getModules()) {
                    auto module = modPair.second;
                    minX = std::min(minX, module->getX());
                    minY = std::min(minY, module->getY());
                }
                
                currentPositions[pair.first] = {minX, minY};
            }
            
            // Perform a perturbation on the combined placement
            bool perturbSuccess = perturbCombinedPlacement(rng, uniformDist);
            
            if (!perturbSuccess) {
                continue;
            }
            
            // Pack the solution and check if it's valid
            bool packSuccess = packCombinedSolution();
            
            if (packSuccess && !hasOverlaps() && validateSymmetryConstraints()) {
                // Calculate new area
                int newArea = calculateTotalArea();
                int deltaCost = newArea - currentArea;
                
                // Decide whether to accept
                bool accept = false;
                if (deltaCost <= 0) {
                    // Always accept improvements
                    accept = true;
                    if (newArea < bestArea) {
                        improved = true;
                        bestArea = newArea;
                        
                        // Update best solution
                        for (const auto& pair : regularModules) {
                            const auto& module = pair.second;
                            bestPositions[pair.first] = {module->getX(), module->getY()};
                            bestRotations[pair.first] = module->getRotated();
                        }
                        
                        // Save positions of symmetry groups
                        for (const auto& pair : symmetryTrees) {
                            // Get the bounding box of the symmetry group
                            int minX = std::numeric_limits<int>::max();
                            int minY = std::numeric_limits<int>::max();
                            
                            for (const auto& modPair : pair.second->getModules()) {
                                auto module = modPair.second;
                                minX = std::min(minX, module->getX());
                                minY = std::min(minY, module->getY());
                            }
                            
                            bestPositions[pair.first] = {minX, minY};
                        }
                        
                        // Reset no improvement counter
                        noImprovementCount = 0;
                    }
                } else {
                    // Accept worsening moves with probability based on temperature
                    double acceptProb = std::exp(-deltaCost / temperature);
                    accept = uniformDist(rng) < acceptProb;
                }
                
                if (accept) {
                    // Move accepted
                    currentArea = newArea;
                    acceptedMoves++;
                    acceptedInPassCount++;
                } else {
                    // Move rejected, restore previous state
                    rejectedMoves++;
                    
                    // Restore regular module positions and rotations
                    for (auto& pair : regularModules) {
                        auto module = pair.second;
                        const auto& oldPos = currentPositions[pair.first];
                        bool oldRot = currentRotations[pair.first];
                        
                        module->setPosition(oldPos.first, oldPos.second);
                        module->setRotation(oldRot);
                    }
                    
                    // Restore symmetry group positions
                    for (auto& pair : symmetryTrees) {
                        const auto& oldPos = currentPositions[pair.first];
                        int deltaX, deltaY;
                        
                        // Calculate the current position to determine delta
                        int currentMinX = std::numeric_limits<int>::max();
                        int currentMinY = std::numeric_limits<int>::max();
                        
                        for (const auto& modPair : pair.second->getModules()) {
                            auto module = modPair.second;
                            currentMinX = std::min(currentMinX, module->getX());
                            currentMinY = std::min(currentMinY, module->getY());
                        }
                        
                        // Calculate deltas
                        deltaX = oldPos.first - currentMinX;
                        deltaY = oldPos.second - currentMinY;
                        
                        // Apply the delta to all modules in the group
                        for (auto& modPair : pair.second->getModules()) {
                            auto module = modPair.second;
                            module->setPosition(
                                module->getX() + deltaX,
                                module->getY() + deltaY
                            );
                        }
                    }
                    
                    // Recalculate area
                    calculateTotalArea();
                }
            } else {
                // Invalid placement, restore previous state
                rejectedMoves++;
                
                // Restore regular module positions and rotations
                for (auto& pair : regularModules) {
                    auto module = pair.second;
                    const auto& oldPos = currentPositions[pair.first];
                    bool oldRot = currentRotations[pair.first];
                    
                    module->setPosition(oldPos.first, oldPos.second);
                    module->setRotation(oldRot);
                }
                
                // Restore symmetry group positions
                for (auto& pair : symmetryTrees) {
                    const auto& oldPos = currentPositions[pair.first];
                    int deltaX, deltaY;
                    
                    // Calculate the current position to determine delta
                    int currentMinX = std::numeric_limits<int>::max();
                    int currentMinY = std::numeric_limits<int>::max();
                    
                    for (const auto& modPair : pair.second->getModules()) {
                        auto module = modPair.second;
                        currentMinX = std::min(currentMinX, module->getX());
                        currentMinY = std::min(currentMinY, module->getY());
                    }
                    
                    // Calculate deltas
                    deltaX = oldPos.first - currentMinX;
                    deltaY = oldPos.second - currentMinY;
                    
                    // Apply the delta to all modules in the group
                    for (auto& modPair : pair.second->getModules()) {
                        auto module = modPair.second;
                        module->setPosition(
                            module->getX() + deltaX,
                            module->getY() + deltaY
                        );
                    }
                }
                
                // Restore area
                calculateTotalArea();
            }
            
            if (!improved) {
                noImprovementCount++;
            }
            
            // Check if we've reached the limit
            if (noImprovementCount >= noImprovementLimit) {
                break;
            }
        }
        
        // Calculate acceptance ratio
        double acceptanceRatio = static_cast<double>(acceptedInPassCount) / iterationsPerTemperature;
        
        // Adaptive cooling schedule
        if (acceptanceRatio > 0.8) {
            temperature *= coolingRate * 0.9;
        } else if (acceptanceRatio < 0.1) {
            temperature *= coolingRate * 1.1;
            if (temperature > initialTemperature) {
                temperature = initialTemperature;
            }
        } else {
            temperature *= coolingRate;
        }
        
        std::cout << "Temperature: " << temperature 
                  << ", Best area: " << bestArea 
                  << ", Current area: " << currentArea 
                  << ", No improvement: " << noImprovementCount 
                  << ", Acceptance ratio: " << acceptanceRatio 
                  << std::endl;
    }
    
    // Restore best solution
    // Restore regular module positions
    for (auto& pair : regularModules) {
        auto module = pair.second;
        auto it = bestPositions.find(pair.first);
        if (it != bestPositions.end()) {
            const auto& bestPos = it->second;
            bool bestRot = bestRotations[pair.first];
            
            module->setPosition(bestPos.first, bestPos.second);
            module->setRotation(bestRot);
        }
    }
    
    // Restore symmetry group positions
    for (auto& pair : symmetryTrees) {
        auto it = bestPositions.find(pair.first);
        if (it != bestPositions.end()) {
            const auto& bestPos = it->second;
            
            // Calculate the current position to determine delta
            int currentMinX = std::numeric_limits<int>::max();
            int currentMinY = std::numeric_limits<int>::max();
            
            for (const auto& modPair : pair.second->getModules()) {
                auto module = modPair.second;
                currentMinX = std::min(currentMinX, module->getX());
                currentMinY = std::min(currentMinY, module->getY());
            }
            
            // Calculate deltas
            int deltaX = bestPos.first - currentMinX;
            int deltaY = bestPos.second - currentMinY;
            
            // Apply the delta to all modules in the group
            for (auto& modPair : pair.second->getModules()) {
                auto module = modPair.second;
                module->setPosition(
                    module->getX() + deltaX,
                    module->getY() + deltaY
                );
            }
        }
    }
    
    // Final validation
    bool finalValid = !hasOverlaps() && validateSymmetryConstraints();
    
    if (!finalValid) {
        std::cerr << "Warning: Final solution has issues, attempting to fix..." << std::endl;
        
        // Try to fix each symmetry group
        for (auto& pair : symmetryTrees) {
            pair.second->pack();
        }
        
        // Try to pack everything once more
        packCombinedSolution();
        
        finalValid = !hasOverlaps() && validateSymmetryConstraints();
        
        if (!finalValid) {
            std::cerr << "Warning: Could not fix all issues in final solution" << std::endl;
        }
    }
    
    // Recalculate the total area
    totalArea = calculateTotalArea();
    
    std::cout << "Final area: " << totalArea << std::endl;
    
    return true;
}

/**
 * Process a single symmetry group with internal SA
 */
bool PlacementSolver::processSymmetryGroupInternally (
    std::shared_ptr<ASFBStarTree> asfTree,
    const std::chrono::steady_clock::time_point& startTime) {
    
    // Initialize the tree if not already done
    if (!asfTree->getRoot()) {
        asfTree->constructInitialTree();
    }
    
    // Initialize random generator
    std::mt19937 rng(randomSeed);
    std::uniform_real_distribution<double> uniformDist(0.0, 1.0);
    
    // Pack the initial solution
    if (!asfTree->pack()) {
        std::cerr << "Error: Initial packing of symmetry group failed" << std::endl;
        return false;
    }
    
    // Calculate initial area
    int currentArea = asfTree->getArea();
    int bestArea = currentArea;
    
    // Save best solution
    std::map<std::string, std::tuple<int, int, bool>> bestModuleState;
    for (const auto& pair : asfTree->getModules()) {
        const auto& module = pair.second;
        bestModuleState[pair.first] = std::make_tuple(
            module->getX(), module->getY(), module->getRotated());
    }
    
    // Tracking variables
    double temperature = initialTemperature;
    int noImprovementCount = 0;
    int totalIterations = 0;
    
    // Main SA loop for internal symmetry group optimization
    while (temperature > finalTemperature && noImprovementCount < noImprovementLimit) {
        // Check time limit
        auto currentTime = std::chrono::steady_clock::now();
        auto elapsedSeconds = std::chrono::duration_cast<std::chrono::seconds>(
            currentTime - startTime).count();
        
        if (elapsedSeconds >= timeLimit / 2) { // Use half the time for phase 1
            std::cout << "Time limit for internal optimization reached" << std::endl;
            break;
        }
        
        int acceptedInThisPass = 0;
        
        // Iterations at current temperature
        for (int i = 0; i < iterationsPerTemperature / 2; ++i) { // Fewer iterations for phase 1
            totalIterations++;
            
            // Save current state
            std::map<std::string, std::tuple<int, int, bool>> currentModuleState;
            for (const auto& pair : asfTree->getModules()) {
                const auto& module = pair.second;
                currentModuleState[pair.first] = std::make_tuple(
                    module->getX(), module->getY(), module->getRotated());
            }
            
            // Determine perturbation type
            double rand = uniformDist(rng);
            bool perturbSuccess = false;
            
            if (rand < 0.3) {
                // Rotate a random module
                perturbSuccess = perturbRotateInSymmetryGroup(asfTree, rng, uniformDist);
            } else if (rand < 0.65) {
                // Pre-order based perturbation
                perturbSuccess = perturbPreOrderInSymmetryGroup(asfTree, rng, uniformDist);
            } else {
                // In-order based perturbation
                perturbSuccess = perturbInOrderInSymmetryGroup(asfTree, rng, uniformDist);
            }
            
            if (!perturbSuccess) {
                continue;
            }
            
            // Pack the tree and validate
            if (!asfTree->pack() || !asfTree->isSymmetricFeasible()) {
                // Invalid perturbation, restore state
                for (auto& pair : asfTree->getModules()) {
                    auto module = pair.second;
                    auto state = currentModuleState[pair.first];
                    module->setPosition(std::get<0>(state), std::get<1>(state));
                    module->setRotation(std::get<2>(state));
                }
                continue;
            }
            
            // Calculate new area
            int newArea = asfTree->getArea();
            int deltaCost = newArea - currentArea;
            
            // Decide whether to accept
            bool accept = false;
            if (deltaCost <= 0) {
                // Accept improvement
                accept = true;
                if (newArea < bestArea) {
                    bestArea = newArea;
                    noImprovementCount = 0;
                    
                    // Update best solution
                    for (const auto& pair : asfTree->getModules()) {
                        const auto& module = pair.second;
                        bestModuleState[pair.first] = std::make_tuple(
                            module->getX(), module->getY(), module->getRotated());
                    }
                } else {
                    noImprovementCount++;
                }
            } else {
                // Accept with probability
                double acceptProb = std::exp(-deltaCost / temperature);
                accept = uniformDist(rng) < acceptProb;
                noImprovementCount++;
            }
            
            if (accept) {
                currentArea = newArea;
                acceptedInThisPass++;
            } else {
                // Rejected, restore state
                for (auto& pair : asfTree->getModules()) {
                    auto module = pair.second;
                    auto state = currentModuleState[pair.first];
                    module->setPosition(std::get<0>(state), std::get<1>(state));
                    module->setRotation(std::get<2>(state));
                }
            }
            
            if (noImprovementCount >= noImprovementLimit) {
                break;
            }
        }
        
        // Calculate acceptance ratio
        double acceptanceRatio = (double)acceptedInThisPass / (iterationsPerTemperature / 2);
        
        // Update temperature
        if (acceptanceRatio > 0.8) {
            temperature *= coolingRate * 0.9;
        } else if (acceptanceRatio < 0.1) {
            temperature *= coolingRate * 1.1;
        } else {
            temperature *= coolingRate;
        }
        
        std::cout << "    Symmetry group - Temp: " << temperature 
                 << ", Area: " << currentArea 
                 << ", Best: " << bestArea 
                 << ", AcceptRatio: " << acceptanceRatio 
                 << std::endl;
    }
    
    // Restore best solution
    for (auto& pair : asfTree->getModules()) {
        auto module = pair.second;
        auto state = bestModuleState[pair.first];
        module->setPosition(std::get<0>(state), std::get<1>(state));
        module->setRotation(std::get<2>(state));
    }
    
    // Final packing
    if (!asfTree->pack()) {
        std::cerr << "Error: Final packing of symmetry group failed" << std::endl;
        return false;
    }
    
    return true;
}

/**
 * Finalizes a symmetry group's contour after internal optimization
 */
void PlacementSolver::finalizeSymmetryGroupContour(std::shared_ptr<ASFBStarTree> asfTree) {
    // Find the bounding rectangle of the symmetry group
    int minX = std::numeric_limits<int>::max();
    int minY = std::numeric_limits<int>::max();
    int maxX = 0;
    int maxY = 0;
    
    for (const auto& pair : asfTree->getModules()) {
        const auto& module = pair.second;
        minX = std::min(minX, module->getX());
        minY = std::min(minY, module->getY());
        maxX = std::max(maxX, module->getX() + module->getWidth());
        maxY = std::max(maxY, module->getY() + module->getHeight());
    }
    
    // Shift all modules in the group to start at (0,0)
    for (auto& pair : asfTree->getModules()) {
        auto module = pair.second;
        module->setPosition(
            module->getX() - minX,
            module->getY() - minY
        );
    }
    
    // If necessary, update the symmetry axis position
    auto symmetryGroup = asfTree->getSymmetryGroup();
    if (symmetryGroup->getType() == SymmetryType::VERTICAL) {
        double oldAxis = symmetryGroup->getAxisPosition();
        symmetryGroup->setAxisPosition(oldAxis - minX);
    } else { // HORIZONTAL
        double oldAxis = symmetryGroup->getAxisPosition();
        symmetryGroup->setAxisPosition(oldAxis - minY);
    }
    
    // Re-pack the tree to update contours
    asfTree->pack();
}

/**
 * Creates initial solution for just regular modules
 */
void PlacementSolver::createInitialRegularSolution() {
    // Create B*-tree for regular modules (if any)
    regularTree = nullptr;
    
    if (!regularModules.empty()) {
        // Sort regular modules by area (largest first)
        std::vector<std::pair<std::string, std::shared_ptr<Module>>> sortedModules;
        for (const auto& pair : regularModules) {
            sortedModules.push_back(pair);
        }
        
        std::sort(sortedModules.begin(), sortedModules.end(), 
                 [](const auto& a, const auto& b) {
                     return a.second->getArea() > b.second->getArea();
                 });
        
        // Create a balanced tree structure
        if (!sortedModules.empty()) {
            regularTree = std::make_shared<BStarTreeNode>(sortedModules[0].first);
            
            if (sortedModules.size() > 1) {
                // Create a more balanced tree
                for (size_t i = 1; i < sortedModules.size(); ++i) {
                    auto newNode = std::make_shared<BStarTreeNode>(sortedModules[i].first);
                    
                    // Use BFS to find a node with available child slot
                    std::queue<std::shared_ptr<BStarTreeNode>> nodeQueue;
                    nodeQueue.push(regularTree);
                    
                    bool placed = false;
                    while (!nodeQueue.empty() && !placed) {
                        auto current = nodeQueue.front();
                        nodeQueue.pop();
                        
                        if (!current->getLeftChild()) {
                            current->setLeftChild(newNode);
                            newNode->setParent(current);
                            placed = true;
                        } else if (!current->getRightChild()) {
                            current->setRightChild(newNode);
                            newNode->setParent(current);
                            placed = true;
                        } else {
                            nodeQueue.push(current->getLeftChild());
                            nodeQueue.push(current->getRightChild());
                        }
                    }
                }
            }
        }
    }
}

/**
 * Packs the combined solution (symmetry groups + regular modules)
 */
bool PlacementSolver::packCombinedSolution() {
    int currentY = 0;
    
    // First, pack all symmetry groups while preserving their internal structure
    for (auto& pair : symmetryTrees) {
        // The internal structure of the symmetry group is already fixed
        // Just position the entire group
        int minX = std::numeric_limits<int>::max();
        int minY = std::numeric_limits<int>::max();
        int maxX = 0;
        int maxY = 0;
        
        // Find the current bounding box
        for (const auto& modPair : pair.second->getModules()) {
            auto module = modPair.second;
            minX = std::min(minX, module->getX());
            minY = std::min(minY, module->getY());
            maxX = std::max(maxX, module->getX() + module->getWidth());
            maxY = std::max(maxY, module->getY() + module->getHeight());
        }
        
        // Calculate shift amount to place at (0, currentY)
        int deltaX = -minX;
        int deltaY = currentY - minY;
        
        // Update symmetry axis position
        auto symGroup = pair.second->getSymmetryGroup();
        if (symGroup->getType() == SymmetryType::VERTICAL) {
            double oldAxis = symGroup->getAxisPosition();
            symGroup->setAxisPosition(oldAxis + deltaX);
        } else {
            double oldAxis = symGroup->getAxisPosition();
            symGroup->setAxisPosition(oldAxis + deltaY);
        }
        
        // Shift all modules in the group
        for (auto& modPair : pair.second->getModules()) {
            auto module = modPair.second;
            module->setPosition(
                module->getX() + deltaX,
                module->getY() + deltaY
            );
        }
        
        // Update current Y for next group with spacing
        currentY = (maxY - minY) + currentY + 10; // 10 units spacing
    }
    
    // Now place regular modules
    if (!regularModules.empty() && regularTree) {
        // Use a simple B*-tree packing algorithm
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
            
            int x = 0, y = currentY;
            
            // Calculate position based on B*-tree rules
            if (currentNode->getParent()) {
                auto parentName = currentNode->getParent()->getModuleName();
                auto parentIt = regularModules.find(parentName);
                if (parentIt != regularModules.end()) {
                    auto parentModule = parentIt->second;
                    
                    if (currentNode->isLeftChild()) {
                        // Left child: place to the right of parent
                        x = parentModule->getX() + parentModule->getWidth();
                        y = parentModule->getY();
                    } else {
                        // Right child: place above parent
                        x = parentModule->getX();
                        y = parentModule->getY() + parentModule->getHeight();
                    }
                }
            }
            
            // Set the module's position
            module->setPosition(x, y);
            
            // Add children to the queue
            if (currentNode->getLeftChild()) {
                nodeQueue.push(currentNode->getLeftChild());
            }
            if (currentNode->getRightChild()) {
                nodeQueue.push(currentNode->getRightChild());
            }
        }
    }
    
    return true;
}

/**
 * Perturbs the combined placement (symmetry groups and regular modules)
 */
bool PlacementSolver::perturbCombinedPlacement (
    std::mt19937& rng, 
    std::uniform_real_distribution<double>& uniformDist) {
    
    // Decide whether to perturb a regular module or a symmetry group
    bool perturbRegular = !regularModules.empty() && 
                         (symmetryTrees.empty() || uniformDist(rng) < 0.5);
    
    if (perturbRegular) {
        // Perturb regular modules
        double randVal = uniformDist(rng);
        
        if (randVal < 0.5) {
            // Rotate a regular module
            int index = static_cast<int>(uniformDist(rng) * regularModules.size());
            auto it = regularModules.begin();
            std::advance(it, index);
            
            it->second->rotate();
            return true;
        } else {
            // Move a regular module
            // For simplicity, just swap two modules
            if (regularModules.size() >= 2) {
                int idx1 = static_cast<int>(uniformDist(rng) * regularModules.size());
                int idx2;
                do {
                    idx2 = static_cast<int>(uniformDist(rng) * regularModules.size());
                } while (idx2 == idx1);
                
                auto it1 = regularModules.begin();
                auto it2 = regularModules.begin();
                std::advance(it1, idx1);
                std::advance(it2, idx2);
                
                // Swap positions
                auto module1 = it1->second;
                auto module2 = it2->second;
                
                int tmpX = module1->getX();
                int tmpY = module1->getY();
                module1->setPosition(module2->getX(), module2->getY());
                module2->setPosition(tmpX, tmpY);
                
                return true;
            }
        }
    } else if (!symmetryTrees.empty()) {
        // Perturb symmetry groups
        // For simplicity, we'll just move symmetry groups around
        
        // Get a list of symmetry groups
        std::vector<std::string> groupNames;
        for (const auto& pair : symmetryTrees) {
            groupNames.push_back(pair.first);
        }
        
        if (groupNames.size() >= 2) {
            // Swap two symmetry groups
            int idx1 = static_cast<int>(uniformDist(rng) * groupNames.size());
            int idx2;
            do {
                idx2 = static_cast<int>(uniformDist(rng) * groupNames.size());
            } while (idx2 == idx1);
            
            auto& group1 = symmetryTrees[groupNames[idx1]];
            auto& group2 = symmetryTrees[groupNames[idx2]];
            
            // Find bounding boxes
            int minX1 = std::numeric_limits<int>::max();
            int minY1 = std::numeric_limits<int>::max();
            int minX2 = std::numeric_limits<int>::max();
            int minY2 = std::numeric_limits<int>::max();
            
            for (const auto& pair : group1->getModules()) {
                auto module = pair.second;
                minX1 = std::min(minX1, module->getX());
                minY1 = std::min(minY1, module->getY());
            }
            
            for (const auto& pair : group2->getModules()) {
                auto module = pair.second;
                minX2 = std::min(minX2, module->getX());
                minY2 = std::min(minY2, module->getY());
            }
            
            // Calculate deltas
            int deltaX1 = minX2 - minX1;
            int deltaY1 = minY2 - minY1;
            int deltaX2 = minX1 - minX2;
            int deltaY2 = minY1 - minY2;
            
            // Shift group 1
            for (auto& pair : group1->getModules()) {
                auto module = pair.second;
                module->setPosition(
                    module->getX() + deltaX1,
                    module->getY() + deltaY1
                );
            }
            
            // Update symmetry axis for group 1
            auto symGroup1 = group1->getSymmetryGroup();
            if (symGroup1->getType() == SymmetryType::VERTICAL) {
                double oldAxis = symGroup1->getAxisPosition();
                symGroup1->setAxisPosition(oldAxis + deltaX1);
            } else {
                double oldAxis = symGroup1->getAxisPosition();
                symGroup1->setAxisPosition(oldAxis + deltaY1);
            }
            
            // Shift group 2
            for (auto& pair : group2->getModules()) {
                auto module = pair.second;
                module->setPosition(
                    module->getX() + deltaX2,
                    module->getY() + deltaY2
                );
            }
            
            // Update symmetry axis for group 2
            auto symGroup2 = group2->getSymmetryGroup();
            if (symGroup2->getType() == SymmetryType::VERTICAL) {
                double oldAxis = symGroup2->getAxisPosition();
                symGroup2->setAxisPosition(oldAxis + deltaX2);
            } else {
                double oldAxis = symGroup2->getAxisPosition();
                symGroup2->setAxisPosition(oldAxis + deltaY2);
            }
            
            return true;
        } else if (groupNames.size() == 1 && !regularModules.empty()) {
            // Move the single symmetry group relative to regular modules
            auto& group = symmetryTrees[groupNames[0]];
            
            // Randomly select a regular module
            int modIdx = static_cast<int>(uniformDist(rng) * regularModules.size());
            auto modIt = regularModules.begin();
            std::advance(modIt, modIdx);
            auto regModule = modIt->second;
            
            // Find bounding box of the group
            int minX = std::numeric_limits<int>::max();
            int minY = std::numeric_limits<int>::max();
            
            for (const auto& pair : group->getModules()) {
                auto module = pair.second;
                minX = std::min(minX, module->getX());
                minY = std::min(minY, module->getY());
            }
            
            // Calculate deltas to move group next to the regular module
            int deltaX = regModule->getX() + regModule->getWidth() - minX;
            int deltaY = regModule->getY() - minY;
            
            // Shift the group
            for (auto& pair : group->getModules()) {
                auto module = pair.second;
                module->setPosition(
                    module->getX() + deltaX,
                    module->getY() + deltaY
                );
            }
            
            // Update symmetry axis
            auto symGroup = group->getSymmetryGroup();
            if (symGroup->getType() == SymmetryType::VERTICAL) {
                double oldAxis = symGroup->getAxisPosition();
                symGroup->setAxisPosition(oldAxis + deltaX);
            } else {
                double oldAxis = symGroup->getAxisPosition();
                symGroup->setAxisPosition(oldAxis + deltaY);
            }
            
            return true;
        }
    }
    
    return false;
}

/**
 * Validates a single symmetry group
 */
bool PlacementSolver::validateSymmetryGroup(std::shared_ptr<ASFBStarTree> asfTree) {
    return asfTree->isSymmetricFeasible() && !hasOverlapsInSymmetryGroup(asfTree);
}

/**
 * Checks for overlaps within a symmetry group
 */
bool PlacementSolver::hasOverlapsInSymmetryGroup(std::shared_ptr<ASFBStarTree> asfTree) {
    const auto& modules = asfTree->getModules();
    
    // Check all pairs of modules for overlaps
    for (auto it1 = modules.begin(); it1 != modules.end(); ++it1) {
        auto it2 = it1;
        ++it2;
        
        for (; it2 != modules.end(); ++it2) {
            if (it1->second->overlaps(*it2->second)) {
                std::cerr << "Overlap detected within symmetry group between modules: " 
                         << it1->first << " and " << it2->first << std::endl;
                return true;
            }
        }
    }
    
    return false;
}

/**
 * Performs rotation perturbation within a symmetry group
 */
bool PlacementSolver::perturbRotateInSymmetryGroup(
    std::shared_ptr<ASFBStarTree> asfTree,
    std::mt19937& rng,
    std::uniform_real_distribution<double>& uniformDist) {
    
    // Select a random module from the symmetry group
    const auto& modules = asfTree->getModules();
    if (modules.empty()) return false;
    
    std::vector<std::string> moduleNames;
    for (const auto& pair : modules) {
        moduleNames.push_back(pair.first);
    }
    
    int index = static_cast<int>(uniformDist(rng) * moduleNames.size());
    std::string moduleName = moduleNames[index];
    
    // Use the ASFBStarTree's rotateModule method which handles symmetry
    return asfTree->rotateModule(moduleName);
}

/**
 * Performs pre-order perturbation within a symmetry group
 */
bool PlacementSolver::perturbPreOrderInSymmetryGroup(
    std::shared_ptr<ASFBStarTree> asfTree,
    std::mt19937& rng,
    std::uniform_real_distribution<double>& uniformDist) {
    
    auto root = asfTree->getRoot();
    if (!root) return false;
    
    // Collect nodes in pre-order
    std::vector<std::shared_ptr<BStarTreeNode>> nodes;
    collectNodesPreOrder(root, nodes);
    
    if (nodes.size() < 2) return false;
    
    // Select two random positions
    int pos1 = static_cast<int>(uniformDist(rng) * nodes.size());
    int pos2;
    do {
        pos2 = static_cast<int>(uniformDist(rng) * nodes.size());
    } while (pos1 == pos2);
    
    auto node1 = nodes[pos1];
    auto node2 = nodes[pos2];
    
    // Check if both nodes represent modules in the same symmetry group
    if (!asfTree->getModules().count(node1->getModuleName()) ||
        !asfTree->getModules().count(node2->getModuleName())) {
        return false;
    }
    
    // Check if swapping would violate symmetry constraints
    bool node1IsSelf = asfTree->isOnBoundary(node1->getModuleName());
    bool node2IsSelf = asfTree->isOnBoundary(node2->getModuleName());
    
    // Only swap if both are self-symmetric or both are not
    if (node1IsSelf == node2IsSelf) {
        // Use ASFBStarTree's swapNodes method which maintains symmetry
        return asfTree->swapNodes(node1->getModuleName(), node2->getModuleName());
    }
    
    return false;
}

/**
 * Performs in-order perturbation within a symmetry group
 */
bool PlacementSolver::perturbInOrderInSymmetryGroup(
    std::shared_ptr<ASFBStarTree> asfTree,
    std::mt19937& rng,
    std::uniform_real_distribution<double>& uniformDist) {
    
    auto root = asfTree->getRoot();
    if (!root) return false;
    
    // Collect nodes in in-order
    std::vector<std::shared_ptr<BStarTreeNode>> nodes;
    collectNodesInOrder(root, nodes);
    
    if (nodes.size() < 2) return false;
    
    // Select two random positions
    int pos1 = static_cast<int>(uniformDist(rng) * nodes.size());
    int pos2;
    do {
        pos2 = static_cast<int>(uniformDist(rng) * nodes.size());
    } while (pos1 == pos2);
    
    auto node1 = nodes[pos1];
    auto node2 = nodes[pos2];
    
    // Check if both nodes represent modules in the same symmetry group
    if (!asfTree->getModules().count(node1->getModuleName()) ||
        !asfTree->getModules().count(node2->getModuleName())) {
        return false;
    }
    
    // Check if swapping would violate symmetry constraints
    bool node1IsSelf = asfTree->isOnBoundary(node1->getModuleName());
    bool node2IsSelf = asfTree->isOnBoundary(node2->getModuleName());
    
    // Only swap if both are self-symmetric or both are not
    if (node1IsSelf == node2IsSelf) {
        // Use ASFBStarTree's swapNodes method which maintains symmetry
        return asfTree->swapNodes(node1->getModuleName(), node2->getModuleName());
    }
    
    return false;
}

/**
 * Helper function to collect nodes in pre-order traversal
 */
void PlacementSolver::collectNodesPreOrder(
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

/**
 * Helper function to collect nodes in in-order traversal
 */
void PlacementSolver::collectNodesInOrder(
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

int PlacementSolver::getSolutionArea() const {
    return totalArea;
}

std::map<std::string, std::shared_ptr<Module>> PlacementSolver::getSolutionModules() const {
    return allModules;
}

std::map<std::string, int> PlacementSolver::getStatistics() const {
    std::map<std::string, int> stats;
    stats["totalArea"] = totalArea;
    stats["width"] = solutionWidth;
    stats["height"] = solutionHeight;
    return stats;
}