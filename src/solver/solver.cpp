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

void PlacementSolver::createInitialSolution() {
    // Initialize ASF-B*-trees for symmetry groups
    for (auto& pair : symmetryTrees) {
        pair.second->constructInitialTree();
    }
    
    // Create B*-tree for regular modules (if any)
    regularTree = nullptr;
    if (!regularModules.empty()) {
        // Create a basic B*-tree structure
        // Sort regular modules by area (largest first)
        std::vector<std::pair<std::string, std::shared_ptr<Module>>> sortedModules;
        for (const auto& pair : regularModules) {
            sortedModules.push_back(pair);
        }
        
        std::sort(sortedModules.begin(), sortedModules.end(), 
                 [](const auto& a, const auto& b) {
                     return a.second->getArea() > b.second->getArea();
                 });
        
        // Create the tree - modified approach for better initial placement
        if (!sortedModules.empty()) {
            regularTree = std::make_shared<BStarTreeNode>(sortedModules[0].first);
            
            if (sortedModules.size() > 1) {
                // Create a more balanced tree structure (not just left-skewed)
                for (size_t i = 1; i < sortedModules.size(); ++i) {
                    // Find a suitable parent node
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
                    
                    // If not placed yet (shouldn't happen), add as a left child to some leaf
                    if (!placed) {
                        auto current = regularTree;
                        while (current->getLeftChild()) {
                            current = current->getLeftChild();
                        }
                        current->setLeftChild(newNode);
                        newNode->setParent(current);
                    }
                }
            }
        }
    }
    
    // Initial packing
    packSolution();
}

bool PlacementSolver::packSolution() {
    // Pack all symmetry islands first, and arrange them in a grid-like pattern
    std::vector<std::pair<std::shared_ptr<SymmetryGroup>, std::pair<int, int>>> groupSizes;
    
    // First, pack each symmetry island in place (0, 0) to get their dimensions
    for (auto& pair : symmetryTrees) {
        // Pack the symmetry island
        if (!pair.second->pack()) {
            std::cerr << "Failed to pack symmetry group: " << pair.first << std::endl;
            return false;
        }
        
        // Calculate bounding box of the symmetry island
        int minX = std::numeric_limits<int>::max();
        int minY = std::numeric_limits<int>::max();
        int maxX = 0;
        int maxY = 0;
        
        for (const auto& modulePair : pair.second->getModules()) {
            const auto& module = modulePair.second;
            
            minX = std::min(minX, module->getX());
            minY = std::min(minY, module->getY());
            maxX = std::max(maxX, module->getX() + module->getWidth());
            maxY = std::max(maxY, module->getY() + module->getHeight());
        }
        
        // Store group and its dimensions
        int width = maxX - minX;
        int height = maxY - minY;
        groupSizes.push_back({pair.second->getSymmetryGroup(), {width, height}});
        
        // Shift all modules to have their lower-left corner at (0, 0)
        for (auto& modulePair : pair.second->getModules()) {
            auto module = modulePair.second;
            module->setPosition(module->getX() - minX, module->getY() - minY);
        }
        
        // Update symmetry axis position
        if (pair.second->getSymmetryGroup()->getType() == SymmetryType::VERTICAL) {
            double oldAxis = pair.second->getSymmetryAxisPosition();
            pair.second->getSymmetryGroup()->setAxisPosition(oldAxis - minX);
        } else { // HORIZONTAL
            double oldAxis = pair.second->getSymmetryAxisPosition();
            pair.second->getSymmetryGroup()->setAxisPosition(oldAxis - minY);
        }
    }
    
    // Sort symmetry groups by area (largest first)
    std::sort(groupSizes.begin(), groupSizes.end(),
             [](const auto& a, const auto& b) {
                 return a.second.first * a.second.second > b.second.first * b.second.second;
             });
    
    // Grid-based placement of symmetry islands
    // Determine grid size based on number of symmetry groups
    int numGroups = groupSizes.size();
    int gridSize = std::ceil(std::sqrt(numGroups));
    
    // Place symmetry islands in a grid, trying to minimize total area
    int gridWidth = 0;
    int gridHeight = 0;
    
    // Track available positions for regular modules
    std::vector<std::pair<int, int>> availablePositions;
    
    // Special case handling for single symmetry group
    if (numGroups == 1) {
        auto& groupSize = groupSizes[0];
        auto& symGroup = groupSize.first;
        int width = groupSize.second.first;
        int height = groupSize.second.second;
        
        // Shift all modules in this group to (0, 0)
        auto& tree = symmetryTrees[symGroup->getName()];
        
        // Update grid dimensions
        gridWidth = width;
        gridHeight = height;
        
        // Add position to the right of this group for regular modules
        availablePositions.push_back({width, 0});
    } 
    else {
        // For multiple groups, try to optimize placement
        std::vector<std::vector<bool>> gridOccupied(gridSize, std::vector<bool>(gridSize, false));
        std::vector<std::vector<std::pair<int, int>>> gridBounds(gridSize, std::vector<std::pair<int, int>>(gridSize, {0, 0}));
        
        for (int i = 0; i < numGroups; ++i) {
            auto& groupSize = groupSizes[i];
            auto& symGroup = groupSize.first;
            int width = groupSize.second.first;
            int height = groupSize.second.second;
            
            // Find best position in grid
            int bestRow = 0;
            int bestCol = 0;
            int bestWastedSpace = std::numeric_limits<int>::max();
            
            for (int row = 0; row < gridSize; ++row) {
                for (int col = 0; col < gridSize; ++col) {
                    if (!gridOccupied[row][col]) {
                        // Calculate wasted space if we place the group here
                        int rowHeight = 0;
                        int colWidth = 0;
                        
                        // Calculate max height in this row and max width in this column
                        for (int r = 0; r < gridSize; ++r) {
                            if (gridOccupied[r][col]) {
                                colWidth = std::max(colWidth, gridBounds[r][col].first);
                            }
                        }
                        
                        for (int c = 0; c < gridSize; ++c) {
                            if (gridOccupied[row][c]) {
                                rowHeight = std::max(rowHeight, gridBounds[row][c].second);
                            }
                        }
                        
                        // Calculate wasted space
                        int wastedSpace = 0;
                        if (colWidth > 0 && width > colWidth) {
                            wastedSpace += (width - colWidth) * rowHeight;
                        }
                        if (rowHeight > 0 && height > rowHeight) {
                            wastedSpace += (height - rowHeight) * colWidth;
                        }
                        
                        // Update best position
                        if (wastedSpace < bestWastedSpace) {
                            bestWastedSpace = wastedSpace;
                            bestRow = row;
                            bestCol = col;
                        }
                    }
                }
            }
            
            // Place group at best position
            gridOccupied[bestRow][bestCol] = true;
            gridBounds[bestRow][bestCol] = {width, height};
            
            // Calculate position for this group
            int posX = 0;
            int posY = 0;
            
            for (int c = 0; c < bestCol; ++c) {
                int maxWidth = 0;
                for (int r = 0; r < gridSize; ++r) {
                    if (gridOccupied[r][c]) {
                        maxWidth = std::max(maxWidth, gridBounds[r][c].first);
                    }
                }
                posX += maxWidth;
            }
            
            for (int r = 0; r < bestRow; ++r) {
                int maxHeight = 0;
                for (int c = 0; c < gridSize; ++c) {
                    if (gridOccupied[r][c]) {
                        maxHeight = std::max(maxHeight, gridBounds[r][c].second);
                    }
                }
                posY += maxHeight;
            }
            
            // Shift all modules in this group to (posX, posY)
            auto& tree = symmetryTrees[symGroup->getName()];
            for (auto& modulePair : tree->getModules()) {
                auto module = modulePair.second;
                module->setPosition(module->getX() + posX, module->getY() + posY);
            }
            
            // Update symmetry axis position
            if (tree->getSymmetryGroup()->getType() == SymmetryType::VERTICAL) {
                double oldAxis = tree->getSymmetryAxisPosition();
                tree->getSymmetryGroup()->setAxisPosition(oldAxis + posX);
            } else { // HORIZONTAL
                double oldAxis = tree->getSymmetryAxisPosition();
                tree->getSymmetryGroup()->setAxisPosition(oldAxis + posY);
            }
            
            // Update grid dimensions
            gridWidth = std::max(gridWidth, posX + width);
            gridHeight = std::max(gridHeight, posY + height);
            
            // Add position to available positions for regular modules
            availablePositions.push_back({posX + width, posY});
        }
        
        // Add position at the bottom-right corner of the grid
        availablePositions.push_back({gridWidth, gridHeight});
    }
    
    // Then, pack regular modules at available positions
    if (regularTree && !regularModules.empty()) {
        if (!packRegularModules(availablePositions)) {
            std::cerr << "Failed to pack regular modules" << std::endl;
            return false;
        }
    }
    
    // Validate symmetry constraints
    if (!validateSymmetryConstraints()) {
        std::cerr << "Symmetry constraints violated" << std::endl;
        return false;
    }
    
    // Check for overlaps
    if (hasOverlaps()) {
        std::cerr << "Placement has overlapping modules" << std::endl;
        return false;
    }
    
    // Calculate total area
    calculateTotalArea();
    
    return true;
}

bool PlacementSolver::packRegularModules(const std::vector<std::pair<int, int>>& availablePositions) {
    if (!regularTree || availablePositions.empty()) {
        return true; // No regular modules to pack or no available positions
    }
    
    // Try each available position and choose the best one
    int bestArea = std::numeric_limits<int>::max();
    std::map<std::string, std::pair<int, int>> bestPositions;
    
    for (const auto& startPos : availablePositions) {
        int startX = startPos.first;
        int startY = startPos.second;
        
        // Use a simple packing algorithm for B*-tree
        std::queue<std::shared_ptr<BStarTreeNode>> nodeQueue;
        nodeQueue.push(regularTree);
        
        // Store temporary positions
        std::map<std::string, std::pair<int, int>> tempPositions;
        
        while (!nodeQueue.empty()) {
            auto currentNode = nodeQueue.front();
            nodeQueue.pop();
            
            const std::string& moduleName = currentNode->getModuleName();
            auto moduleIt = regularModules.find(moduleName);
            if (moduleIt == regularModules.end()) {
                continue;
            }
            
            auto module = moduleIt->second;
            
            int x = startX, y = startY;
            
            // Calculate position based on B*-tree rules
            if (currentNode->getParent()) {
                auto parentName = currentNode->getParent()->getModuleName();
                auto parentIt = tempPositions.find(parentName);
                if (parentIt != tempPositions.end()) {
                    auto& parentPos = parentIt->second;
                    auto parentModule = regularModules[parentName];
                    
                    if (currentNode->isLeftChild()) {
                        // Left child: place to the right of parent
                        x = parentPos.first + parentModule->getWidth();
                        y = parentPos.second;
                    } else {
                        // Right child: place above parent
                        x = parentPos.first;
                        y = parentPos.second + parentModule->getHeight();
                    }
                }
            }
            
            // Store temporary position
            tempPositions[moduleName] = {x, y};
            
            // Add children to the queue
            if (currentNode->getLeftChild()) {
                nodeQueue.push(currentNode->getLeftChild());
            }
            if (currentNode->getRightChild()) {
                nodeQueue.push(currentNode->getRightChild());
            }
        }
        
        // Calculate the area for this arrangement
        int minX = startX;
        int minY = startY;
        int maxX = startX;
        int maxY = startY;
        
        for (const auto& positionPair : tempPositions) {
            const auto& moduleName = positionPair.first;
            const auto& position = positionPair.second;
            auto module = regularModules[moduleName];
            
            maxX = std::max(maxX, position.first + module->getWidth());
            maxY = std::max(maxY, position.second + module->getHeight());
        }
        
        // Include symmetry islands in area calculation
        for (const auto& pair : symmetryTrees) {
            for (const auto& modulePair : pair.second->getModules()) {
                const auto& module = modulePair.second;
                
                minX = std::min(minX, module->getX());
                minY = std::min(minY, module->getY());
                maxX = std::max(maxX, module->getX() + module->getWidth());
                maxY = std::max(maxY, module->getY() + module->getHeight());
            }
        }
        
        int area = (maxX - minX) * (maxY - minY);
        
        // Update best area and positions
        if (area < bestArea) {
            bestArea = area;
            bestPositions = tempPositions;
        }
    }
    
    // Apply the best positions
    for (const auto& positionPair : bestPositions) {
        const auto& moduleName = positionPair.first;
        const auto& position = positionPair.second;
        auto module = regularModules[moduleName];
        
        module->setPosition(position.first, position.second);
    }
    
    return true;
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
    // Create initial solution
    createInitialSolution();
    
    if (!regularTree && symmetryTrees.empty()) {
        std::cerr << "Error: No modules to place." << std::endl;
        return false;
    }
    
    // Calculate initial area
    calculateTotalArea();
    std::cout << "Initial area: " << totalArea << std::endl;
    
    // Setup enhanced SA
    std::cout << "Starting simulated annealing..." << std::endl;
    std::cout << "Initial temperature: " << initialTemperature << std::endl;
    std::cout << "Final temperature: " << finalTemperature << std::endl;
    std::cout << "Cooling rate: " << coolingRate << std::endl;
    std::cout << "Iterations per temperature: " << iterationsPerTemperature << std::endl;
    std::cout << "No improvement limit: " << noImprovementLimit << std::endl;
    
    // Save initial solution as best solution
    std::map<std::string, std::pair<int, int>> bestPositions;
    std::map<std::string, bool> bestRotations;
    int bestArea = totalArea;
    
    // Save positions and rotations of all modules
    for (const auto& pair : allModules) {
        const auto& module = pair.second;
        bestPositions[pair.first] = {module->getX(), module->getY()};
        bestRotations[pair.first] = module->getRotated();
    }
    
    // Initialize random number generator
    std::mt19937 rng(randomSeed);
    std::uniform_real_distribution<double> uniformDist(0.0, 1.0);
    
    // SA parameters
    double temperature = initialTemperature;
    int noImprovementCount = 0;
    int totalIterations = 0;
    int acceptedMoves = 0;
    int rejectedMoves = 0;
    
    // Start timing
    auto startTime = std::chrono::steady_clock::now();
    
    // Main SA loop
    while (temperature > finalTemperature && noImprovementCount < noImprovementLimit) {
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
            
            // Store the current state of each module
            for (const auto& pair : allModules) {
                const auto& module = pair.second;
                currentPositions[pair.first] = {module->getX(), module->getY()};
                currentRotations[pair.first] = module->getRotated();
            }
            
            // Store current symmetry axis positions
            std::map<std::string, double> currentAxes;
            for (const auto& pair : symmetryTrees) {
                currentAxes[pair.first] = pair.second->getSymmetryAxisPosition();
            }
            
            // Perform a random perturbation
            bool perturbSuccess = false;
            std::string perturbedGroup;
            std::string perturbedModule;
            
            // Choose perturbation type
            double randVal = uniformDist(rng);
            
            if (randVal < probRotate) {
                // Rotate a random module
                if (!regularModules.empty() && uniformDist(rng) < 0.5) {
                    // Rotate a regular module
                    auto it = regularModules.begin();
                    std::advance(it, uniformDist(rng) * regularModules.size());
                    perturbedModule = it->first;
                    it->second->rotate();
                    perturbSuccess = true;
                } else if (!symmetryTrees.empty()) {
                    // Rotate a module in a symmetry group
                    auto it = symmetryTrees.begin();
                    std::advance(it, uniformDist(rng) * symmetryTrees.size());
                    perturbedGroup = it->first;
                    
                    // Get a random module from the symmetry group
                    auto& modules = it->second->getModules();
                    if (!modules.empty()) {
                        auto moduleIt = modules.begin();
                        std::advance(moduleIt, uniformDist(rng) * modules.size());
                        perturbedModule = moduleIt->first;
                        
                        // Rotate the module
                        perturbSuccess = it->second->rotateModule(perturbedModule);
                    }
                }
            } else if (randVal < probRotate + probMove && regularTree) {
                // Move operation for regular modules (simplified - just reorder the tree)
                if (regularTree && regularModules.size() > 1) {
                    // Find two random nodes to swap positions
                    std::vector<std::string> regularModuleNames;
                    for (const auto& pair : regularModules) {
                        regularModuleNames.push_back(pair.first);
                    }
                    
                    if (regularModuleNames.size() >= 2) {
                        // Pick two random modules
                        int idx1 = uniformDist(rng) * regularModuleNames.size();
                        int idx2;
                        do {
                            idx2 = uniformDist(rng) * regularModuleNames.size();
                        } while (idx2 == idx1);
                        
                        // Swap positions
                        std::string name1 = regularModuleNames[idx1];
                        std::string name2 = regularModuleNames[idx2];
                        
                        auto module1 = regularModules[name1];
                        auto module2 = regularModules[name2];
                        
                        // Exchange positions
                        int tmpX = module1->getX();
                        int tmpY = module1->getY();
                        module1->setPosition(module2->getX(), module2->getY());
                        module2->setPosition(tmpX, tmpY);
                        
                        perturbSuccess = true;
                    }
                }
            } else if (randVal < probRotate + probMove + probSwap) {
                // Swap modules within a symmetry group or between regular modules
                if (uniformDist(rng) < 0.5 && regularModules.size() >= 2) {
                    // Swap two regular modules (simplified)
                    std::vector<std::string> regularModuleNames;
                    for (const auto& pair : regularModules) {
                        regularModuleNames.push_back(pair.first);
                    }
                    
                    if (regularModuleNames.size() >= 2) {
                        // Pick two random modules
                        int idx1 = uniformDist(rng) * regularModuleNames.size();
                        int idx2;
                        do {
                            idx2 = uniformDist(rng) * regularModuleNames.size();
                        } while (idx2 == idx1);
                        
                        // Swap their positions
                        std::string name1 = regularModuleNames[idx1];
                        std::string name2 = regularModuleNames[idx2];
                        
                        auto module1 = regularModules[name1];
                        auto module2 = regularModules[name2];
                        
                        // Swap positions if dimensions are compatible
                        if (module1->getWidth() <= module2->getWidth() &&
                            module1->getHeight() <= module2->getHeight()) {
                            int tmpX = module1->getX();
                            int tmpY = module1->getY();
                            module1->setPosition(module2->getX(), module2->getY());
                            module2->setPosition(tmpX, tmpY);
                            perturbSuccess = true;
                        }
                    }
                } else if (!symmetryTrees.empty()) {
                    // Swap modules within a symmetry group (more complex)
                    // Not implemented in this simplified version
                }
            } else if (randVal < probRotate + probMove + probSwap + probChangeRep) {
                // Change representative in a symmetry group
                if (!symmetryTrees.empty()) {
                    auto it = symmetryTrees.begin();
                    std::advance(it, uniformDist(rng) * symmetryTrees.size());
                    perturbedGroup = it->first;
                    
                    // Get a random symmetry pair
                    auto group = it->second->getSymmetryGroup();
                    const auto& pairs = group->getSymmetryPairs();
                    if (!pairs.empty()) {
                        int pairIndex = uniformDist(rng) * pairs.size();
                        const auto& pair = pairs[pairIndex];
                        
                        // Choose one of the modules in the pair
                        perturbedModule = (uniformDist(rng) < 0.5) ? pair.first : pair.second;
                        
                        // Change representative
                        perturbSuccess = it->second->changeRepresentative(perturbedModule);
                    }
                }
            } else {
                // Convert symmetry type
                if (!symmetryTrees.empty()) {
                    auto it = symmetryTrees.begin();
                    std::advance(it, uniformDist(rng) * symmetryTrees.size());
                    perturbedGroup = it->first;
                    
                    // Convert symmetry type
                    perturbSuccess = it->second->convertSymmetryType();
                }
            }
            
            // Skip if perturbation failed
            if (!perturbSuccess) {
                continue;
            }
            
            // Pack solution and check if it's valid
            bool packSuccess = packSolution();
            
            if (packSuccess && !hasOverlaps() && validateSymmetryConstraints()) {
                // Calculate new area
                int newArea = calculateTotalArea();
                int deltaCost = newArea - totalArea;
                
                // Decide whether to accept
                bool accept = false;
                if (deltaCost <= 0) {
                    // Always accept improvements
                    accept = true;
                    if (newArea < bestArea) {
                        improved = true;
                        bestArea = newArea;
                        
                        // Update best solution
                        for (const auto& pair : allModules) {
                            const auto& module = pair.second;
                            bestPositions[pair.first] = {module->getX(), module->getY()};
                            bestRotations[pair.first] = module->getRotated();
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
                    totalArea = newArea;
                    acceptedMoves++;
                    acceptedInPassCount++;
                } else {
                    // Move rejected, restore previous state
                    rejectedMoves++;
                    
                    // Restore all module positions and rotations
                    for (auto& pair : allModules) {
                        auto module = pair.second;
                        const auto& oldPos = currentPositions[pair.first];
                        bool oldRot = currentRotations[pair.first];
                        
                        module->setPosition(oldPos.first, oldPos.second);
                        module->setRotation(oldRot);
                    }
                    
                    // Restore symmetry axis positions
                    for (auto& pair : symmetryTrees) {
                        auto& tree = pair.second;
                        auto group = tree->getSymmetryGroup();
                        double oldAxis = currentAxes[pair.first];
                        group->setAxisPosition(oldAxis);
                    }
                    
                    // Restore area
                    calculateTotalArea();
                }
            } else {
                // Invalid placement, restore previous state
                rejectedMoves++;
                
                // Restore all module positions and rotations
                for (auto& pair : allModules) {
                    auto module = pair.second;
                    const auto& oldPos = currentPositions[pair.first];
                    bool oldRot = currentRotations[pair.first];
                    
                    module->setPosition(oldPos.first, oldPos.second);
                    module->setRotation(oldRot);
                }
                
                // Restore symmetry axis positions
                for (auto& pair : symmetryTrees) {
                    auto& tree = pair.second;
                    auto group = tree->getSymmetryGroup();
                    double oldAxis = currentAxes[pair.first];
                    group->setAxisPosition(oldAxis);
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
            // Cool faster if acceptance rate is high
            temperature *= coolingRate * 0.9;
        } else if (acceptanceRatio < 0.1) {
            // Cool slower if acceptance rate is low
            temperature *= coolingRate * 1.1;
            if (temperature > initialTemperature) {
                temperature = initialTemperature;
            }
        } else {
            // Normal cooling
            temperature *= coolingRate;
        }
        
        std::cout << "Temperature: " << temperature 
                  << ", Best area: " << bestArea 
                  << ", Current area: " << totalArea 
                  << ", No improvement: " << noImprovementCount 
                  << ", Acceptance ratio: " << acceptanceRatio 
                  << std::endl;
    }
    
    // Restore best solution
    for (auto& pair : allModules) {
        auto module = pair.second;
        const auto& bestPos = bestPositions[pair.first];
        bool bestRot = bestRotations[pair.first];
        
        module->setPosition(bestPos.first, bestPos.second);
        module->setRotation(bestRot);
    }
    
    // Make sure symmetry axis positions are correct
    for (auto& pair : symmetryTrees) {
        auto& tree = pair.second;
        auto group = tree->getSymmetryGroup();
        
        if (group->getType() == SymmetryType::VERTICAL) {
            // For each symmetry pair, find the midpoint of their X coordinates
            double sumAxis = 0.0;
            int count = 0;
            
            for (const auto& symPair : group->getSymmetryPairs()) {
                auto it1 = allModules.find(symPair.first);
                auto it2 = allModules.find(symPair.second);
                
                if (it1 != allModules.end() && it2 != allModules.end()) {
                    auto& mod1 = it1->second;
                    auto& mod2 = it2->second;
                    
                    // Calculate centers
                    double center1X = mod1->getX() + mod1->getWidth() / 2.0;
                    double center2X = mod2->getX() + mod2->getWidth() / 2.0;
                    
                    // X1 + X2 = 2 * axis
                    double axis = (center1X + center2X) / 2.0;
                    sumAxis += axis;
                    count++;
                }
            }
            
            if (count > 0) {
                double axisPos = sumAxis / count;
                group->setAxisPosition(axisPos);
            }
        } else { // HORIZONTAL
            // For each symmetry pair, find the midpoint of their Y coordinates
            double sumAxis = 0.0;
            int count = 0;
            
            for (const auto& symPair : group->getSymmetryPairs()) {
                auto it1 = allModules.find(symPair.first);
                auto it2 = allModules.find(symPair.second);
                
                if (it1 != allModules.end() && it2 != allModules.end()) {
                    auto& mod1 = it1->second;
                    auto& mod2 = it2->second;
                    
                    // Calculate centers
                    double center1Y = mod1->getY() + mod1->getHeight() / 2.0;
                    double center2Y = mod2->getY() + mod2->getHeight() / 2.0;
                    
                    // Y1 + Y2 = 2 * axis
                    double axis = (center1Y + center2Y) / 2.0;
                    sumAxis += axis;
                    count++;
                }
            }
            
            if (count > 0) {
                double axisPos = sumAxis / count;
                group->setAxisPosition(axisPos);
            }
        }
    }
    
    // Final pack and validation
    bool finalPackSuccess = packSolution();
    if (!finalPackSuccess) {
        std::cerr << "Final packing failed, trying to recover..." << std::endl;
        
        // Reapply best positions directly
        for (auto& pair : allModules) {
            auto module = pair.second;
            const auto& bestPos = bestPositions[pair.first];
            bool bestRot = bestRotations[pair.first];
            
            module->setPosition(bestPos.first, bestPos.second);
            module->setRotation(bestRot);
        }
        
        // Recalculate area
        calculateTotalArea();
    }
    
    // Final validation
    if (hasOverlaps()) {
        std::cerr << "Warning: Final solution has overlaps" << std::endl;
    }
    
    if (!validateSymmetryConstraints()) {
        std::cerr << "Warning: Final solution violates symmetry constraints" << std::endl;
    }
    
    std::cout << "Simulated annealing completed." << std::endl;
    std::cout << "Final area: " << totalArea << std::endl;
    
    return true;
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