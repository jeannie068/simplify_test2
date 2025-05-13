// PlacementSolver.hpp
#pragma once

#include <memory>
#include <vector>
#include <map>
#include <string>
#include <unordered_set>
#include <unordered_map>
#include "../data_struct/Module.hpp"
#include "../data_struct/SymmetryConstraint.hpp"
#include "../data_struct/BStarTreeNode.hpp"
#include "../data_struct/ASFBStarTree.hpp"
#include "../utils/SA.hpp"
#include "../data_struct/SymmetryIslandBlock.hpp"

class PlacementSolver {
private:
    friend class SimulatedAnnealing;

    // Regular modules (not part of symmetry groups)
    std::map<std::string, std::shared_ptr<Module>> regularModules;
    
    // Symmetry groups and their modules
    std::vector<std::shared_ptr<SymmetryGroup>> symmetryGroups;
    std::map<std::string, std::shared_ptr<ASFBStarTree>> symmetryTrees;

    // Symmetry islands as blocks for global placement
    std::vector<std::shared_ptr<SymmetryIslandBlock>> symmetryIslands;
    
    // Global B*-tree for overall placement
    std::shared_ptr<BStarTreeNode> globalTree;

    // Map from node name to either Module or SymmetryIslandBlock
    std::map<std::string, std::variant<std::shared_ptr<Module>, std::shared_ptr<SymmetryIslandBlock>>> globalNodes;
    
    // All modules (reference to both regular and symmetry modules)
    std::map<std::string, std::shared_ptr<Module>> allModules;
    
    // Mapping from module name to its parent symmetry group
    std::map<std::string, std::shared_ptr<SymmetryGroup>> moduleToGroup;
    
    // B*-tree for regular modules
    std::shared_ptr<BStarTreeNode> regularTree;
    
    // Simulated annealing parameters
    double initialTemperature;
    double finalTemperature;
    double coolingRate;
    int iterationsPerTemperature;
    int noImprovementLimit;
    
    // Perturbation probabilities
    double probRotate;
    double probMove;
    double probSwap;
    double probChangeRep;
    double probConvertSym;
    
    // Cost function weights
    double areaWeight;
    double wirelengthWeight;
    
    // Random seed
    unsigned int randomSeed;
    
    // Statistics
    int totalArea;
    int solutionWidth;
    int solutionHeight;
    
    // Time limit in seconds
    int timeLimit;
    
    /**
     * Creates an initial placement solution
     */
    void createInitialSolution();
    
    /**
     * Packs all modules (regular and symmetry groups)
     * using grid-based packing strategy
     * @return True if packing succeeded
     */
    bool packSolution();
    
    /**
     * Packs regular modules using contour-based placement
     * 
     * @param horizontalContour The global horizontal contour
     * @param verticalContour The global vertical contour
     * @return True if packing succeeded
     */
    bool packRegularModules(std::shared_ptr<Contour> horizontalContour, 
                           std::shared_ptr<Contour> verticalContour);
    
    /**
     * Calculates total placement area
     * @return Total area
     */
    int calculateTotalArea();

    /**
     * Calculates the area of a symmetry group
     * 
     * @param group The symmetry group
     * @return Total area of all modules in the group
     */
    int calculateSymmetryGroupArea(const std::shared_ptr<SymmetryGroup>& group);
    
    /**
     * Checks if there are any overlaps between modules
     * @return True if there are overlaps
     */
    bool hasOverlaps() const;

    // Helper function to check if a position is valid (no overlaps)
    bool isPositionValid(int x, int y, int width, int height);
    
    /**
     * Validates symmetry constraints
     * @return True if all symmetry constraints are valid
     */
    bool validateSymmetryConstraints() const;
    
    /**
     * Initializes module grouping
     * Identifies which modules belong to which symmetry groups
     */
    void initializeModuleGrouping();

    /**
     * Creates a global B*-tree
     */
    void createGlobalBTree();
    
    /**
     * Packs the global B*-tree
     */
    bool packGlobalBTree();
    
public:
    /**
     * Constructor
     */
    PlacementSolver();
    
    /**
     * Destructor
     */
    ~PlacementSolver();
    
    /**
     * Loads modules and symmetry constraints
     * 
     * @param modules Map of module names to modules
     * @param symmetryGroups Vector of symmetry groups
     */
    void loadProblem(const std::map<std::string, std::shared_ptr<Module>>& modules,
                    const std::vector<std::shared_ptr<SymmetryGroup>>& symmetryGroups);
    
    /**
     * Sets simulated annealing parameters
     * 
     * @param initialTemp Initial temperature
     * @param finalTemp Final temperature
     * @param coolRate Cooling rate
     * @param iterations Number of iterations per temperature
     * @param noImprovementLimit Maximum number of iterations without improvement
     */
    void setAnnealingParameters(double initialTemp, double finalTemp, double coolRate, 
                               int iterations, int noImprovementLimit);
    
    /**
     * Sets perturbation probabilities
     * 
     * @param rotate Probability of rotation operation
     * @param move Probability of move operation
     * @param swap Probability of swap operation
     * @param changeRep Probability of change representative operation
     * @param convertSym Probability of convert symmetry type operation
     */
    void setPerturbationProbabilities(double rotate, double move, double swap, 
                                     double changeRep, double convertSym);
    
    /**
     * Sets cost function weights
     * 
     * @param area Weight for area term
     * @param wirelength Weight for wirelength term
     */
    void setCostWeights(double area, double wirelength);
    
    /**
     * Sets random seed for reproducibility
     * 
     * @param seed Random seed
     */
    void setRandomSeed(unsigned int seed);
    
    /**
     * Sets time limit for SA in seconds
     * 
     * @param seconds Time limit in seconds
     */
    void setTimeLimit(int seconds);
    
    /**
     * Solves the placement problem using simulated annealing
     * 
     * @return True if a valid solution was found, false otherwise
     */
    bool solve();
    
    /**
     * Gets the solution area
     * 
     * @return Total area of the placement
     */
    int getSolutionArea() const;
    
    /**
     * Gets the solution modules with their positions
     * 
     * @return Map of module names to modules
     */
    std::map<std::string, std::shared_ptr<Module>> getSolutionModules() const;
    
    /**
     * Gets placement solution statistics
     * 
     * @return Map of statistic name to value
     */
    std::map<std::string, int> getStatistics() const;

    std::shared_ptr<BStarTreeNode>& getGlobalTree() { return globalTree; }
    
    std::map<std::string, std::variant<std::shared_ptr<Module>, 
        std::shared_ptr<SymmetryIslandBlock>>>& getGlobalNodes() { 
        return globalNodes; 
    }
};