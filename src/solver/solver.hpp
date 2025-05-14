/**
 * solver.hpp
 * 
 * This file defines the PlacementSolver class which orchestrates the overall
 * placement algorithm using the new modular architecture.
 */

#pragma once

#include <memory>
#include <vector>
#include <map>
#include <string>
#include <random>
#include <variant>
#include <chrono>

#include "../data_struct/Module.hpp"
#include "../data_struct/SymmetryConstraint.hpp"
#include "../data_struct/BStarTreeNode.hpp"
#include "../data_struct/RestructuredBStarTree.hpp"
#include "../data_struct/ASFBStarTree.hpp"
#include "../data_struct/SymmetryIslandBlock.hpp"
#include "../utils/PlacementModel.hpp"
#include "../utils/SA.hpp"

class PlacementSolver {
private:
    // Placement model containing all solution data
    PlacementModel model;
    
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
    
    // Random seed and generator
    unsigned int randomSeed;
    std::mt19937 rng;
    std::uniform_real_distribution<double> uniformDist;
    
    // Time limit in seconds
    int timeLimit;
    
    /**
     * Creates an initial placement solution
     */
    void createInitialSolution();
    
    /**
     * Creates an initial ASF-B*-tree for a symmetry group
     * 
     * @param asfTree The ASF-B*-tree to initialize
     */
    void createInitialASFBTree(std::shared_ptr<ASFBStarTree> asfTree);
    
    /**
     * Creates a global B*-tree for overall placement
     */
    void createGlobalBTree();

    std::shared_ptr<BStarTreeNode> convertRestructuredTreeToGlobalTree(const RestructuredBStarTree &restructuredTree);

    void optimizeGlobalTreeStructure();

    void collectGlobalNodesPreOrder(const std::shared_ptr<BStarTreeNode> &node, std::vector<std::shared_ptr<BStarTreeNode>> &nodes);

    void rebuildGlobalTree();

    void updateTreeStructureAfterMovement(const std::string &moduleName);

    bool validateAndUpdateTreeStructure(const std::string &fromNodeName, const std::string &toNodeName, bool asLeftChild);

    /**
     * Initializes module grouping
     * Identifies which modules belong to which symmetry groups
     */
    void initializeModuleGrouping();

    std::shared_ptr<BStarTreeNode> findNodeInGlobalTree(const std::shared_ptr<BStarTreeNode> &root, const std::string &nodeName);

    /**
     * Performs a random perturbation on the current solution
     *
     * @return True if perturbation succeeded
     */
    bool performRandomPerturbation();
    
    /**
     * Cost function for the placement
     * 
     * @param model The placement model
     * @return Total cost (mainly area)
     */
    int calculateCost(const PlacementModel& model);
    
    /**
     * Saves the current solution
     * 
     * @param source Source solution
     * @param destination Destination to save to
     */
    void saveSolution(const PlacementModel& source, PlacementModel& destination);
    
    /**
     * Restores a saved solution
     * 
     * @param destination Solution to restore to
     * @param source Source solution
     */
    void restoreSolution(PlacementModel& destination, const PlacementModel& source);
    
    /**
     * Validates the current solution
     * 
     * @param model The placement model to validate
     * @return True if solution is valid
     */
    bool validateSolution(const PlacementModel& model);

    /**
     * Validates and repairs the initial solution until it's valid or max attempts reached
     * @return True if a valid solution was found
     */
    bool validateAndRepairInitialSolution();

    /**
     * Attempts to repair an invalid initial solution by resolving overlaps
     * @return True if repair was successful
     */
    bool repairInitialSolution();

    /**
     * Attempts to resolve specific overlaps between modules
     * @param module1Name First module name
     * @param module2Name Second module name
     * @return True if overlap was resolved
     */
    bool resolveModuleOverlap(const std::string& module1Name, const std::string& module2Name);
    
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
};