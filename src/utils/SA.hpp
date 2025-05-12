// SA.hpp
#pragma once

#include <memory>
#include <random>
#include <cmath>
#include <functional>
#include <vector>
#include <string>
#include <chrono>
#include <map>
#include <unordered_map>
#include "../data_struct/Module.hpp"
#include "../data_struct/SymmetryConstraint.hpp"
#include "../data_struct/ASFBStarTree.hpp"
#include "../data_struct/BStarTreeNode.hpp"

// Enum for different perturbation types
enum class PerturbationType {
    NONE,
    ROTATE,
    MOVE,
    SWAP,
    CHANGE_REP,
    CONVERT_SYM
};

// Structure to store module state for efficient reversion
struct ModuleState {
    int x;
    int y;
    bool rotated;
    
    // Add default constructor to fix compilation error
    ModuleState() : x(0), y(0), rotated(false) {}
    ModuleState(int x, int y, bool rotated) : x(x), y(y), rotated(rotated) {}
};

// Structure to store symmetry group state
struct SymmetryGroupState {
    double axisPosition;
    SymmetryType type;
    
    // Add default constructor to fix compilation error
    SymmetryGroupState() : axisPosition(0.0), type(SymmetryType::VERTICAL) {}
    SymmetryGroupState(double axisPosition, SymmetryType type) 
        : axisPosition(axisPosition), type(type) {}
};

class SimulatedAnnealing {
private:
    // Reference to all modules
    std::map<std::string, std::shared_ptr<Module>> allModules;
    
    // Regular modules (not part of symmetry groups)
    std::map<std::string, std::shared_ptr<Module>> regularModules;
    
    // Symmetry trees for symmetry groups
    std::map<std::string, std::shared_ptr<ASFBStarTree>> symmetryTrees;
    
    // B*-tree for regular modules
    std::shared_ptr<BStarTreeNode> regularTree;
    
    // Mapping from module name to its parent symmetry group
    std::map<std::string, std::shared_ptr<SymmetryGroup>> moduleToGroup;
    
    // Simulated annealing parameters
    double initialTemperature;
    double finalTemperature;
    double coolingRate;
    int iterationsPerTemperature;
    int noImprovementLimit;
    int timeLimit; // in seconds
    
    // Random number generation - make both mutable to fix const method issue
    mutable std::mt19937 rng;
    mutable std::uniform_real_distribution<double> uniformDist;
    
    // Current and best solutions
    int currentArea;
    int bestArea;
    std::map<std::string, ModuleState> bestSolution;
    std::map<std::string, SymmetryGroupState> bestSymmetryState;
    
    // Last perturbation details for efficient reversion
    PerturbationType lastPerturbation;
    std::string perturbedModule1;
    std::string perturbedModule2;
    std::string perturbedSymGroup;
    std::map<std::string, ModuleState> preperturbationState;
    std::map<std::string, SymmetryGroupState> preperturbationSymState;
    
    // Statistics
    int totalIterations;
    int acceptedMoves;
    int rejectedMoves;
    int noImprovementCount;
    std::chrono::steady_clock::time_point startTime;
    
    // Perturbation probabilities
    double probRotate;
    double probMove;
    double probSwap;
    double probChangeRep;
    double probConvertSym;
    
    // Cost function weights
    double areaWeight;
    double wirelengthWeight;
    
    /**
     * Packs the solution
     * @return True if packing succeeded without overlaps
     */
    bool packSolution();
    
    /**
     * Packs regular modules
     * @param startX Starting X coordinate for regular modules
     * @param startY Starting Y coordinate for regular modules
     * @return True if packing succeeded
     */
    bool packRegularModules(int startX, int startY);
    
    /**
     * Calculates the area of the current solution
     * @return Total area
     */
    int calculateArea();
    
    /**
     * Checks if there are any overlaps between modules
     * @return True if overlaps exist
     */
    bool hasOverlaps() const;
    
    /**
     * Validates symmetry constraints
     * @return True if all symmetry constraints are valid
     */
    bool validateSymmetryConstraints() const;
    
    /**
     * Performs a random perturbation
     * @return True if perturbation succeeded
     */
    bool perturb();
    
    /**
     * Performs a rotation perturbation
     * @return True if perturbation succeeded
     */
    bool perturbRotate();
    
    /**
     * Performs a move perturbation
     * @return True if perturbation succeeded
     */
    bool perturbMove();
    
    /**
     * Performs a swap perturbation
     * @return True if perturbation succeeded
     */
    bool perturbSwap();
    
    /**
     * Performs a change representative perturbation
     * @return True if perturbation succeeded
     */
    bool perturbChangeRepresentative();
    
    /**
     * Performs a convert symmetry type perturbation
     * @return True if perturbation succeeded
     */
    bool perturbConvertSymmetryType();
    
    /**
     * Reverts the last perturbation
     */
    void revertPerturbation();
    
    /**
     * Decides whether to accept a move based on cost difference and temperature
     * @param costDiff Cost difference (new - old)
     * @param temp Current temperature
     * @return True if move should be accepted
     */
    bool acceptMove(int costDiff, double temp) const;
    
    /**
     * Saves the current solution as the best solution
     */
    void saveBestSolution();
    
    /**
     * Restores the best solution
     */
    void restoreBestSolution();
    
    /**
     * Saves the current state before perturbation
     */
    void saveCurrentState();
    
    /**
     * Checks if time limit has been reached
     * @return True if time limit reached
     */
    bool isTimeLimitReached() const;

public:
    /**
     * Constructor
     * @param allModules All modules in the design
     * @param regularModules Regular modules (not part of symmetry groups)
     * @param symmetryTrees Symmetry trees for symmetry groups
     * @param regularTree B*-tree for regular modules
     * @param moduleToGroup Mapping from module name to symmetry group
     * @param initialTemp Initial temperature
     * @param finalTemp Final temperature
     * @param coolRate Cooling rate
     * @param iterations Iterations per temperature
     * @param noImprovementLimit No improvement limit
     * @param timeLimit Time limit in seconds
     */
    SimulatedAnnealing(
        const std::map<std::string, std::shared_ptr<Module>>& allModules,
        const std::map<std::string, std::shared_ptr<Module>>& regularModules,
        const std::map<std::string, std::shared_ptr<ASFBStarTree>>& symmetryTrees,
        std::shared_ptr<BStarTreeNode> regularTree,
        const std::map<std::string, std::shared_ptr<SymmetryGroup>>& moduleToGroup,
        double initialTemp = 1000.0,
        double finalTemp = 0.1,
        double coolRate = 0.98,
        int iterations = 300,
        int noImprovementLimit = 3000,
        int timeLimit = 290
    );
    
    /**
     * Sets perturbation probabilities
     * @param rotate Probability of rotation
     * @param move Probability of move
     * @param swap Probability of swap
     * @param changeRep Probability of changing representative
     * @param convertSym Probability of converting symmetry type
     */
    void setPerturbationProbabilities(double rotate, double move, double swap,
                                     double changeRep, double convertSym);
    
    /**
     * Sets cost function weights
     * @param area Weight for area
     * @param wirelength Weight for wirelength
     */
    void setCostWeights(double area, double wirelength);
    
    /**
     * Sets random seed
     * @param seed Random seed
     */
    void setSeed(unsigned int seed);
    
    /**
     * Sets time limit
     * @param seconds Time limit in seconds
     */
    void setTimeLimit(int seconds);
    
    /**
     * Runs the simulated annealing algorithm
     * @return True if a valid solution was found
     */
    bool run();
    
    /**
     * Gets the best area found
     * @return Best area
     */
    int getBestArea() const;
    
    /**
     * Gets statistics about the annealing process
     * @return Map of statistic name to value
     */
    std::map<std::string, int> getStatistics() const;
};