/**
 * SimulatedAnnealing.hpp
 * 
 * A generic simulated annealing algorithm implementation that can be used
 * for various optimization problems including placement.
 */

#pragma once

#include <memory>
#include <random>
#include <functional>
#include <chrono>
#include <map>
#include <string>
#include <vector>
#include <iostream>

/**
 * A generic simulated annealing algorithm that can be applied to various optimization problems.
 * Template parameters:
 * - SolutionType: Type representing the solution (e.g., placement configuration)
 * - CostType: Type for the cost function (usually int or double)
 */
template<typename SolutionType, typename CostType = double>
class SimulatedAnnealing {
public:
    // Define function types for cost calculation and perturbation
    using CostFunction = std::function<CostType(const SolutionType&)>;
    using PerturbFunction = std::function<bool(SolutionType&)>;
    using SaveFunction = std::function<void(const SolutionType&, SolutionType&)>;
    using RestoreFunction = std::function<void(SolutionType&, const SolutionType&)>;
    using ValidationFunction = std::function<bool(const SolutionType&)>;
    
    /**
     * Constructor
     * 
     * @param initialSolution Initial solution
     * @param costFunction Function to calculate the cost of a solution
     * @param perturbFunction Function to perturb a solution
     * @param saveSolution Function to save a solution
     * @param restoreSolution Function to restore a solution
     * @param validateSolution Function to validate a solution
     * @param initialTemp Initial temperature
     * @param finalTemp Final temperature
     * @param coolRate Cooling rate
     * @param iterations Iterations per temperature
     * @param noImprovementLimit Maximum number of iterations without improvement
     * @param timeLimit Time limit in seconds
     */
    SimulatedAnnealing(
        SolutionType& initialSolution,
        CostFunction costFunction,
        PerturbFunction perturbFunction,
        SaveFunction saveSolution,
        RestoreFunction restoreSolution,
        ValidationFunction validateSolution,
        double initialTemp = 1000.0,
        double finalTemp = 0.1,
        double coolRate = 0.98,
        int iterations = 300,
        int noImprovementLimit = 3000,
        int timeLimit = 290)
        : currentSolution(initialSolution),
          costFn(costFunction),
          perturbFn(perturbFunction),
          saveFn(saveSolution),
          restoreFn(restoreSolution),
          validateFn(validateSolution),
          initialTemperature(initialTemp),
          finalTemperature(finalTemp),
          coolingRate(coolRate),
          iterationsPerTemperature(iterations),
          noImprovementLimit(noImprovementLimit),
          timeLimit(timeLimit),
          uniformDist(0.0, 1.0),
          totalIterations(0),
          acceptedMoves(0),
          rejectedMoves(0),
          noImprovementCount(0)
    {
        // Initialize random number generator
        rng.seed(static_cast<unsigned int>(std::time(nullptr)));
        
        // Save initial solution as the best
        bestCost = costFn(currentSolution);
        saveFn(currentSolution, bestSolution);
    }
    
    /**
     * Sets random seed for reproducibility
     * 
     * @param seed Random seed
     */
    void setSeed(unsigned int seed) {
        rng.seed(seed);
    }
    
    /**
     * Sets time limit
     * 
     * @param seconds Time limit in seconds
     */
    void setTimeLimit(int seconds) {
        timeLimit = seconds;
    }
    
    /**
     * Main optimization function that runs the simulated annealing algorithm
     * 
     * @return Best solution found
     */
    SolutionType optimize() {
        double temperature = initialTemperature;
        noImprovementCount = 0;
        totalIterations = 0;
        acceptedMoves = 0;
        rejectedMoves = 0;
        startTime = std::chrono::steady_clock::now();
        
        // Calculate initial cost
        currentCost = costFn(currentSolution);
        
        std::cout << "Starting SA with initial cost: " << currentCost << std::endl;
        
        // Main annealing loop
        while (temperature > finalTemperature && 
               noImprovementCount < noImprovementLimit && 
               !isTimeLimitReached()) {
            
            int acceptedInPass = 0;
            int totalInPass = 0;
            
            // Iterations at current temperature
            for (int i = 0; i < iterationsPerTemperature && !isTimeLimitReached(); ++i) {
                // Save current solution before perturbation
                SolutionType tempSolution;
                saveFn(currentSolution, tempSolution);
                
                // Perform perturbation
                if (!perturbFn(currentSolution)) {
                    continue;
                }
                
                totalInPass++;
                
                // Validate solution
                bool isValid = validateFn(currentSolution);
                
                if (isValid) {
                    // Calculate new cost
                    CostType newCost = costFn(currentSolution);
                    CostType costDiff = newCost - currentCost;
                    
                    // Accept or reject
                    if (acceptMove(costDiff, temperature)) {
                        // Accept the perturbation
                        currentCost = newCost;
                        acceptedMoves++;
                        acceptedInPass++;
                        
                        // Update best solution if improved
                        if (newCost < bestCost) {
                            bestCost = newCost;
                            saveFn(currentSolution, bestSolution);
                            noImprovementCount = 0;
                            
                            std::cout << "New best cost found: " << bestCost << std::endl;
                        } else {
                            noImprovementCount++;
                        }
                    } else {
                        // Reject the perturbation
                        restoreFn(currentSolution, tempSolution);
                        rejectedMoves++;
                        noImprovementCount++;
                    }
                } else {
                    // Invalid solution, revert the perturbation
                    // At high temperatures, occasionally accept invalid solutions to explore more
                    if (temperature > finalTemperature * 10 && acceptInvalidMove(temperature)) {
                        // Accept the invalid solution to escape local minimum
                        currentCost = bestCost;
                        acceptedMoves++;
                        std::cout << "Accepted invalid solution at temperature " << temperature << std::endl;
                    } else {
                        // Reject invalid solution
                        restoreFn(currentSolution, tempSolution);
                        rejectedMoves++;
                    }
                }
                
                totalIterations++;
                
                // Periodically output stats
                if (totalIterations % 100 == 0) {
                    std::cout << "Iteration: " << totalIterations 
                              << ", Temperature: " << temperature 
                              << ", Current cost: " << currentCost 
                              << ", Best cost: " << bestCost 
                              << std::endl;
                }
            }
            
            // Calculate acceptance ratio
            double acceptanceRatio = (totalInPass > 0) ? 
                static_cast<double>(acceptedInPass) / totalInPass : 0.0;
            
            // Adaptive cooling schedule
            if (acceptanceRatio > 0.6) {
                // Too many acceptances, cool faster
                temperature *= coolingRate * 0.8;
            } else if (acceptanceRatio < 0.1) {
                // Too few acceptances, cool slower
                temperature *= coolingRate * 1.1;
                if (temperature > initialTemperature) {
                    temperature = initialTemperature;
                }
            } else {
                // Normal cooling
                temperature *= coolingRate;
            }
            
            // If stuck in a bad local minimum, consider reheating
            if (acceptanceRatio < 0.05 && noImprovementCount > noImprovementLimit / 2) {
                // Reheat to escape local minimum
                temperature = std::min(initialTemperature * 0.5, temperature * 5.0);
                std::cout << "Reheating to escape local minimum. New temperature: " << temperature << std::endl;
                
                // Consider reverting to best solution to reset path
                if (noImprovementCount > noImprovementLimit * 0.8) {
                    restoreFn(currentSolution, bestSolution);
                    currentCost = bestCost;
                    std::cout << "Restoring best solution to reset path. Cost: " << bestCost << std::endl;
                }
            }
            
            std::cout << "Temperature: " << temperature 
                      << ", Best cost: " << bestCost 
                      << ", Current cost: " << currentCost 
                      << ", No improvement: " << noImprovementCount 
                      << ", Acceptance ratio: " << acceptanceRatio 
                      << std::endl;
        }
        
        // Restore best solution
        restoreFn(currentSolution, bestSolution);
        
        std::cout << "Simulated annealing completed." << std::endl;
        std::cout << "Final cost: " << bestCost << std::endl;
        std::cout << "Total iterations: " << totalIterations << std::endl;
        std::cout << "Accepted moves: " << acceptedMoves << std::endl;
        std::cout << "Rejected moves: " << rejectedMoves << std::endl;
        
        return bestSolution;
    }
    
    /**
     * Gets the best cost found
     * 
     * @return Best cost
     */
    CostType getBestCost() const {
        return bestCost;
    }
    
    /**
     * Gets statistics about the annealing process
     * 
     * @return Map of statistic name to value
     */
    std::map<std::string, int> getStatistics() const {
        std::map<std::string, int> stats;
        stats["totalIterations"] = totalIterations;
        stats["acceptedMoves"] = acceptedMoves;
        stats["rejectedMoves"] = rejectedMoves;
        stats["noImprovementCount"] = noImprovementCount;
        
        auto currentTime = std::chrono::steady_clock::now();
        stats["elapsedTimeSeconds"] = static_cast<int>(std::chrono::duration_cast<std::chrono::seconds>(
            currentTime - startTime).count());
        
        return stats;
    }
    
private:
    // Current and best solutions
    SolutionType currentSolution;
    SolutionType bestSolution;
    
    // Function objects
    CostFunction costFn;
    PerturbFunction perturbFn;
    SaveFunction saveFn;
    RestoreFunction restoreFn;
    ValidationFunction validateFn;
    
    // Simulated annealing parameters
    double initialTemperature;
    double finalTemperature;
    double coolingRate;
    int iterationsPerTemperature;
    int noImprovementLimit;
    int timeLimit;
    
    // Current and best costs
    CostType currentCost;
    CostType bestCost;
    
    // Random number generation - make both mutable to use in const methods
    mutable std::mt19937 rng;
    mutable std::uniform_real_distribution<double> uniformDist;
    
    // Statistics
    int totalIterations;
    int acceptedMoves;
    int rejectedMoves;
    int noImprovementCount;
    std::chrono::steady_clock::time_point startTime;
    
    /**
     * Decides whether to accept a move based on cost difference and temperature
     * 
     * @param costDiff Cost difference (new - old)
     * @param temp Current temperature
     * @return True if move should be accepted
     */
    bool acceptMove(CostType costDiff, double temp) const {
        // Always accept improvements
        if (costDiff <= 0) {
            return true;
        }
        
        // Accept worsening moves with probability e^(-costDiff/temp)
        double probability = std::exp(-static_cast<double>(costDiff) / temp);
        return uniformDist(rng) < probability;
    }
    
    /**
     * Checks if time limit has been reached
     * 
     * @return True if time limit reached
     */
    bool isTimeLimitReached() const {
        auto currentTime = std::chrono::steady_clock::now();
        auto elapsedSeconds = std::chrono::duration_cast<std::chrono::seconds>(
            currentTime - startTime).count();
        return elapsedSeconds >= timeLimit;
    }

    /**
     * Decides whether to accept an invalid move based on current temperature
     * @param temp Current temperature
     * @return True if the invalid move should be accepted
     */
    bool acceptInvalidMove(double temp) const {
        // At high temperatures, occasionally accept invalid solutions to explore more
        double acceptanceProbability = temp / initialTemperature * 0.2;
        return uniformDist(rng) < acceptanceProbability;
    }
};