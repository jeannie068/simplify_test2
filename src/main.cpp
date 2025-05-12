#include <iostream>
#include <string>
#include <memory>
#include <ctime>
#include <cstdlib>

#include "parser/Parser.hpp"
#include "solver/solver.hpp"
#include "data_struct/Module.hpp"
#include "data_struct/SymmetryConstraint.hpp"

void printUsage(const char* programName) {
    std::cout << "Usage: " << programName << " <input_file> <output_file> [area_ratio]" << std::endl;
    std::cout << "  input_file: Path to the input .txt file" << std::endl;
    std::cout << "  output_file: Path to the output .out file" << std::endl;
    std::cout << "  area_ratio: Optional parameter for area vs. wirelength weight ratio (default 1.0)" << std::endl;
}

int main(int argc, char* argv[]) {
    // Check command line arguments
    if (argc < 3 || argc > 4) {
        printUsage(argv[0]);
        return 1;
    }

    std::string inputFile = argv[1];
    std::string outputFile = argv[2];
    double areaRatio = 1.0;  // Default area weight ratio
    
    // Parse optional area ratio parameter
    if (argc == 4) {
        try {
            areaRatio = std::stod(argv[3]);
            if (areaRatio < 0.0) {
                std::cerr << "Error: Area ratio must be non-negative" << std::endl;
                return 1;
            }
        } catch (const std::exception& e) {
            std::cerr << "Error parsing area ratio: " << e.what() << std::endl;
            return 1;
        }
    }
    
    // Record start time
    clock_t startTime = clock();
    
    // Parse input file
    std::map<std::string, std::shared_ptr<Module>> modules;
    std::vector<std::shared_ptr<SymmetryGroup>> symmetryGroups;
    
    std::cout << "Parsing input file: " << inputFile << std::endl;
    if (!Parser::parseInputFile(inputFile, modules, symmetryGroups)) {
        std::cerr << "Error parsing input file" << std::endl;
        return 1;
    }
    
    // Configure and run placement solver
    PlacementSolver solver;
    
    // Load problem data
    solver.loadProblem(modules, symmetryGroups);
    
    // Configure simulated annealing parameters (optimized for better convergence)
    solver.setAnnealingParameters(
        1000.0,     // Initial temperature
        0.1,        // Final temperature
        0.98,       // Cooling rate (slower cooling for better exploration)
        300,        // Iterations per temperature (increased for better exploration)
        3000        // No improvement limit (increased to allow more exploration)
    );
    
    // Configure perturbation probabilities (favor rotation which is more effective)
    solver.setPerturbationProbabilities(
        0.5,        // Rotate probability (increased from 0.3)
        0.2,        // Move probability (decreased from 0.3)
        0.2,        // Swap probability (decreased from 0.3)
        0.05,       // Change representative probability
        0.05        // Convert symmetry type probability
    );
    
    // Set cost function weights
    solver.setCostWeights(
        areaRatio,      // Area weight
        1.0 - areaRatio // Wirelength weight (complementary to area weight)
    );
    
    // Set random seed for reproducibility
    solver.setRandomSeed(static_cast<unsigned int>(time(nullptr)));
    
    // Set time limit to 290 seconds (4:50) to ensure completion within 5 minutes
    solver.setTimeLimit(290);
    
    // Solve the placement problem
    std::cout << "Solving placement problem..." << std::endl;
    if (!solver.solve()) {
        std::cerr << "Error solving placement problem" << std::endl;
        return 1;
    }
    
    // Get the final solution
    int solutionArea = solver.getSolutionArea();
    auto solutionModules = solver.getSolutionModules();
    
    // Write output file
    std::cout << "Writing output file: " << outputFile << std::endl;
    if (!Parser::writeOutputFile(outputFile, solutionModules, solutionArea)) {
        std::cerr << "Error writing output file" << std::endl;
        return 1;
    }
    
    // Display execution time
    clock_t endTime = clock();
    double executionTime = static_cast<double>(endTime - startTime) / CLOCKS_PER_SEC;
    std::cout << "Execution time: " << executionTime << " seconds" << std::endl;
    std::cout << "Final area: " << solutionArea << std::endl;
    
    return 0;
}