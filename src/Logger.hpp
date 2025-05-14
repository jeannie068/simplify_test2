#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <chrono>
#include <ctime>
#include <iomanip>

/**
 * @brief A simple logger class for debugging
 */
class Logger {
private:
    static std::ofstream logFile;
    static bool initialized;
    static int indent;
    
public:
    static void init(const std::string& filename = "debug_log.txt") {
        if (!initialized) {
            logFile.open(filename, std::ios::out | std::ios::trunc);
            initialized = true;
            indent = 0;
            log("Logger initialized");
        }
    }
    
    static void close() {
        if (initialized) {
            logFile.close();
            initialized = false;
        }
    }
    
    static void increaseIndent() {
        indent += 2;
    }
    
    static void decreaseIndent() {
        indent = std::max(0, indent - 2);
    }
    
    template<typename T>
    static void log(const T& message) {
        if (!initialized) {
            init();
        }
        
        // Get current time
        auto now = std::chrono::system_clock::now();
        auto time = std::chrono::system_clock::to_time_t(now);
        
        // Format timestamp
        std::stringstream timestamp;
        timestamp << std::put_time(std::localtime(&time), "%H:%M:%S");
        
        // Create indentation
        std::string indentation(indent, ' ');
        
        // Write to log file
        logFile << timestamp.str() << " | " << indentation << message << std::endl;
        
        // Also print to console for immediate feedback
        std::cout << "LOG: " << indentation << message << std::endl;
    }
    
    // Specific logger for tree structures
    template<typename NodeType>
    static void logTreeStructure(const std::string& title, NodeType* root) {
        log("Tree Structure: " + title);
        increaseIndent();
        
        if (!root) {
            log("Empty tree (null root)");
        } else {
            std::stringstream ss;
            visualizeTree(root, "", true, ss);
            log(ss.str());
        }
        
        decreaseIndent();
    }
    
private:
    // Helper for tree visualization
    template<typename NodeType>
    static void visualizeTree(NodeType* node, std::string prefix, bool isLast, std::stringstream& ss) {
        if (!node) return;
        
        ss << prefix;
        ss << (isLast ? "└── " : "├── ");
        
        // Print node info (customize based on your node type)
        ss << nodeToString(node) << "\n";
        
        // Prepare prefix for children
        prefix += isLast ? "    " : "│   ";
        
        // Process children (assuming left and right pointers)
        bool hasRight = (node->right != nullptr);
        visualizeTree(node->left, prefix, !hasRight, ss);
        visualizeTree(node->right, prefix, true, ss);
    }
    
    // Convert node to string representation (customize for your node type)
    template<typename NodeType>
    static std::string nodeToString(NodeType* node) {
        // Default implementation, override for specific node types
        return "Node*";
    }
};
