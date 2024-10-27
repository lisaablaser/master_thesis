#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <chrono>
#include <fstream>
#include <iostream>
#include <iomanip>

struct EpLog {
    double generate_t = -1;     // time to generate all candidates
    double evaluate_t = -1;     // total time to evaluate candidates
    double evaluate_av_t = -1;  // average time to evaluate one candidate (RayCast). Milliseconds
    double select_t = -1;       // time to select nbv
    int attempts = -1;          // attempts to generate n candidates
    int n_candidates = -1;      // generated candidates
    double est_gain = -1;       // Estimated gain from selected nbv
    double utility_score = -1;  // Score of the best utility 
};

struct SmLog {
    double nbv_calculation_t = -1;
    int iteration = -1;
    double progress = -1;
    int planner = -1;
    double traj_lenght = -1;
    double move_t = -1;
    double map_update_t = -1;
};

struct LogEntry {
    EpLog ep_log;
    SmLog sm_log;
};



class Logger {
public:
    Logger(const std::string& filename) {
        file.open(filename, std::ios::out | std::ios::app);
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open log file");
        }

        // Write header if file is empty
        if (file.tellp() == 0) {
            file << "Time,"
                 << "GenerateTime,EvalTime,EvalAvgTime,SelectTime,Attempts,Candidates,"
                 << "EstimatedGain,UtilityScore,NBVTime,Iteration,Progress,Planner,"
                 << "TrajectoryLength,MoveTime,MapUpdateTime\n";
        }
    }

    ~Logger() {
        if (file.is_open()) {
            file.close();
        }
    }

void logData(LogEntry& entry) {

        auto timestamp = std::chrono::system_clock::now();
        auto seconds_since_epoch = std::chrono::duration_cast<std::chrono::seconds>(
            timestamp.time_since_epoch()
        ).count();

        // Write the data to the CSV file
        file << seconds_since_epoch << ","
             << (entry.ep_log.generate_t >= 0 ? std::to_string(entry.ep_log.generate_t) : "N/A") << ","
             << (entry.ep_log.evaluate_t >= 0 ? std::to_string(entry.ep_log.evaluate_t) : "N/A") << ","
             << (entry.ep_log.evaluate_av_t >= 0 ? std::to_string(entry.ep_log.evaluate_av_t) : "N/A") << ","
             << (entry.ep_log.select_t >= 0 ? std::to_string(entry.ep_log.select_t) : "N/A") << ","
             << (entry.ep_log.attempts >= 0 ? std::to_string(entry.ep_log.attempts) : "N/A") << ","
             << (entry.ep_log.n_candidates >= 0 ? std::to_string(entry.ep_log.n_candidates) : "N/A") << ","
             << (entry.ep_log.est_gain >= 0 ? std::to_string(entry.ep_log.est_gain) : "N/A") << ","
             << (entry.ep_log.utility_score >= 0 ? std::to_string(entry.ep_log.utility_score) : "N/A") << ","
             << (entry.sm_log.nbv_calculation_t >= 0 ? std::to_string(entry.sm_log.nbv_calculation_t) : "N/A") << ","
             << (entry.sm_log.iteration >= 0 ? std::to_string(entry.sm_log.iteration) : "N/A") << ","
             << (entry.sm_log.progress >= 0 ? std::to_string(entry.sm_log.progress) : "N/A") << ","
             << (entry.sm_log.planner >= 0 ? std::to_string(entry.sm_log.planner) : "N/A") << ","
             << (entry.sm_log.traj_lenght >= 0 ? std::to_string(entry.sm_log.traj_lenght) : "N/A") << ","
             << (entry.sm_log.move_t >= 0 ? std::to_string(entry.sm_log.move_t) : "N/A") << ","
             << (entry.sm_log.map_update_t >= 0 ? std::to_string(entry.sm_log.map_update_t) : "N/A") << "\n";

        file.flush();
    }


private:
    std::ofstream file;
};

#endif // LOGGER_HPP