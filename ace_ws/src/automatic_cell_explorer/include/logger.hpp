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
    int n_candidates = -1;      // generated valid candidates
    double est_gain = -1;       // Estimated gain from selected nbv
    double utility_score = -1;  // Score of the best utility 
    double cluster_t = -1;      // Time to generate clusters
};

struct SmLog {
    std::chrono::time_point<std::chrono::system_clock> time;
    double nbv_calculation_t = -1;
    int iteration = -1;
    double progress = -1;
    int planner = -1;
    double traj_lenght = -1;
    double move_t = -1;
    double map_update_t = -1;
    double gain = -1; //acutal gain of the capture
    int n_nbv_memory = -1; // Candidates in memory
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

        start_time = std::chrono::system_clock::now();
        traj_lenght = 0.0;

        // Write header if file is empty
        if (file.tellp() == 0) {
            file << "Time,"
                 << "GenerateTime,EvalTime,EvalAvgTime,SelectTime,Attempts,Candidates,"
                 << "EstimatedGain,UtilityScore,NBVTime,Iteration,Progress,Planner,"
                 << "TrajectoryLength,MoveTime,MapUpdateTime,Gain,ClusterTime,NbvMemory\n";
        }
    }

    ~Logger() {
        if (file.is_open()) {
            file.close();
        }
    }

void logData(LogEntry& entry) {

        auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(
            entry.sm_log.time - start_time
        ).count();
        traj_lenght += entry.sm_log.traj_lenght;

        // Write the data to the CSV file
        file << elapsed_time << ","
             << (entry.ep_log.generate_t >= 0 ? std::to_string(entry.ep_log.generate_t) : "-1") << ","
             << (entry.ep_log.evaluate_t >= 0 ? std::to_string(entry.ep_log.evaluate_t) : "-1") << ","
             << (entry.ep_log.evaluate_av_t >= 0 ? std::to_string(entry.ep_log.evaluate_av_t) : "-1") << ","
             << (entry.ep_log.select_t >= 0 ? std::to_string(entry.ep_log.select_t) : "-1") << ","
             << (entry.ep_log.attempts >= 0 ? std::to_string(entry.ep_log.attempts) : "-1") << ","
             << (entry.ep_log.n_candidates >= 0 ? std::to_string(entry.ep_log.n_candidates) : "-1") << ","
             << (entry.ep_log.est_gain >= 0 ? std::to_string(entry.ep_log.est_gain) : "-1") << ","
             << (entry.ep_log.utility_score >= 0 ? std::to_string(entry.ep_log.utility_score) : "-1") << ","
             << (entry.sm_log.nbv_calculation_t >= 0 ? std::to_string(entry.sm_log.nbv_calculation_t) : "-1") << ","
             << (entry.sm_log.iteration >= 0 ? std::to_string(entry.sm_log.iteration) : "-1") << ","
             << (entry.sm_log.progress >= 0 ? std::to_string(entry.sm_log.progress) : "-1") << ","
             << (entry.sm_log.planner >= 0 ? std::to_string(entry.sm_log.planner) : "-1") << ","
             << traj_lenght << ","
             << (entry.sm_log.move_t >= 0 ? std::to_string(entry.sm_log.move_t) : "-1") << ","
             << (entry.sm_log.map_update_t >= 0 ? std::to_string(entry.sm_log.map_update_t) : "-1") << ","
             << (entry.sm_log.gain >= 0 ? std::to_string(entry.sm_log.gain) : "-1") << ","
             << (entry.ep_log.cluster_t >= 0 ? std::to_string(entry.ep_log.cluster_t) : "-1") << ","
             << (entry.sm_log.n_nbv_memory >= 0 ? std::to_string(entry.sm_log.n_nbv_memory) : "-1") << "\n";

        file.flush();
    }


private:
    std::ofstream file;
    std::chrono::time_point<std::chrono::system_clock> start_time;
    double traj_lenght;
};

#endif // LOGGER_HPP