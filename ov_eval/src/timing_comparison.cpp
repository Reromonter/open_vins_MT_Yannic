/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <Eigen/Eigen>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <fstream>
#include <iostream>
#include <string>

#include "utils/Loader.h"
#include "utils/Statistics.h"
#include "utils/colors.h"
#include "utils/print.h"

#ifdef HAVE_PYTHONLIBS

// import the c++ wrapper for matplot lib
// https://github.com/lava/matplotlib-cpp
// sudo apt-get install python-matplotlib python-numpy python2.7-dev
#include "plot/matplotlibcpp.h"

#endif

int main(int argc, char **argv) {

  // Verbosity setting
  ov_core::Printer::setPrintLevel("INFO");

  // Ensure we have a path
  if (argc < 2) {
    PRINT_ERROR(RED "ERROR: Please specify timing files or a folder to search\n" RESET);
    PRINT_ERROR(RED "ERROR: ./timing_comparison <file_times1.txt> ... <file_timesN.txt>\n" RESET);
    PRINT_ERROR(RED "ERROR: ./timing_comparison <search_folder>\n" RESET);
    PRINT_ERROR(RED "ERROR: rosrun ov_eval timing_comparison <files_or_folder>\n" RESET);
    std::exit(EXIT_FAILURE);
  }

  // Determine if we have a folder search or individual files
  std::vector<std::pair<std::string, std::string>> timing_files; // pair of <name, file_path>
  
  if (argc == 2 && boost::filesystem::is_directory(argv[1])) {
    // Folder search mode - search for traj_timing files recursively
    std::string search_folder(argv[1]);
    PRINT_INFO("Searching for traj_timing files in: %s\n", search_folder.c_str());
    
    for (auto &entry : boost::filesystem::recursive_directory_iterator(search_folder)) {
      // Skip directories
      if (boost::filesystem::is_directory(entry))
        continue;
      
      // Check if filename starts with "traj_timing"
      std::string filename = entry.path().filename().string();
      if (filename.find("traj_timing") == 0) {
        // Extract both the immediate parent folder and the folder above it
        boost::filesystem::path parent_path = entry.path().parent_path();
        std::string immediate_parent = parent_path.filename().string();
        
        // Get the folder above the immediate parent
        boost::filesystem::path grandparent_path = parent_path.parent_path();
        std::string grandparent = grandparent_path.filename().string();
        
        // Combine the two folder names
        std::string combined_name;
        if (parent_path == search_folder) {
          // If the parent is the root search folder, just use the filename
          combined_name = entry.path().stem().string();
        } else if (grandparent_path == search_folder || grandparent.empty()) {
          // If grandparent is the search folder or empty, use only immediate parent
          combined_name = immediate_parent;
        } else {
          // Combine grandparent and immediate parent
          combined_name = grandparent + "_" + immediate_parent;
        }
        
        timing_files.push_back(std::make_pair(combined_name, entry.path().string()));
        PRINT_DEBUG("Found traj_timing file: %s -> %s\n", combined_name.c_str(), entry.path().string().c_str());
      }
    }
    
    // Sort by name for consistent processing
    std::sort(timing_files.begin(), timing_files.end());
    
    if (timing_files.empty()) {
      PRINT_ERROR(RED "ERROR: No traj_timing files found in %s\n" RESET, search_folder.c_str());
      std::exit(EXIT_FAILURE);
    }
    
    PRINT_INFO("Found %d traj_timing files to process\n", (int)timing_files.size());
  } else {
    // Individual files mode (original behavior)
    for (int z = 1; z < argc; z++) {
      boost::filesystem::path path(argv[z]);
      std::string name = path.stem().string();
      timing_files.push_back(std::make_pair(name, std::string(argv[z])));
    }
  }

  // Read in all our trajectories from file
  std::vector<std::string> names;
  std::vector<ov_eval::Statistics> total_times;
  PRINT_INFO("======================================\n");
  for (size_t z = 0; z < timing_files.size(); z++) {

    // Parse the name of this timing
    std::string name = timing_files.at(z).first;
    std::string file_path = timing_files.at(z).second;
    PRINT_INFO("[TIME]: loading data for %s\n", name.c_str());

    // Load it!!
    std::vector<std::string> names_temp;
    std::vector<double> times;
    std::vector<Eigen::VectorXd> timing_values;
    ov_eval::Loader::load_timing_flamegraph(file_path, names_temp, times, timing_values);
    PRINT_DEBUG("[TIME]: loaded %d timestamps from file (%d categories)!!\n", (int)times.size(), (int)names_temp.size());

    // Our categories
    std::vector<ov_eval::Statistics> stats;
    for (size_t i = 0; i < names_temp.size(); i++)
      stats.push_back(ov_eval::Statistics());

    // Loop through each and report the average timing information
    for (size_t i = 0; i < times.size(); i++) {
      for (size_t c = 0; c < names_temp.size(); c++) {
        stats.at(c).timestamps.push_back(times.at(i));
        stats.at(c).values.push_back(timing_values.at(i)(c));
      }
    }

    // Now print the statistic for this run
    for (size_t i = 0; i < names_temp.size(); i++) {
      stats.at(i).calculate();
      PRINT_INFO("mean_time = %.4f | std = %.4f | 99th = %.4f  | max = %.4f (%s)\n", stats.at(i).mean, stats.at(i).std,
                 stats.at(i).ninetynine, stats.at(i).max, names_temp.at(i).c_str());
    }

    // Append the total stats to the big vector
    if (!stats.empty()) {
      names.push_back(name);
      total_times.push_back(stats.at(stats.size() - 1));
    } else {
      PRINT_ERROR(RED "[TIME]: unable to load any data.....\n" RESET);
    }
    PRINT_INFO("======================================\n");
  }

  // Calculate and print mean rates and minimum rates for each timing file
  PRINT_INFO("======================================\n");
  PRINT_INFO("RATES (Hz):\n");
  for (size_t i = 0; i < names.size(); i++) {
    if (total_times.at(i).mean > 0 && total_times.at(i).max > 0) {
      double mean_rate = 1.0 / total_times.at(i).mean;
      double min_rate = 1.0 / total_times.at(i).max;  // Minimum rate occurs at maximum time
      PRINT_INFO("%s: mean=%.2f Hz, min=%.2f Hz (times: mean=%.4fs, max=%.4fs)\n", 
                 names.at(i).c_str(), mean_rate, min_rate, 
                 total_times.at(i).mean, total_times.at(i).max);
    } else {
      PRINT_INFO("%s: undefined (mean time = 0)\n", names.at(i).c_str());
    }
  }
  PRINT_INFO("======================================\n");

#ifdef HAVE_PYTHONLIBS

  // Valid colors
  // https://matplotlib.org/stable/tutorials/colors/colors.html
  // std::vector<std::string> colors = {"blue","aqua","lightblue","lightgreen","yellowgreen","green"};
  // std::vector<std::string> colors = {"navy","blue","lightgreen","green","gold","goldenrod"};
  std::vector<std::string> colors = {"black", "blue", "red", "green", "cyan", "magenta"};

  // Plot this figure
  matplotlibcpp::figure_size(1200, 400);

  // Zero our time arrays
  double starttime = (total_times.at(0).timestamps.empty()) ? 0 : total_times.at(0).timestamps.at(0);
  double endtime = (total_times.at(0).timestamps.empty()) ? 0 : total_times.at(0).timestamps.at(total_times.at(0).timestamps.size() - 1);
  for (size_t i = 0; i < total_times.size(); i++) {
    for (size_t j = 0; j < total_times.at(i).timestamps.size(); j++) {
      total_times.at(i).timestamps.at(j) -= starttime;
    }
  }

  // Now loop through each and plot it!
  for (size_t n = 0; n < names.size(); n++) {

    // Sub-sample the time and values
    int keep_every = 10;
    std::vector<double> times_skipped;
    for (size_t t = 0; t < total_times.at(n).timestamps.size(); t++) {
      if (t % keep_every == 0) {
        times_skipped.push_back(total_times.at(n).timestamps.at(t));
      }
    }
    std::vector<double> values_skipped;
    for (size_t t = 0; t < total_times.at(n).values.size(); t++) {
      if (t % keep_every == 0) {
        values_skipped.push_back(total_times.at(n).values.at(t));
      }
    }

    // Paramters for our line
    std::map<std::string, std::string> params;
    params.insert({"label", names.at(n)});
    params.insert({"linestyle", "-"});
    params.insert({"color", colors.at(n % colors.size())});

    // Finally plot
    matplotlibcpp::plot(times_skipped, values_skipped, params);
  }

  // Finally add labels and show it
  matplotlibcpp::ylabel("execution time (s)");
  matplotlibcpp::xlim(0.0, endtime - starttime);
  matplotlibcpp::xlabel("dataset time (s)");
  matplotlibcpp::legend();
  matplotlibcpp::tight_layout();

  // Display to the user
  matplotlibcpp::show(true);

#endif

  // Done!
  return EXIT_SUCCESS;
}
