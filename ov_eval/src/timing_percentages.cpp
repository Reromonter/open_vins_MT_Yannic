/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 * Adapted Yannic Hofmann 2025
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
  ov_core::Printer::setPrintLevel("ALL");

  // Ensure we have a path
  if (argc < 2) {
    PRINT_ERROR(RED "ERROR: Please specify a folder to search for psutil_log files\n" RESET);
    PRINT_ERROR(RED "ERROR: ./timing_percentages <search_folder>\n" RESET);
    PRINT_ERROR(RED "ERROR: rosrun ov_eval timing_percentages <search_folder>\n" RESET);
    std::exit(EXIT_FAILURE);
  }

  // Search for all psutil_log files in the specified folder and subfolders
  std::string search_folder(argv[1]);
  std::vector<std::pair<std::string, boost::filesystem::path>> psutil_files; // pair of <subfolder_name, file_path>
  
  // Recursively search for psutil_log files
  for (auto &entry : boost::filesystem::recursive_directory_iterator(search_folder)) {
    // Skip directories
    if (boost::filesystem::is_directory(entry))
      continue;
    
    // Check if filename starts with "psutil_log"
    std::string filename = entry.path().filename().string();
    if (filename.find("psutil_log") == 0) {
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
      
      psutil_files.push_back(std::make_pair(combined_name, entry.path()));
      PRINT_DEBUG("Found psutil_log file: %s -> %s\n", combined_name.c_str(), entry.path().string().c_str());
    }
  }
  
  // Sort by subfolder name for consistent processing
  std::sort(psutil_files.begin(), psutil_files.end());

  //===============================================================================
  //===============================================================================
  //===============================================================================

  // Check if we found any psutil_log files
  if (psutil_files.empty()) {
    PRINT_ERROR(RED "ERROR: No psutil_log files found in %s\n" RESET, search_folder.c_str());
    std::exit(EXIT_FAILURE);
  }

  PRINT_INFO("Found %d psutil_log files to process\n", (int)psutil_files.size());

  // Summary information (%cpu, %mem, threads)
  std::map<std::string, std::vector<ov_eval::Statistics>> algo_timings;
  for (const auto &file_pair : psutil_files) {
    std::vector<ov_eval::Statistics> temp = {ov_eval::Statistics(), ov_eval::Statistics(), ov_eval::Statistics()};
    algo_timings.insert({file_pair.first, temp});
  }

  // Loop through each psutil_log file
  for (size_t i = 0; i < psutil_files.size(); i++) {
    
    std::string subfolder_name = psutil_files.at(i).first;
    boost::filesystem::path file_path = psutil_files.at(i).second;

    // Debug print
    PRINT_DEBUG("======================================\n");
    PRINT_DEBUG("[COMP]: processing %s from %s\n", subfolder_name.c_str(), file_path.string().c_str());

    // Load the data from the psutil_log file
    std::vector<double> times;
    std::vector<Eigen::Vector3d> summed_values;
    std::vector<Eigen::VectorXd> node_values;
    ov_eval::Loader::load_timing_percent(file_path.string(), times, summed_values, node_values);

    // Store the data in our statistics map, filtering out CPU values below 10%
    int filtered_count = 0;
    for (size_t j = 0; j < times.size(); j++) {
      // Skip entries where CPU usage is below 10%
      if (summed_values.at(j)(0) < 10.0) {
        filtered_count++;
        continue;
      }
      
      algo_timings.at(subfolder_name).at(0).timestamps.push_back(times.at(j));
      algo_timings.at(subfolder_name).at(0).values.push_back(summed_values.at(j)(0));
      algo_timings.at(subfolder_name).at(1).timestamps.push_back(times.at(j));
      algo_timings.at(subfolder_name).at(1).values.push_back(summed_values.at(j)(1));
      algo_timings.at(subfolder_name).at(2).timestamps.push_back(times.at(j));
      algo_timings.at(subfolder_name).at(2).values.push_back(summed_values.at(j)(2));
    }
    
    if (filtered_count > 0) {
      PRINT_DEBUG("\tFiltered out %d entries with CPU < 10%%\n", filtered_count);
    }

    // Display statistics for the user
    PRINT_DEBUG("\tloaded %d timestamps from file!!\n", (int)algo_timings.at(subfolder_name).at(0).timestamps.size());
    algo_timings.at(subfolder_name).at(0).calculate();
    algo_timings.at(subfolder_name).at(1).calculate();
    algo_timings.at(subfolder_name).at(2).calculate();
    PRINT_DEBUG("\tCPU: mean = %.3f +- %.3f\n", algo_timings.at(subfolder_name).at(0).mean, algo_timings.at(subfolder_name).at(0).std);
    PRINT_DEBUG("\tMEM: mean = %.3f +- %.3f\n", algo_timings.at(subfolder_name).at(1).mean, algo_timings.at(subfolder_name).at(1).std);
    PRINT_DEBUG("\tTHR: mean = %.3f +- %.3f\n", algo_timings.at(subfolder_name).at(2).mean, algo_timings.at(subfolder_name).at(2).std);
    PRINT_DEBUG("======================================\n");
  }

  //===============================================================================
  //===============================================================================
  //===============================================================================

  // Generate summary table
  PRINT_INFO("===============================================\n");
  PRINT_INFO("PSUTIL_LOG SUMMARY TABLE\n");
  PRINT_INFO("===============================================\n");
  PRINT_INFO("%-20s | %-12s | %-12s | %-12s\n", "Dataset", "CPU Mean±Std", "MEM Mean±Std", "THR Mean±Std");
  PRINT_INFO("%-20s-+-%-12s-+-%-12s-+-%-12s\n", "--------------------", "------------", "------------", "------------");
  
  for (const auto &algo : algo_timings) {
    PRINT_INFO("%-20s | %5.1f±%-5.1f | %5.3f±%-5.2f | %5.1f±%-5.1f\n",
               algo.first.c_str(),
               algo.second.at(0).mean, algo.second.at(0).std,
               algo.second.at(1).mean, algo.second.at(1).std,
               algo.second.at(2).mean, algo.second.at(2).std);
  }
  PRINT_INFO("===============================================\n");

#ifdef HAVE_PYTHONLIBS

  // Plot line colors
  std::vector<std::string> colors = {"blue", "red", "black", "green", "cyan", "magenta"};
  std::vector<std::string> linestyle = {"-", "--", "-."};
  assert(algo_timings.size() <= colors.size() * linestyle.size());

  // Parameters
  std::map<std::string, std::string> params_rpe;
  params_rpe.insert({"notch", "false"});
  params_rpe.insert({"sym", ""});

  //============================================================
  //============================================================
  // Plot this figure
  matplotlibcpp::figure_size(1500, 400);

  // Plot each RPE next to each other
  double width = 0.1 / (algo_timings.size() + 1);
  std::vector<double> yticks;
  std::vector<std::string> labels;
  int ct_algo = 0;
  double ct_pos = 0;
  for (auto &algo : algo_timings) {
    // Start based on what algorithm we are doing
    ct_pos = 1 + 1.5 * ct_algo * width;
    yticks.push_back(ct_pos);
    labels.push_back(algo.first);
    // Plot it!!!
    matplotlibcpp::boxplot(algo.second.at(0).values, ct_pos, width, colors.at(ct_algo % colors.size()),
                           linestyle.at(ct_algo / colors.size()), params_rpe, false);
    // Move forward
    ct_algo++;
  }

  // Add "fake" plots for our legend
  ct_algo = 0;
  for (const auto &algo : algo_timings) {
    std::map<std::string, std::string> params_empty;
    params_empty.insert({"label", algo.first});
    params_empty.insert({"linestyle", linestyle.at(ct_algo / colors.size())});
    params_empty.insert({"color", colors.at(ct_algo % colors.size())});
    std::vector<double> vec_empty;
    matplotlibcpp::plot(vec_empty, vec_empty, params_empty);
    ct_algo++;
  }

  // Display to the user
  matplotlibcpp::ylim(1.0 - 1 * width, ct_pos + 1 * width);
  matplotlibcpp::yticks(yticks, labels);
  matplotlibcpp::xlabel("CPU Percent Usage");
  matplotlibcpp::tight_layout();
  matplotlibcpp::show(false);

  //============================================================
  //============================================================
  // Plot this figure
  matplotlibcpp::figure_size(1500, 400);

  // Plot each RPE next to each other
  width = 0.1 / (algo_timings.size() + 1);
  yticks.clear();
  labels.clear();
  ct_algo = 0;
  ct_pos = 0;
  for (auto &algo : algo_timings) {
    // Start based on what algorithm we are doing
    ct_pos = 1 + 1.5 * ct_algo * width;
    yticks.push_back(ct_pos);
    labels.push_back(algo.first);
    // Plot it!!!
    matplotlibcpp::boxplot(algo.second.at(1).values, ct_pos, width, colors.at(ct_algo % colors.size()),
                           linestyle.at(ct_algo / colors.size()), params_rpe, false);
    // Move forward
    ct_algo++;
  }

  // Add "fake" plots for our legend
  ct_algo = 0;
  for (const auto &algo : algo_timings) {
    std::map<std::string, std::string> params_empty;
    params_empty.insert({"label", algo.first});
    params_empty.insert({"linestyle", linestyle.at(ct_algo / colors.size())});
    params_empty.insert({"color", colors.at(ct_algo % colors.size())});
    std::vector<double> vec_empty;
    matplotlibcpp::plot(vec_empty, vec_empty, params_empty);
    ct_algo++;
  }

  // Display to the user
  matplotlibcpp::ylim(1.0 - 1 * width, ct_pos + 1 * width);
  matplotlibcpp::yticks(yticks, labels);
  matplotlibcpp::xlabel("Memory Percent Usage");
  matplotlibcpp::tight_layout();
  matplotlibcpp::show(true);

#endif

  // Done!
  return EXIT_SUCCESS;
}
