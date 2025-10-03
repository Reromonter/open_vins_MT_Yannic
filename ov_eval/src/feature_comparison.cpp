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
#include <vector>
#include <map>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <numeric>   // for std::accumulate

#include "utils/Loader.h"
#include "utils/Statistics.h"
#include "utils/colors.h"
#include "utils/print.h"

#ifdef HAVE_PYTHONLIBS
// import the c++ wrapper for matplotlib
#include "plot/matplotlibcpp.h"
namespace plt = matplotlibcpp;
#endif

/**
 * @brief Structure to hold feature data from a file
 */
struct FeatureData {
  std::vector<double> timestamps;
  std::vector<double> msckf_features;
  std::vector<double> slam_features;
  std::vector<double> total_features;
  std::string name;

  // Statistics
  ov_eval::Statistics stats_msckf;
  ov_eval::Statistics stats_slam;
  ov_eval::Statistics stats_total;

  // Calculate usage frequency (percentage of non-zero entries)
  double msckf_frequency() const {
    int count = 0;
    for (const auto &val : msckf_features) {
      if (val > 0) count++;
    }
    return msckf_features.empty() ? 0.0
                                  : static_cast<double>(count) / static_cast<double>(msckf_features.size()) * 100.0;
  }

  double slam_frequency() const {
    int count = 0;
    for (const auto &val : slam_features) {
      if (val > 0) count++;
    }
    return slam_features.empty() ? 0.0
                                 : static_cast<double>(count) / static_cast<double>(slam_features.size()) * 100.0;
  }
};

/**
 * @brief Load feature data from traj_features.txt file
 * @param filepath Path to the feature data file
 * @param name Name to associate with this data (for display)
 * @return Loaded feature data
 */
FeatureData load_feature_data(const std::string &filepath, const std::string &name) {
  // Create return structure
  FeatureData data;
  data.name = name;

  // Open the file
  std::ifstream file(filepath);
  if (!file.is_open()) {
    PRINT_ERROR(RED "ERROR: Unable to open feature file %s\n" RESET, filepath.c_str());
    std::exit(EXIT_FAILURE);
  }

  // Read the file line by line
  std::string line;
  while (std::getline(file, line)) {
    // Skip header/comment lines
    if (line.empty() || line[0] == '#') {
      continue;
    }

    // Parse the line
    std::stringstream ss(line);
    std::string token;

    // Parse timestamp
    if (!std::getline(ss, token, ',')) continue;
    double timestamp = std::stod(token);
    data.timestamps.push_back(timestamp);

    // Parse MSCKF features
    if (!std::getline(ss, token, ',')) continue;
    data.msckf_features.push_back(static_cast<double>(std::stoi(token)));

    // Parse SLAM features
    if (!std::getline(ss, token, ',')) continue;
    data.slam_features.push_back(static_cast<double>(std::stoi(token)));

    // Parse total features
    if (!std::getline(ss, token, ',')) continue;
    data.total_features.push_back(static_cast<double>(std::stoi(token)));
  }

  // Close the file
  file.close();

  // Compute statistics
  for (size_t i = 0; i < data.timestamps.size(); i++) {
    data.stats_msckf.timestamps.push_back(data.timestamps[i]);
    data.stats_msckf.values.push_back(data.msckf_features[i]);

    data.stats_slam.timestamps.push_back(data.timestamps[i]);
    data.stats_slam.values.push_back(data.slam_features[i]);

    data.stats_total.timestamps.push_back(data.timestamps[i]);
    data.stats_total.values.push_back(data.total_features[i]);
  }

  data.stats_msckf.calculate();
  data.stats_slam.calculate();
  data.stats_total.calculate();

  return data;
}

/**
 * @brief Display comparison statistics
 * @param data1 First dataset
 * @param data2 Second dataset
 */
void display_statistics(const FeatureData &data1, const FeatureData &data2) {
  PRINT_INFO("\n===== FEATURE STATISTICS COMPARISON =====\n\n");

  // helper sums (Statistics has no 'sum' field)
  auto sum_vec = [](const std::vector<double> &v) {
    return std::accumulate(v.begin(), v.end(), 0.0);
  };
  double msckf_sum1 = sum_vec(data1.msckf_features);
  double msckf_sum2 = sum_vec(data2.msckf_features);
  double slam_sum1  = sum_vec(data1.slam_features);
  double slam_sum2  = sum_vec(data2.slam_features);
  double total_sum1 = sum_vec(data1.total_features);
  double total_sum2 = sum_vec(data2.total_features);

  // Table header
  PRINT_INFO("%-15s | %-30s | %-30s | %-15s\n", "Feature Type", data1.name.c_str(), data2.name.c_str(), "Difference");
  PRINT_INFO("---------------------------------------------------------------------------------------------\n");

  // MSCKF features
  double msckf_diff = data1.stats_msckf.mean - data2.stats_msckf.mean;
  double msckf_pct = (data2.stats_msckf.mean > 0) ? (msckf_diff / data2.stats_msckf.mean) * 100.0 : 0.0;

  PRINT_INFO("%-15s | mean: %-5.2f, max: %-3.0f, sum: %-7.0f | mean: %-5.2f, max: %-3.0f, sum: %-7.0f | %-5.2f (%+.2f%%)\n",
             "MSCKF Features",
             data1.stats_msckf.mean, data1.stats_msckf.max, msckf_sum1,
             data2.stats_msckf.mean, data2.stats_msckf.max, msckf_sum2,
             msckf_diff, msckf_pct);

  // SLAM features
  double slam_diff = data1.stats_slam.mean - data2.stats_slam.mean;
  double slam_pct = (data2.stats_slam.mean > 0) ? (slam_diff / data2.stats_slam.mean) * 100.0 : 0.0;

  PRINT_INFO("%-15s | mean: %-5.2f, max: %-3.0f, sum: %-7.0f | mean: %-5.2f, max: %-3.0f, sum: %-7.0f | %-5.2f (%+.2f%%)\n",
             "SLAM Features",
             data1.stats_slam.mean, data1.stats_slam.max, slam_sum1,
             data2.stats_slam.mean, data2.stats_slam.max, slam_sum2,
             slam_diff, slam_pct);

  // Total features
  double total_diff = data1.stats_total.mean - data2.stats_total.mean;
  double total_pct = (data2.stats_total.mean > 0) ? (total_diff / data2.stats_total.mean) * 100.0 : 0.0;

  PRINT_INFO("%-15s | mean: %-5.2f, max: %-3.0f, sum: %-7.0f | mean: %-5.2f, max: %-3.0f, sum: %-7.0f | %-5.2f (%+.2f%%)\n",
             "Total Features",
             data1.stats_total.mean, data1.stats_total.max, total_sum1,
             data2.stats_total.mean, data2.stats_total.max, total_sum2,
             total_diff, total_pct);

  PRINT_INFO("---------------------------------------------------------------------------------------------\n");

  // Feature usage frequency
  PRINT_INFO("\n===== FEATURE USAGE FREQUENCY (how often not 0) =====\n\n");
  PRINT_INFO("%-15s | %-15s | %-15s | %-15s\n", "Feature Type", data1.name.c_str(), data2.name.c_str(), "Difference");
  PRINT_INFO("----------------------------------------------------------------------\n");

  double msckf_freq1 = data1.msckf_frequency();
  double msckf_freq2 = data2.msckf_frequency();
  double msckf_freq_diff = msckf_freq1 - msckf_freq2;

  PRINT_INFO("%-15s | %13.2f%% | %13.2f%% | %+.2f%%\n", "MSCKF Features", msckf_freq1, msckf_freq2, msckf_freq_diff);

  double slam_freq1 = data1.slam_frequency();
  double slam_freq2 = data2.slam_frequency();
  double slam_freq_diff = slam_freq1 - slam_freq2;

  PRINT_INFO("%-15s | %13.2f%% | %13.2f%% | %+.2f%%\n", "SLAM Features", slam_freq1, slam_freq2, slam_freq_diff);

  PRINT_INFO("----------------------------------------------------------------------\n");
}

int main(int argc, char **argv) {
  // Verbosity setting
  ov_core::Printer::setPrintLevel("INFO");

  // Ensure we have paths to the two files
  if (argc < 3) {
    PRINT_ERROR(RED "ERROR: Please specify two feature files to compare\n" RESET);
    PRINT_ERROR(RED "ERROR: ./feature_comparison <file1_features.txt> <file2_features.txt> [name1] [name2]\n" RESET);
    PRINT_ERROR(RED "ERROR: rosrun ov_eval feature_comparison <file1_features.txt> <file2_features.txt> [name1] [name2]\n" RESET);
    std::exit(EXIT_FAILURE);
  }

  // Get the filenames
  std::string file1 = argv[1];
  std::string file2 = argv[2];

  // Get optional display names
  std::string name1 = "File1";
  std::string name2 = "File2";
  if (argc >= 4) name1 = argv[3];
  if (argc >= 5) name2 = argv[4];

  // Load feature data from both files
  PRINT_INFO("[FEAT]: Loading feature data from files...\n");
  FeatureData data1 = load_feature_data(file1, name1);
  FeatureData data2 = load_feature_data(file2, name2);

  PRINT_INFO("[FEAT]: Loaded %d timestamps from %s\n", (int)data1.timestamps.size(), file1.c_str());
  PRINT_INFO("[FEAT]: Loaded %d timestamps from %s\n", (int)data2.timestamps.size(), file2.c_str());

  // Display statistics
  display_statistics(data1, data2);

#ifdef HAVE_PYTHONLIBS
  // Normalize timestamps to start from zero
  double starttime1 = data1.timestamps.empty() ? 0.0 : data1.timestamps.front();
  double starttime2 = data2.timestamps.empty() ? 0.0 : data2.timestamps.front();

  std::vector<double> times1 = data1.timestamps;
  std::vector<double> times2 = data2.timestamps;
  for (auto &t : times1) t -= starttime1;
  for (auto &t : times2) t -= starttime2;

  plt::figure_size(1200, 900);

  // MSCKF
  plt::subplot(3, 1, 1);
  plt::named_plot(name1, times1, data1.msckf_features, "b-");
  plt::named_plot(name2, times2, data2.msckf_features, "r-");
  plt::ylabel("MSCKF Features");
  plt::grid(true);
  plt::legend();
  plt::title("MSCKF Features over Time");

  // SLAM
  plt::subplot(3, 1, 2);
  plt::named_plot(name1, times1, data1.slam_features, "b-");
  plt::named_plot(name2, times2, data2.slam_features, "r-");
  plt::ylabel("SLAM Features");
  plt::grid(true);
  plt::legend();
  plt::title("SLAM Features over Time");

  // Total
  plt::subplot(3, 1, 3);
  plt::named_plot(name1, times1, data1.total_features, "b-");
  plt::named_plot(name2, times2, data2.total_features, "r-");
  plt::ylabel("Total Features");
  plt::xlabel("Time (seconds)");
  plt::grid(true);
  plt::legend();
  plt::title("Total Features over Time");

  plt::tight_layout();
  plt::show();
#else
  PRINT_WARNING(YELLOW "Python libraries not found, skipping plot generation\n" RESET);
#endif

  // Done!
  return EXIT_SUCCESS;
}
