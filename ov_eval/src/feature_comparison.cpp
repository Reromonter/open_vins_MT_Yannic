// Author @Yannic Hofmann

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
             "Total Triangulated Features",
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
  ov_core::Printer::setPrintLevel("INFO");

  if (argc < 3) {
    PRINT_ERROR(RED "ERROR: Please specify at least two feature files to compare\n" RESET);
    PRINT_ERROR(RED "ERROR: ./feature_comparison <file1_features.txt> <file2_features.txt> [more files ...] [name1] [name2] [...names]\n" RESET);
    std::exit(EXIT_FAILURE);
  }

  // Collect file paths and optional names
  std::vector<std::string> files;
  std::vector<std::string> names;
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (boost::algorithm::ends_with(arg, ".txt")) {
      files.push_back(arg);
    } else {
      names.push_back(arg);
    }
  }
  if (files.size() < 2) {
    PRINT_ERROR(RED "ERROR: Need at least two feature files (.txt) as input\n" RESET);
    std::exit(EXIT_FAILURE);
  }
  while (names.size() < files.size()) names.push_back("File" + std::to_string(names.size() + 1));

  // Load all feature data
  std::vector<FeatureData> all_data;
  for (size_t i = 0; i < files.size(); ++i) {
    PRINT_INFO("[FEAT]: Loading feature data from %s as '%s'\n", files[i].c_str(), names[i].c_str());
    all_data.push_back(load_feature_data(files[i], names[i]));
    PRINT_INFO("[FEAT]: Loaded %d timestamps from %s\n", (int)all_data.back().timestamps.size(), files[i].c_str());
  }

  // Display statistics for all pairs
  PRINT_INFO("\n===== FEATURE STATISTICS COMPARISON =====\n");
  for (size_t i = 0; i < all_data.size(); ++i) {
    for (size_t j = i + 1; j < all_data.size(); ++j) {
      PRINT_INFO("\n--- Comparing '%s' vs '%s' ---\n", all_data[i].name.c_str(), all_data[j].name.c_str());
      display_statistics(all_data[i], all_data[j]);
    }
  }

#ifdef HAVE_PYTHONLIBS
  // Normalize timestamps to start from zero for each file
  std::vector<std::vector<double>> times;
  for (const auto &data : all_data) {
    std::vector<double> t = data.timestamps;
    double t0 = t.empty() ? 0.0 : t.front();
    for (auto &v : t) v -= t0;
    times.push_back(std::move(t));
  }

  plt::figure_size(1200, 900);

  // MSCKF Features
  plt::subplot(3, 1, 1);
  for (size_t i = 0; i < all_data.size(); ++i)
    plt::named_plot(all_data[i].name, times[i], all_data[i].msckf_features);
  plt::ylabel("MSCKF Features");
  plt::xlabel("Time (seconds)");
  plt::grid(true);
  plt::legend();
  plt::title("MSCKF Features over Time");

  // SLAM Features
  plt::subplot(3, 1, 2);
  for (size_t i = 0; i < all_data.size(); ++i)
    plt::named_plot(all_data[i].name, times[i], all_data[i].slam_features);
  plt::ylabel("SLAM Features");
  plt::xlabel("Time (seconds)");
  plt::grid(true);
  plt::legend();
  plt::title("SLAM Features over Time");

  // Triangulated Features
  plt::subplot(3, 1, 3);
  for (size_t i = 0; i < all_data.size(); ++i)
    plt::named_plot(all_data[i].name, times[i], all_data[i].total_features);
  plt::ylabel("Triangulated Features");
  plt::xlabel("Time (seconds)");
  plt::grid(true);
  plt::legend();
  plt::title("Triangulated Features over Time");

  plt::tight_layout();
  plt::show();
#else
  PRINT_WARNING(YELLOW "Python libraries not found, skipping plot generation\n" RESET);
#endif

  return EXIT_SUCCESS;
}
