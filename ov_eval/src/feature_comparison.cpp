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
 * @brief Display statistics for all datasets
 * @param all_data Vector of all feature datasets
 */
void display_statistics(const std::vector<FeatureData> &all_data) {
  PRINT_INFO("\n===== FEATURE STATISTICS =====\n\n");

  // helper sums (Statistics has no 'sum' field)
  auto sum_vec = [](const std::vector<double> &v) {
    return std::accumulate(v.begin(), v.end(), 0.0);
  };

  // Table header
  PRINT_INFO("%-25s | %-10s | %-10s | %-10s | %-10s | %-10s | %-10s\n", 
             "Dataset Name", "MSCKF Mean", "MSCKF Max", "MSCKF Sum", "SLAM Mean", "SLAM Max", "SLAM Sum");
  PRINT_INFO("---------------------------------------------------------------------------------------------------------------------------\n");

  for (const auto &data : all_data) {
    double msckf_sum = sum_vec(data.msckf_features);
    double slam_sum = sum_vec(data.slam_features);
    
    PRINT_INFO("%-25s | %10.2f | %10.0f | %10.0f | %10.2f | %10.0f | %10.0f\n",
               data.name.c_str(),
               data.stats_msckf.mean, data.stats_msckf.max, msckf_sum,
               data.stats_slam.mean, data.stats_slam.max, slam_sum);
  }

  PRINT_INFO("---------------------------------------------------------------------------------------------------------------------------\n");

  // Total triangulated features table
  PRINT_INFO("\n%-25s | %-10s | %-10s | %-10s\n", 
             "Dataset Name", "Total Mean", "Total Max", "Total Sum");
  PRINT_INFO("--------------------------------------------------------------\n");

  for (const auto &data : all_data) {
    double total_sum = sum_vec(data.total_features);
    
    PRINT_INFO("%-25s | %10.2f | %10.0f | %10.0f\n",
               data.name.c_str(),
               data.stats_total.mean, data.stats_total.max, total_sum);
  }

  PRINT_INFO("--------------------------------------------------------------\n");

  // Feature usage frequency
  PRINT_INFO("\n===== FEATURE USAGE FREQUENCY (how often not 0) =====\n\n");
  PRINT_INFO("%-25s | %-15s | %-15s\n", "Dataset Name", "MSCKF Freq (%)", "SLAM Freq (%)");
  PRINT_INFO("--------------------------------------------------------------\n");

  for (const auto &data : all_data) {
    double msckf_freq = data.msckf_frequency();
    double slam_freq = data.slam_frequency();

    PRINT_INFO("%-25s | %14.2f%% | %14.2f%%\n", data.name.c_str(), msckf_freq, slam_freq);
  }

  PRINT_INFO("--------------------------------------------------------------\n");
}

int main(int argc, char **argv) {
  ov_core::Printer::setPrintLevel("INFO");

  if (argc < 2) {
    PRINT_ERROR(RED "ERROR: Please specify feature files or a folder to search\n" RESET);
    PRINT_ERROR(RED "ERROR: ./feature_comparison <file1_features.txt> <file2_features.txt> [more files ...] [name1] [name2] [...names]\n" RESET);
    PRINT_ERROR(RED "ERROR: ./feature_comparison <search_folder>\n" RESET);
    std::exit(EXIT_FAILURE);
  }

  // Determine if we have a folder search or individual files
  std::vector<std::pair<std::string, std::string>> feature_files; // pair of <name, file_path>
  
  if (argc == 2 && boost::filesystem::is_directory(argv[1])) {
    // Folder search mode - search for traj_features files recursively
    std::string search_folder(argv[1]);
    PRINT_INFO("Searching for traj_features files in: %s\n", search_folder.c_str());
    
    for (auto &entry : boost::filesystem::recursive_directory_iterator(search_folder)) {
      // Skip directories
      if (boost::filesystem::is_directory(entry))
        continue;
      
      // Check if filename starts with "traj_features"
      std::string filename = entry.path().filename().string();
      if (filename.find("traj_features") == 0) {
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
        
        feature_files.push_back(std::make_pair(combined_name, entry.path().string()));
        PRINT_DEBUG("Found traj_features file: %s -> %s\n", combined_name.c_str(), entry.path().string().c_str());
      }
    }
    
    // Sort by name for consistent processing
    std::sort(feature_files.begin(), feature_files.end());
    
    if (feature_files.empty()) {
      PRINT_ERROR(RED "ERROR: No traj_features files found in %s\n" RESET, search_folder.c_str());
      std::exit(EXIT_FAILURE);
    }
    
    PRINT_INFO("Found %d traj_features files to process\n", (int)feature_files.size());
  } else {
    // Individual files mode (original behavior)
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
    
    // Convert to the new format
    for (size_t i = 0; i < files.size(); ++i) {
      feature_files.push_back(std::make_pair(names[i], files[i]));
    }
  }

  // Load all feature data
  std::vector<FeatureData> all_data;
  for (size_t i = 0; i < feature_files.size(); ++i) {
    std::string name = feature_files[i].first;
    std::string filepath = feature_files[i].second;
    PRINT_INFO("[FEAT]: Loading feature data from %s as '%s'\n", filepath.c_str(), name.c_str());
    all_data.push_back(load_feature_data(filepath, name));
    PRINT_INFO("[FEAT]: Loaded %d timestamps from %s\n", (int)all_data.back().timestamps.size(), filepath.c_str());
  }

  // Display statistics for all datasets
  display_statistics(all_data);

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

  // Generate LaTeX table
  PRINT_INFO("\n===== LATEX TABLE =====\n");
  PRINT_INFO("\\begin{table}[htbp]\n");
  PRINT_INFO("\\centering\n");
  PRINT_INFO("\\begin{tabular}{l c c c c c}\n");
  PRINT_INFO("\\hline\n");
  PRINT_INFO("Name & MSCKF Mean & MSCKF non-zero & SLAM Mean & SLAM non-zero & Total Features Mean \\\\\n");
  PRINT_INFO("\\hline\n");
  
  for (const auto &data : all_data) {
    // Calculate percentage of non-zero entries for MSCKF
    double msckf_nonzero_percent = data.msckf_frequency();
    // Calculate percentage of non-zero entries for SLAM
    double slam_nonzero_percent = data.slam_frequency();
    
    PRINT_INFO("%s & %.1f & %.1f\\%% & %.1f & %.1f\\%% & %.1f \\\\\n",
               data.name.c_str(),
               data.stats_msckf.mean,
               msckf_nonzero_percent,
               data.stats_slam.mean,
               slam_nonzero_percent,
               data.stats_total.mean);
  }
  
  PRINT_INFO("\\hline\n");
  PRINT_INFO("\\end{tabular}\n");
  PRINT_INFO("\\caption{Feature Statistics Comparison}\n");
  PRINT_INFO("\\label{tab:feature_stats}\n");
  PRINT_INFO("\\end{table}\n");
  PRINT_INFO("========================\n");

  return EXIT_SUCCESS;
}
