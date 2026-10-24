#include <iostream>
#include <boost/filesystem.hpp>
#include <vector>
#include <string>
#include <cstdlib>

namespace fs = boost::filesystem;

int main(int argc, char** argv) {
    std::string folder;
    if (argc > 1) {
        folder = argv[1];
    } else {
        std::cout << "Enter the folder path to search for rosbag files: ";
        std::getline(std::cin, folder);
    }

    std::vector<fs::path> bagfiles;
    fs::path folder_path(folder);
    if (!fs::exists(folder_path) || !fs::is_directory(folder_path)) {
        std::cout << "Invalid folder path.\n";
        return 1;
    }

    for (const auto& entry : fs::directory_iterator(folder_path)) {
        if (fs::is_regular_file(entry.path())) {
            auto ext = entry.path().extension().string();
            std::cout << "Found file: " << entry.path().string() << "\n";
            if (ext == ".bag") {
                bagfiles.push_back(entry.path());
            }
        }
    }

    if (bagfiles.empty()) {
        std::cout << "No rosbag files found in the folder.\n";
        return 1;
    }

    std::cout << "Found rosbag files:\n";
    for (size_t i = 0; i < bagfiles.size(); ++i) {
        std::cout << i << ": " << bagfiles[i] << "\n";
    }

    std::cout << "Enter the number of the bagfile to play: ";
    unsigned int choice;
    std::cin >> choice;

    if (choice >= bagfiles.size()) {
        std::cout << "Invalid choice.\n";
        return 1;
    }

    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Clear input buffer

    std::string remap_from, remap_to, remap_arg;
    std::cout << "Do you want to remap a topic? (y/n): ";
    std::string remap_answer;
    std::getline(std::cin, remap_answer);

    if (!remap_answer.empty() && (remap_answer[0] == 'y' || remap_answer[0] == 'Y')) {
        std::cout << "Enter the topic to remap from (e.g. /old_topic): ";
        std::getline(std::cin, remap_from);
        std::cout << "Enter the topic to remap to (e.g. /new_topic): ";
        std::getline(std::cin, remap_to);
        if (!remap_from.empty() && !remap_to.empty()) {
            remap_arg = " " + remap_from + ":=" + remap_to;
        }
    }

    std::string cmd = "rosbag play \"" + bagfiles[choice].string() + "\"" + remap_arg;
    std::cout << "Running: " << cmd << std::endl;
    int ret = std::system(cmd.c_str());
    return ret;
}