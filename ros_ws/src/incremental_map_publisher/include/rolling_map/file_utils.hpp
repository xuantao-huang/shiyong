#ifndef FILE_UTILS_HPP
#define FILE_UTILS_HPP

#include <unordered_map>
#include <Eigen/Core>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <regex>
#include <cstring>
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <unistd.h>
#include <errno.h>

namespace file_utils
{
    // 将 Eigen::Vector3i 转换为 string
    std::string vector3iToString(const Eigen::Vector3i& vec) {
        std::ostringstream oss;
        for (int i = 0; i < 3; ++i) {
            if (vec[i] < 0) {
                oss << "_";
            }
            if (i > 0) {
                oss << "_";
            }
            oss << std::abs(vec[i]);
        }
        return oss.str();
    }

    // 将 string 转换回 Eigen::Vector3i
    Eigen::Vector3i stringToVector3i(const std::string& str) {
        std::istringstream iss(str);
        std::string token;
        Eigen::Vector3i vec;
        int index = 0;

        while (std::getline(iss, token, '_')) {
            if (token.empty()) {
                // 遇到额外的下划线，表示负数
                if (std::getline(iss, token, '_')) {
                    vec[index] = -std::stoi(token);
                }
            } else {
                vec[index] = std::stoi(token);
            }
            ++index;
            if (index >= 3) break;
        }

        return vec;
    }

    bool isValidFilename(const std::string& filename) {
        // 检查文件名不包含 "."
        if (filename.find('.') != std::string::npos) {
            return false;
        }

        // 检查下划线数量
        int underscoreCount = std::count(filename.begin(), filename.end(), '_');
        if (underscoreCount < 2 || underscoreCount > 5) {
            return false;
        }

        // 检查文件名只包含下划线和数字
        std::regex pattern("^[0-9_]+$");
        return std::regex_match(filename, pattern);
    }


    bool checkAndClearDirectory(const std::string& dirPath) {
        DIR* dir = opendir(dirPath.c_str());
        if (dir == nullptr) {
            std::cerr << "Error opening directory: " << strerror(errno) << std::endl;
            return false;
        }

        std::vector<std::string> filesToDelete;
        struct dirent* entry;
        while ((entry = readdir(dir)) != nullptr) {
            std::string filename = entry->d_name;
            if (filename == "." || filename == "..") {
                continue;
            }

            std::string fullPath = dirPath + "/" + filename;
            struct stat fileStat;
            if (stat(fullPath.c_str(), &fileStat) == -1) {
                std::cerr << "Error getting file stats: " << strerror(errno) << std::endl;
                closedir(dir);
                return false;
            }

            if (S_ISREG(fileStat.st_mode)) {  // 如果是普通文件
                if (!isValidFilename(filename)) {
                    std::cout << "Invalid file found: " << filename << std::endl;
                    std::cout << "Aborting deletion process." << std::endl;
                    closedir(dir);
                    return false;
                }
                filesToDelete.push_back(fullPath);
            }
        }
        closedir(dir);

        if (!filesToDelete.empty()) {
            std::cout << "All files are valid. Proceeding with deletion..." << std::endl;
            for (const auto& file : filesToDelete) {
                if (unlink(file.c_str()) == -1) {
                    std::cerr << "Error deleting file " << file << ": " << strerror(errno) << std::endl;
                } else {
                    std::cout << "Deleted: " << file << std::endl;
                }
            }
            std::cout << "Successfully deleted " << filesToDelete.size() << " files." << std::endl;
        } else {
            std::cout << "No files to delete in the directory." << std::endl;
        }

        return true;
    }


    bool checkAndLoadDirectoryName(const std::string& dirPath, std::vector<std::string> &fileNames) {
        DIR* dir = opendir(dirPath.c_str());
        if (dir == nullptr) {
            std::cerr << "Error opening directory: " << strerror(errno) << std::endl;
            return false;
        }

        std::vector<std::string> filesToLoad;
        fileNames.clear();
        struct dirent* entry;
        while ((entry = readdir(dir)) != nullptr) {
            std::string filename = entry->d_name;
            if (filename == "." || filename == "..") {
                continue;
            }

            std::string fullPath = dirPath + "/" + filename;
            struct stat fileStat;
            if (stat(fullPath.c_str(), &fileStat) == -1) {
                std::cerr << "Error getting file stats: " << strerror(errno) << std::endl;
                closedir(dir);
                return false;
            }

            if (S_ISREG(fileStat.st_mode)) {  // 如果是普通文件
                if (!isValidFilename(filename)) {
                    std::cout << "Invalid file found: " << filename << std::endl;
                    std::cout << "Aborting deletion process." << std::endl;
                    closedir(dir);
                    return false;
                }
                filesToLoad.push_back(fullPath);
                fileNames.push_back(filename);
            }
        }
        closedir(dir);

        if (!filesToLoad.empty()) {
            std::cout << "Successfully find " << filesToLoad.size() << " map files to load." << std::endl;
        } else {
            std::cout << "No map files to load in the directory." << std::endl;
        }

        return true;
    }

    void saveKeysToFile(const std::unordered_map<uint32_t, bool>& occupiedCells, const std::string& file_path) 
    {
        std::ofstream file(file_path, std::ios::binary);
        if (!file) 
        {
            throw std::runtime_error("Unable to open file for writing: " + file_path);
        }

        // 首先写入 key 的数量
        uint32_t count = occupiedCells.size();
        file.write(reinterpret_cast<const char*>(&count), sizeof(count));

        // 然后写入所有的 keys
        for (const auto& pair : occupiedCells) 
        {
            file.write(reinterpret_cast<const char*>(&pair.first), sizeof(uint32_t));
        }

        if (!file) 
        {
            throw std::runtime_error("Error writing to file: " + file_path);
        }

        file.close();
    }

    void loadKeysFromFile(std::unordered_map<uint32_t, bool>& occupiedCells, const std::string& file_path) 
    {
        std::ifstream file(file_path, std::ios::binary);
        if (!file) 
        {
            throw std::runtime_error("Unable to open file for reading: " + file_path);
        }

        // 首先读取 key 的数量
        uint32_t count;
        file.read(reinterpret_cast<char*>(&count), sizeof(count));

        // 预分配哈希表空间以提高性能
        occupiedCells.reserve(count);

        // 然后读取所有的 keys
        uint32_t key;
        for (uint32_t i = 0; i < count; ++i)
        {
            file.read(reinterpret_cast<char*>(&key), sizeof(uint32_t));
            occupiedCells[key] = true;
        }

        if (!file) 
        {
            throw std::runtime_error("Error reading from file: " + file_path);
        }

        file.close();
    }
}

#endif