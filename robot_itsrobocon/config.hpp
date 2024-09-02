
#ifndef _ITS_CONFIG_HPP_
#define _ITS_CONFIG_HPP_

#include <iostream>
#include "file_manager/file_manager.h"
#include <algorithm>
#include <string>
#include <vector>

class config {
private:
    inline static std::vector<std::string> name_container;
    inline static std::vector<float> val_container;
    uint32_t index;
public:

    static void init(FileManager &file) {
        std::cout << "Proses penguraian konfigurasi dimulai" << std::endl;
        name_container.clear();
        val_container.clear();
        for(auto i = file.file_data.begin(); i != file.file_data.end(); i++) {
            if(i->data.at(1) == ":" && i->data.size() == 3) {
                auto &first_word = i->data.at(0);
                if(std::find(name_container.begin(), name_container.end(), first_word) != name_container.end()) {
                    std::cerr << "keanehan di baris " << i->line_number << " | nama konfigurasi terduplikat!" << std::endl;
                }
                else {
                    float temp = 0;
                    try {
                        temp = std::stof(i->data.at(2));
                        name_container.push_back(first_word);
                        val_container.push_back(temp);
                    }
                    catch(...) {
                        try {
                            auto n = std::find(name_container.begin(), name_container.end(), i->data.at(2));
                            if(i->data.at(2) == "TRUE") temp = 1;
                            else if(i->data.at(2) == "FALSE") temp = 0;
                            else if(i->data.size() == 3 && n != name_container.end()) {
                                auto index = std::distance(name_container.begin(), n);
                                temp = val_container.at(index);
                            }
                            else throw;
                            name_container.push_back(first_word);
                            val_container.push_back(temp);
                        }
                        catch (...) {
                        std::cerr << "keanehan di baris " << i->line_number << " | kata \"" << i->data.at(2) << "\" tidak diketahui!" << std::endl;
                        }
                    }
                }
            }
            else {
                std::cerr << "keanehan di baris " << i->line_number << " | konfigurasi tidak dapat diurai!" << std::endl;
            }
        }
        std::cout << "Proses penguraian konfigurasi selesai" << std::endl;
    }

    static float get_value(std::string str) {
        auto i = std::find(name_container.begin(), name_container.end(), str);
        if(i == name_container.end()) {
            std::cerr << "Konfigurasi " << str << " tidak ditemukan!" << std::endl;
            return 0;
        }
        auto index = std::distance(name_container.begin(), i);
        return val_container.at(index);
    }

    config(std::string str) {
        auto i = std::find(name_container.begin(), name_container.end(), str);
        if(i == name_container.end()) {
            std::cerr << "Konfigurasi " << str << " tidak ditemukan!" << std::endl;
            index = (uint32_t)-1;
            return;
        }
        index = std::distance(name_container.begin(), i);
    }

    float get_value() {
        if(index == (uint32_t)-1) return 0;
        return val_container.at(index);
    }
};

#endif //_ITS_CONFIG_HPP_
