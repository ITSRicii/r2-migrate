/*
 * joy_config.cpp
 *
 *  Created on: Feb 4, 2024
 *      Author: Zain Irsyad
 */

#ifndef _JOY_CMD_HPP_
#define _JOY_CMD_HPP_

#include <string>
#include <vector>
#include <algorithm>
#include <iostream>
#include "file_manager/file_manager.h"
#include "remote-controller/ds4_define.h"

class JoyCommand {
private:
    struct joyConfig{
        uint32_t button_mask;
        uint32_t axis_index;
    };

    static std::vector<joyConfig> joyConfig_container;
    static std::vector<std::string> name_list;
    static uint32_t button_cond;
    static std::vector<float> axis_val;

    inline static uint32_t get_mask_from_string(std::vector<std::string> &str) {
        uint32_t temp = 0;
        for(auto i = str.begin(); i!= str.end(); i++) {
            auto button_itter = std::find(std::begin(button_str),
                                        std::end(button_str),
                                        *i);
            if(button_itter != std::end(button_str)) {
                int button_index = std::distance(std::begin(button_str),
                                                button_itter);
                temp |= (1<<button_index);
            }
            else {
                throw (int)std::distance(str.begin(), i); 
            }
        }
        return temp;
    }

    inline static uint32_t get_axis_from_string(std::vector<std::string> &str) {
        uint32_t temp = 0;
        for(auto i = str.begin(); i!= str.end(); i++) {
            auto axis_itter = std::find(std::begin(axis_str),
                                        std::end(axis_str),
                                        *i);
            if(axis_itter != std::end(axis_str)) {
                int axe_index = std::distance(std::begin(axis_str),
                                                axis_itter);
                temp = axe_index;
                return temp;
            }
            else {
                throw (int)std::distance(str.begin(), i); 
            }
        }
        return temp;
    }

    uint16_t index;
    bool last_status = false;

public:

    static bool init(FileManager &file) {
        std::cout << "Proses penguraian joystick command config dimulai" << std::endl;
        joyConfig_container.clear();
        try {
            for(auto i = file.file_data.begin(); i != file.file_data.end(); i++) {
                if(i->data.at(1) == ":") {
                    auto &first_word = i->data.at(0);
                    if(std::find(name_list.begin(), name_list.end(), first_word) != name_list.end()) {
                        std::cerr << "nama command terduplikat!" << std::endl;
                        throw i->line_number;
                    }
                    std::vector<std::string> command_str(i->data.begin()+2, i->data.end());
                    joyConfig temp;
                    temp.button_mask = 0;
                    temp.axis_index = 0;
                    try {
                        temp.button_mask = get_mask_from_string(command_str);
                    }
                    catch(int e) {
                        if(e>1) {
                            std::cerr << "command tercampur!" << std::endl;
                            throw i->line_number;
                        }
                        temp.axis_index = get_axis_from_string(command_str);
                        if(temp.button_mask != 0 && temp.axis_index != 0) throw;
                    }
                    name_list.push_back(first_word);
                    joyConfig_container.push_back(temp);
                }
                else {
                    throw i->line_number;
                }
            }
        }
        catch(int ln) {
            std::cerr << "GALAT ditemukan di " << ln << std::endl;
            return 0;
        }
        catch(...) {
            std::cerr << "GALAT ditemukan!! " << std::endl;
            return 0;
        }
        std::cout << "Proses penguraian joystick command config selesai" << std::endl;
        return 1;
    };

    static void update(uint32_t btn, std::vector<float> &axis) {
        button_cond = btn;
        if(axis.size() != 6) return;
        axis_val = axis;
    };

    JoyCommand(std::string str) {
        auto i = std::find(name_list.begin(), name_list.end(), str);
        if(i == name_list.end()) {
            std::cerr << "Command tidak ditemukan!" << std::endl;
            index = (uint16_t)-1;
            return;
        }
        index = std::distance(name_list.begin(), i);
    };

    bool is_button_pressed() {
        if(index == (uint16_t)-1) return false;
            return (button_cond == joyConfig_container.at(index).button_mask);
        };

        bool is_button_pressed_latch() {
        if(index == (uint16_t)-1) return false;
        bool condition = (button_cond == joyConfig_container.at(index).button_mask);
        if(condition == true && last_status == false) {
            last_status = true;
            return true;
        }
        else if(condition == false && last_status == true) {
            last_status = false;
            return false;
        }
        else return false;
    };

    float get_axis_data() {
        if(index == (uint16_t)-1) return 0;
        return axis_val.at(joyConfig_container.at(index).axis_index);
    };
};


#endif //_JOY_CMD_HPP_