/**
 ******************************************************************************
  * File Name          : file_manager.cpp
  * Description        : File ini adalah program untuk memuat file sequence
  ******************************************************************************
  */

#ifndef __ROBOT_ITSROBOCON_FILE_MNGR__
#define __ROBOT_ITSROBOCON_FILE_MNGR__

#include <string>
#include <vector>
#include <iostream>
#include <fstream> 
#include <sstream>
#include <algorithm>

class FileManager {
public:
    typedef struct {
        int line_number;
        std::vector<std::string> data;
    } lineData_t;
    std::vector<lineData_t> file_data;

    struct includeLine{
        std::string name;
        std::string param;
    };

    struct varExt {
        std::string name;
        std::string alias;
    };

    FileManager(std::string _file, bool update=true) {
        file_path = _file;
        if(update) update_file();
    }

    void invoke(FileManager &fm) {
        file_path = fm.get_file_path();
        update_file();
    }

    /*****************************************************************************
     * @brief   Fungsi ini berguna untuk membaca file, memisahkan kata per kata
     *          dan memasukkannya ke penyimpanan utama (file_data)
     * ***************************************************************************
    */
    void update_file() {
        //membaca file dan descriptor-nya berada di fd
        std::ifstream fd(file_path);

        //membersihkan penyimpanan utama data
        file_data.clear();

        defined_param_container.clear();

        //variabel yang menghitung baris yang dibaca. dimulai dari 1
        int line_count = 1;

        //membaca file (fd) baris demi baris hingga akhir, baris dimasuukan ke line_string
        for (std::string line_string; getline(fd, line_string); line_count++) {

            //variabel tempat penampungan kata sementara
            lineData_t line_temp;

            //Cek apakah ada komentar. jika ada, dibuang
            for(auto sign = comment_sign.begin(); sign != comment_sign.end(); sign++) {
                //mencari posisi tanda komentar
                auto comment_pos = line_string.find(*sign);
                //jika terdapat tanda komentar, hanya gunakan karakter-karakter sebelumnya
                if(comment_pos != std::string::npos) {
                    line_string = line_string.substr(0, comment_pos);
                }
            }
            
            //Cek apakah barisnya kosong
            auto line_empty = line_string.empty();
            if(line_empty) continue;

            auto var_start_pos = line_string.find('%');
            if(var_start_pos != std::string::npos) {
                auto var_last_pos = line_string.find('%', var_start_pos+1);
                if(var_last_pos != std::string::npos) {
                    std::string name = line_string.substr(var_start_pos+1, var_last_pos-var_start_pos-1);
                    //std::find(external_var_container.begin(), external_var_container.last(), )
                    for(auto ct = external_var_container.begin();ct != external_var_container.end(); ct++) {
                        if(ct->name == name) {
                            line_string.replace(var_start_pos, name.size()+2, ct->alias);
                            std::cout << "Var \"" << name << "\" diubah menjadi \"" << ct->alias << "\"!" << std::endl;
                            break;
                        }
                    }
                }
            }

            auto param_pos = line_string.find('$');
            if(param_pos != std::string::npos) {
                std::string param_name;
                std::stringstream(line_string.substr(param_pos+1)) >> param_name;
                for(auto ct = defined_param_container.begin();ct != defined_param_container.end(); ct++) {
                    if(ct->name == param_name) {
                        line_string.replace(param_pos, param_name.size()+1, ct->param);
                        break;
                    }
                }
            }
            else if(line_string.at(0) == '@') {
                std::cout << "Including..." << std::endl;
                std::stringstream line_et_stream(line_string);
                std::string word_et;
                line_et_stream >> word_et;
                if(word_et == "@param") {
                    line_et_stream >> word_et;
                    std::string param_path = file_path;
                    param_path = param_path.replace(param_path.rfind('/')+1, param_path.size(), word_et);
                    std::ifstream fdparam(param_path);
                    for (std::string line_et_string; getline(fdparam, line_et_string);) {
                        
                        if(line_et_string.empty()) continue;
                        line_et_stream = std::stringstream(line_et_string);
                        auto temp = includeLine();
                        line_et_stream >> temp.name;
                        temp.param = line_et_stream.str().substr(temp.name.size()+1);
                        defined_param_container.push_back(temp);
                        // std::cout << "Namep: " << temp.name
                        //           << "String: " << temp.param
                        //           << std::endl; 
                    }
                    fdparam.close();
                }
                continue;
            }

            search_space_inside_parentheses_then_delete_it(line_string);

            //memasukkan string ke stream (agar lebih mudah dipisah menggunakan spasi)
            std::stringstream line_stream(line_string);

            //mengisi nomor baris sesuai file mentah
            line_temp.line_number = line_count;

            //mengisi kata kata di baris yang dibaca pada penampungan sementara
            for(std::string word_temp; line_stream >> word_temp;) {
                line_temp.data.push_back(word_temp);
            }

            //menyimpan ke variabel utama
            if(line_temp.data.size() > 0) file_data.push_back(line_temp);
        }
        
        std::cout << "File closed!!!" << std::endl;
        fd.close();
    }

    /*****************************************************************************
     * @brief   Fungsi ini berguna untuk menambahkan karakter atau kata yang
     *          dianggap sebagai penanda komentar
     * ***************************************************************************
    */
    void add_comment_sign(std::string sign) {
        comment_sign.push_back(sign);
    }

    /*****************************************************************************
     * @brief   Fungsi ini berguna untuk membersihkan memory
     * ***************************************************************************
    */
    void clear() {
        file_data.clear();
    }

    std::string get_file_path() {
        return file_path;
    }

    void add_var(std::string name, std::string alias) {
        auto temp = varExt();
        temp.name = name;
        temp.alias = alias;
        external_var_container.push_back(temp);
    }

protected:

    void search_space_inside_parentheses_then_delete_it(std::string& str) {
        size_t char_found_i = str.find('(');
        while(char_found_i != std::string::npos) {
            size_t char_found_f = str.find(')', char_found_i);
            if(char_found_f != std::string::npos) {
                for(;char_found_i<char_found_f; char_found_i++) {
                    switch(str.at(char_found_i)) {
                        case ' ' :
                        case '\t':
                        case '\0':
                            str.erase(char_found_i, 1);
                            char_found_i--;
                            char_found_f--;
                            // std::cout << "Halo!!" << std::endl;
                    }
                }
                // std::string sub = str.substr(char_found_i, char_found_f-char_found_i);
                // std::erase(sub, ' ');
                // std::erase(sub, '\t');
                // std::erase(sub, '\0');
                // auto it = str.begin();
                // do {
                //     it = std::find(str.begin()+char_found_i, 
                //                    str.begin()+char_found_f, 
                //                    ' ');
                //     if(it != str.begin()+char_found_f)
                //         str.erase(it);

                //     it = std::find(str.begin()+char_found_i, 
                //                    str.begin()+char_found_f, 
                //                    '\t');
                //     if(it != str.begin()+char_found_f)
                //         str.erase(it);

                // } while(it < str.begin()+char_found_f);
            }
            char_found_i = str.find('(', char_found_i+1);
        }
    }

private:
    std::string file_path;
    std::vector<std::string> comment_sign = {"//", "#"};
    std::vector<includeLine> defined_param_container;
    std::vector<varExt> external_var_container;
};

#endif /*__ROBOT_ITSROBOCON_FILE_MNGR__*/
