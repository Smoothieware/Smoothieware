/*
 * FileSorter.h
 *
 *  Created on: Sep 29, 2016
 *      Author: mmoore
 */

#ifndef SRC_LIBS_FILESORTER_H_
#define SRC_LIBS_FILESORTER_H_

#include <string>
using std::string;


typedef bool (*__file_sort_filter_fn_t)(struct dirent*);

typedef struct _file_info {
    char* file_name;
    size_t file_size;
    bool is_dir;
} file_info_t;


class FileSorter {
public:
    FileSorter(string dir_path, __file_sort_filter_fn_t callback);
    virtual ~FileSorter();

    void open_directory(string dir_path);
    size_t get_file_count(void);    // the number of filtered and sorted files.
    size_t get_total_file_count();  // the raw number of files in the directory.
    const char* get_file_name(void);
    size_t get_file_size(void);
    bool get_file_is_dir(void);
    const char* next_file(void);
    const char* first_file(void);
    file_info_t* file_at(int index);
    bool has_error(void);
    void set_filter_callback(__file_sort_filter_fn_t callback);

private:
    size_t count_files_in_folder(string dir_path);
    static int compare_dir(file_info_t* file_a, file_info_t* file_b);
    void sort_directory(void);
    void delete_file_info_array(void);

    string directory_path;
    bool error;
    file_info_t* file_info_array;
    size_t file_count;
    size_t total_file_count;
    int file_index;
    __file_sort_filter_fn_t filter_callback;
};

#endif /* SRC_LIBS_FILESORTER_H_ */
