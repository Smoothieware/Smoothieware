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

class StreamOutput;

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
    void print_file_list(StreamOutput* stream, bool verbose);
    size_t get_file_count(void);    // the number of filtered and sorted files.
    size_t get_total_file_count();  // the raw number of files in the directory.
    const char* file_at(int index, bool& isdir);
    bool has_error(void);
    bool is_sorted(void);
    void set_filter_callback(__file_sort_filter_fn_t callback);
    static bool can_do_sort(void);

private:
    void calculate_sizes(void);
    static int compare_dir(file_info_t* file_a, file_info_t* file_b);
    void sort_directory(void);
    void delete_file_info_array(void);

    string directory_path;
    file_info_t* file_info_array;
    size_t file_count;
    size_t total_file_count;
    size_t bytes_required_for_names;
    __file_sort_filter_fn_t filter_callback;

    struct {
      bool error:1;
      bool sorted:1;
    };
};

#endif /* SRC_LIBS_FILESORTER_H_ */
