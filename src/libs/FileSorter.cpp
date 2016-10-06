/*
 * FileSorter.cpp
 *
 *  Created on: Sep 29, 2016
 *      Author: mmoore
 */

#include "FileSorter.h"
#include "DirHandle.h"
#include "FileStream.h"

#include <new>


FileSorter::FileSorter(string dir_path, __file_sort_filter_fn_t callback)
{
    this->directory_path = dir_path;
    this->error = false;
    this->file_info_array = NULL;
    this->file_count = 0;
    this->file_index = -1;
    this->filter_callback = callback;

    open_directory(dir_path);
}

FileSorter::~FileSorter()
{
    if ( this->file_info_array ) {
        delete[] this->file_info_array;
    }
}

int FileSorter::compare_dir(file_info_t* file_a, file_info_t* file_b)
{
    return strcmp(file_a->file_name, file_b->file_name);
}

size_t FileSorter::count_files_in_folder(string dir_path)
{
    struct dirent *p;
    uint16_t count = 0;
    DIR* folder = opendir(dir_path.c_str());
    if ( folder != NULL ) {
        while ( (p = readdir(folder)) != NULL ) {
            if ( (this->filter_callback == NULL) || this->filter_callback(p) ) {
                count++;
            }
        }
        closedir(folder);
        return count;
    }
    return 0;
}

void FileSorter::sort_directory(void)
{
    // sort the array.
    qsort(this->file_info_array, this->file_count, sizeof(file_info_t), (__compar_fn_t)compare_dir);
    this->file_index = -1;
}

void FileSorter::open_directory(string dir_path)
{
    // reset all member state variables.
    this->directory_path = dir_path;
    this->error = false;
    this->file_count = 0;
    this->file_index = -1;
    if ( this->file_info_array ) {
        delete[] this->file_info_array;
        this->file_info_array = NULL;
    }

    DIR *d;
    d = opendir(this->directory_path.c_str());
    if ( !(this->error = (d == NULL)) ) {
        // get a count of items in the folder.
        size_t files_in_folder = count_files_in_folder(this->directory_path);

        // allocate the array used for sorting.
        this->file_info_array = new (std::nothrow) file_info_t[files_in_folder];

        // read the file info into the array.
        if ( !(this->error = (this->file_info_array == NULL)) ) {
            struct dirent* file_info;
            file_info_t* array_entry;
            this->file_count = 0;
            array_entry = this->file_info_array;
            while ( this->file_count < files_in_folder && (file_info = readdir(d)) != NULL ) {
                if ( (this->filter_callback == NULL) || this->filter_callback(file_info) ) {
                    array_entry->is_dir = file_info->d_isdir;
                    array_entry->file_size = file_info->d_fsize;
                    std::strncpy(array_entry->file_name, file_info->d_name, FILE_SORTER_MAX_NAME_SIZE);
                    this->file_count++;
                    array_entry++;
                }
            }
        }

        // close the directory handle and sort the file array.
        closedir(d);
        d = NULL;
        sort_directory();
    }
}

int FileSorter::get_file_count(void)
{
    return this->file_count;
}

const char* FileSorter::get_file_name(void)
{
    if ( this->file_index >= 0 && (size_t) this->file_index < this->file_count ) {
        return this->file_info_array[this->file_index].file_name;
    } else {
        return NULL;
    }
}

size_t FileSorter::get_file_size(void)
{
    if ( this->file_index >= 0 && (size_t) this->file_index < this->file_count ) {
        return this->file_info_array[this->file_index].file_size;
    } else {
        return 0;
    }
}

bool FileSorter::get_file_is_dir(void)
{
    if ( this->file_index >= 0 && (size_t) this->file_index < this->file_count ) {
        return this->file_info_array[this->file_index].is_dir;
    } else {
        return false;
    }
}

const char* FileSorter::next_file(void)
{
    if ( this->file_index == -1 || (this->file_index >= 0 && (size_t) this->file_index < (this->file_count - 1)) ) {
        return this->file_info_array[++this->file_index].file_name;
    } else {
        return NULL;
    }
}

const char* FileSorter::first_file(void)
{
    if ( this->file_count > 0 ) {
        this->file_index = 0;
        return this->file_info_array[this->file_index].file_name;
    } else {
        return NULL;
    }
}

file_info_t* FileSorter::file_at(int index)
{
    if ( index >= 0 && (size_t) index < this->file_count ) {
        return &this->file_info_array[index];
    } else {
        return NULL;
    }
}

bool FileSorter::has_error(void)
{
    return this->error;
}

void FileSorter::set_filter_callback(__file_sort_filter_fn_t callback)
{
    this->filter_callback = callback;
}
