/*
 * FileSorter.cpp
 *
 *  Created on: Sep 29, 2016
 *      Author: mmoore
 */

#include "FileSorter.h"
#include "DirHandle.h"
#include "FileStream.h"
#include "checksumm.h"
#include "PublicData.h"
#include "modules/utils/player/PlayerPublicAccess.h"
#include "SimpleShell.h"
#include "kernel.h"

#include <new>
#include <malloc.h>
#include <string.h>

#define FILE_NAME_ARRAY_MIN_PADDING   1024


FileSorter::FileSorter(string dir_path, __file_sort_filter_fn_t callback)
{
    this->directory_path = dir_path;
    this->error = false;
    this->file_info_array = NULL;
    this->file_count = 0;
    this->total_file_count = 0;
    this->file_index = -1;
    this->filter_callback = callback;

    open_directory(dir_path);
}

FileSorter::~FileSorter()
{
    delete_file_info_array();
}

int FileSorter::compare_dir(file_info_t* file_a, file_info_t* file_b)
{
    return strcmp(file_a->file_name, file_b->file_name);
}

void FileSorter::calculate_sizes(void)
{
    struct dirent *p;
    DIR* folder = opendir(this->directory_path.c_str());
    this->total_file_count = 0;
    this->bytes_required_for_names = 0;
    if ( folder != NULL ) {
        int itr = 0;
        while ( (p = readdir(folder)) != NULL ) {
            if ( (this->filter_callback == NULL) || this->filter_callback(p) ) {
                this->total_file_count++;
                this->bytes_required_for_names += (strlen(p->d_name) + 1);
            }

            // process ON_IDLE every 20 iterations.
            if ( itr++ % 20 ) {
                THEKERNEL->call_event(ON_IDLE);
            }
        }
        closedir(folder);
    }
}

void FileSorter::sort_directory(void)
{
    // sort the array.
    qsort(this->file_info_array, this->file_count, sizeof(file_info_t), (__compar_fn_t)compare_dir);
    this->file_index = -1;
}

void FileSorter::delete_file_info_array(void)
{
    if ( this->file_info_array != NULL) {
        // file names were allocated previously with strdup - free them now.
        for ( size_t i=0; i<this->file_count; i++ ) {
            free(this->file_info_array[i].file_name);
        }

        // free the array.
        delete[] this->file_info_array;
    }
}

void FileSorter::open_directory(string dir_path)
{
    // reset all member state variables.
    delete_file_info_array();
    this->directory_path = dir_path;
    this->error = false;
    this->file_count = 0;
    this->total_file_count = 0;
    this->file_index = -1;

    DIR *d;
    d = opendir(this->directory_path.c_str());
    if ( !(this->error = (d == NULL)) ) {

        // calculate the folder count and required array size.
        calculate_sizes();

        // disallow sorting when the machine is active (printing, milling, etc),
        // or when there's not enough heap memory available for the sort array.
        uint32_t free_bytes = SimpleShell::heapWalk(NULL, false);
        this->error = (!FileSorter::can_do_sort() || free_bytes < (this->bytes_required_for_names + FILE_NAME_ARRAY_MIN_PADDING));
        if ( !this->error ) {

            // allocate the array used for sorting.
            this->file_info_array = new (std::nothrow) file_info_t[this->total_file_count];

            // read the file info into the array.
            if ( !(this->error = (this->file_info_array == NULL)) ) {
                int itr = 0;
                struct dirent* file_info;
                file_info_t* array_entry;
                this->file_count = 0;
                array_entry = this->file_info_array;
                while ( this->file_count < this->total_file_count && (file_info = readdir(d)) != NULL ) {

                    // if the filename passes the filter, add it to the sort array.
                    if ( (this->filter_callback == NULL) || this->filter_callback(file_info) ) {
                        array_entry->is_dir = file_info->d_isdir;
                        array_entry->file_size = file_info->d_fsize;
                        array_entry->file_name = strdup(file_info->d_name);

                        // ensure that the filename was copied successfully.
                        if ( (this->error = (array_entry->file_name == NULL)) ) {
                            // failed to copy the filename, clean up and get out.
                            delete_file_info_array();
                            break;
                        }

                        this->file_count++;
                        array_entry++;
                    }

                    // process ON_IDLE every 20 iterations.
                    if ( itr++ % 20 ) {
                        THEKERNEL->call_event(ON_IDLE);
                    }
                }
            }
        }
        // close the directory handle and sort the file array.
        closedir(d);
        d = NULL;
        if ( !this->error ) {
            sort_directory();
        }
    }
}

size_t FileSorter::get_file_count(void)
{
    return this->file_count;
}

size_t FileSorter::get_total_file_count(void)
{
    return this->total_file_count;
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

bool FileSorter::can_do_sort(void)
{
    void *returned_data;
    bool ok = PublicData::get_value( player_checksum, is_playing_checksum, &returned_data );
    if (ok) {
        bool b = *static_cast<bool *>(returned_data);
        return !b;
    }
    return true;
}
