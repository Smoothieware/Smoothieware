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
#include "PlayerPublicAccess.h"
#include "SimpleShell.h"
#include "Kernel.h"
#include "StreamOutput.h"

#include <new>
#include <malloc.h>
#include <string.h>

#define FILE_NAME_ARRAY_MIN_PADDING   1024


FileSorter::FileSorter(string dir_path, __file_sort_filter_fn_t callback)
{
    this->filter_callback = callback;
    this->file_info_array = NULL;

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
        this->file_info_array = NULL;
    }
}

void FileSorter::open_directory(string dir_path)
{
    // reset all member state variables.
    delete_file_info_array();
    //
    this->directory_path = dir_path;
    this->error = false;
    this->sorted = false;
    this->file_count = 0;
    this->total_file_count = 0;

    DIR *d;
    d = opendir(this->directory_path.c_str());
    if ( !(this->error = (d == NULL)) ) {

        // calculate the folder count and required array size.
        calculate_sizes();

        // disallow sorting when the machine is active (printing, milling, etc),
        // or when there's not enough heap memory available for the sort array.
        uint32_t free_bytes = SimpleShell::heapWalk(NULL, false);
        uint32_t bytes_required = (this->bytes_required_for_names + (this->total_file_count * sizeof(file_info_t)) + FILE_NAME_ARRAY_MIN_PADDING);
        this->sorted = (FileSorter::can_do_sort() && free_bytes > bytes_required);
        if ( this->sorted ) {

            // allocate the array used for sorting.
            this->file_info_array = new (std::nothrow) file_info_t[this->total_file_count];

            // read the file info into the array.
            if ( (this->sorted = (this->file_info_array != NULL)) ) {
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
                        if ( !(this->sorted = (array_entry->file_name != NULL)) ) {
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
        if ( this->sorted ) {
            sort_directory();
        }
    }
}

void FileSorter::print_file_list(StreamOutput* stream, bool verbose)
{
    // attempt to use the file sorter to sort alphabetically.
    if ( this->sorted ) {
        file_info_t* file_info;

        for ( size_t i=0; i<this->file_count; i++ ) {
            if ( (file_info = &this->file_info_array[i] ) != NULL ) {
                //stream->printf("%s", lc(string(files->get_file_name())).c_str());
                stream->printf("%s", file_info->file_name);
                if ( file_info->is_dir ) {
                    stream->printf("/");
                } else if ( verbose ) {
                    stream->printf(" %d", file_info->file_size);
                }
                stream->printf("\r\n");
            }
        }

    // if the file sorter fails (likely due to memory constraints),
    // just list the files the old fashioned way.
    } else {
        DIR *d;
        struct dirent *p;
        d = opendir(this->directory_path.c_str());
        if (d != NULL) {
            int itr = 0;
            while ((p = readdir(d)) != NULL) {
                stream->printf("%s", p->d_name);
                if(p->d_isdir) {
                    stream->printf("/");
                } else if( verbose ) {
                    stream->printf(" %d", p->d_fsize);
                }
                stream->printf("\r\n");

                // process ON_IDLE every 20 iterations.
                if ( itr++ % 20 ) {
                    THEKERNEL->call_event(ON_IDLE);
                }
           }
            closedir(d);
        } else {
            stream->printf("Could not open directory %s\r\n", this->directory_path.c_str());
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

const char* FileSorter::file_at(int index, bool& isdir)
{
    // attempt to sort the files alphabetically.
    if ( this->sorted ) {
        if ( index >= 0 && (size_t) index < this->file_count ) {
            isdir = this->file_info_array[index].is_dir;
            return this->file_info_array[index].file_name;
        } else {
            return NULL;
        }

    // if the file sorter failed (likely due to memory constraints), do
    // the filename lookup the old fashioned way.
    } else {
        DIR *d;
        struct dirent *p;
        uint16_t count = 0;
        d = opendir(THEKERNEL->current_path.c_str());
        if (d != NULL) {
            int itr = 0;
            while ((p = readdir(d)) != NULL) {
                if ( ((this->filter_callback == NULL) || this->filter_callback(p)) && index == count++ ) {
                    closedir(d);
                    isdir = p->d_isdir;
                    return p->d_name;
                }

                // process ON_IDLE every 20 iterations.
                if ( itr++ % 20 ) {
                    THEKERNEL->call_event(ON_IDLE);
                }
            }

            closedir(d);
        }

        return NULL;
    }
}

bool FileSorter::has_error(void)
{
    return this->error;
}

bool FileSorter::is_sorted(void)
{
    return this->sorted;
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
