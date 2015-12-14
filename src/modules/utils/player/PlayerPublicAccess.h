#ifndef PLAYERPUBLICACCESS_H
#define PLAYERPUBLICACCESS_H

#define player_checksum           CHECKSUM("player")
#define is_playing_checksum       CHECKSUM("is_playing")
#define is_suspended_checksum     CHECKSUM("is_suspended")
#define abort_play_checksum       CHECKSUM("abort_play")
#define get_progress_checksum     CHECKSUM("progress")

struct pad_progress {
    unsigned int percent_complete;
    unsigned long elapsed_secs;
    string filename;
};
#endif
