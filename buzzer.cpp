#include "buzzer.h"
#include "pins.h"
#include <cstring>

void BuzzerController::play_song(const Sound* tune, uint32_t length_) {
    if(length > MAX_TUNE_LENGTH) length = MAX_TUNE_LENGTH;
    memcpy(tune_buffer_, tune, length * sizeof(Sound));
    index = 0;
    length = length_;
    new_tune_started = true;

    
    for(int i = 0; i < length; i++) {
        Sound& currentNote = tune_buffer_[i];
        tone(BUZZER_PIN, currentNote.frequency, currentNote.duration_ms);
    }
}
/*
    play_tune starts playing a new song

    tune = song to be played: holds array of notes
    length = length of song to be played
*/

void BuzzerController::play_tune(const Sound* tune, uint32_t length_) {
    if(length > MAX_TUNE_LENGTH) length = MAX_TUNE_LENGTH;
    memcpy(tune_buffer_, tune, length * sizeof(Sound));
    index = 0;
    length = length_;
    new_tune_started = true;
}

/*
    public interface to tick the buzzer
*/
void BuzzerController::tick() {
    tick_sounds();
}

/*
    returns whether the buzzer is playing a tune
    if length is > 0, a song has started
*/

bool BuzzerController::is_playing() {
    return (length > 0);
}

/*
    Ticks the buzzer, plays the next note / starts a new song if applicable
*/
void BuzzerController::tick_sounds() {
    if(length == 0) {
        return;
    }

    Sound& current_sound = tune_buffer_[index];

    if(new_tune_started) {
        Sound& next_sound = tune_buffer_[index];
    }
}

void BuzzerController::set_bpm(uint8_t bpm_) {
    bpm = bpm_;
}

/*
    Notes to be used for creating songs
*/

// bpm, note, note length, 1 note time, note time * length, 

//note lenths
#define beat 60000/bpm

#define whole beat * 4
#define half beat * 2
#define quarter beat
#define eigth beat / 2
#define sixteenth beat / 4

#define dotted_eigth eigth + sixteenth

#define a4_flat_sixteenth Sound{415, sixteenth}
#define b4_flat_sixteenth Sound{466, sixteenth}





// rick_role song
Sound rick_role[RICK_ROLE_LENGTH] = {}