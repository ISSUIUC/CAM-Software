//most of this is going to be copied from Midas buzzer
//Sound contains information to play a single note. The frequency and duration.
struct Sound {
    public:
        uint32_t frequency;
        uint8_t duration_ms;
};


//BuzzerController holds all the functions needed to play songs
struct BuzzerController {
    private:
        static constexpr uint32_t MAX_TUNE_LENGTH = 32;
        sound tune_buffer_[MAX_TUNE_LENGTH];
        uint32_t index = 0;
        uint32_t length = 0;
        uint8_t bpm = 0;

        bool new_tune_started = false;
        uint32_t when_sound_started = 0;

        void tick_sounds();
    public:
        BuzzerController() = default;;

        void tick();
        void report_beeps(bool * cont, bool fsm_fail); //beeps that report system status;
        void play_tune(const Sound* tune, uint32_t length);
        void is_playing();
        void set_bpm();
}

//define song lengths
#define RICK_ROLE_LENGTH 16

//array to hold notes for song
extern Sound rick_role[RICK_ROLE_LENGTH];