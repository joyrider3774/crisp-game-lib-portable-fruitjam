#ifndef I2STONES_H
#define I2STONES_H

#include <stdint.h>

// Configuration
#ifndef MAX_CHANNELS
#define MAX_CHANNELS 64
#endif

// Debug configuration
// Uncomment to enable diagnostic output every 5 seconds:
//#define I2STONES_DEBUG

// Audio output modes
enum AudioOutputMode {
    AUDIO_OUT_HEADPHONES_ONLY,   // Only output to headphone jack
    AUDIO_OUT_SPEAKER_ONLY,      // Only output to internal speaker
    AUDIO_OUT_BOTH,              // Output to both (default)
    AUDIO_OUT_AUTO_DETECT        // Auto-switch: headphones when plugged, speaker when not
};

// CRITICAL: Call updateI2SAudio() in your main loop to handle scheduled tones
// Audio generation happens automatically via timer interrupt (just like mixedtones)
// Typical usage:
//   void loop() {
//     updateI2SAudio();  // Activates scheduled tones
//     // ... rest of your code - audio plays automatically!
//   }

// Setup function
bool setupI2SAudio(uint32_t sample_rate = 44100, AudioOutputMode output_mode = AUDIO_OUT_BOTH, uint32_t buffer_size_bytes = 32768);

// Core functions
void updateI2SAudio();
int8_t playTone(float frequency, uint8_t volume, float duration_sec = 0, float delay_sec = 0);
int8_t playToneOnChannel(uint8_t channel, float frequency, uint8_t volume, float duration_sec = 0, float delay_sec = 0);
void stopChannel(int8_t channel);
void stopAllTones();
void cancelScheduled(int8_t channel);

// Query functions
uint8_t getMaxChannels();
int8_t findFreeChannel();
bool isChannelActive(int8_t channel);
uint8_t getActiveChannelCount();
uint8_t getPlayingChannelCount();
uint32_t getAudioStartTime();
uint32_t getSampleRate();
uint8_t getBitDepth();
uint32_t getTimerCallCount();  // Debug: check if timer is firing
uint32_t getBufferSkipCount();  // Debug: check buffer overruns
uint32_t getBufferUnderrunCount();  // Debug: check buffer underruns
uint32_t getRecoveryWriteCount();  // Debug: check recovery attempts
uint32_t getBufferUnderrunCount();  // Debug: check buffer underruns
uint32_t getRecoveryWriteCount();  // Debug: check recovery attempts

// Output control functions
bool isHeadphonePluggedIn();
void setAudioOutput(AudioOutputMode mode);
AudioOutputMode getAudioOutputMode();

// Diagnostic functions
void printI2SAudioDiagnostics();   // Print buffer health, skip rate, etc
void resetI2SAudioDiagnostics();   // Reset diagnostic counters
uint32_t getActualBufferSize();     // Get actual I2S buffer size
uint32_t getBufferAvailable();      // Get available buffer space
#endif
