// i2stones.cpp - I2S audio library with multi-channel tone mixing
// Based on mixedtones architecture with I2S output
// Uses RP2350 alarm pool for timer interrupts

#include "i2stones.h"
#include <Adafruit_TLV320DAC3100.h>
#include <I2S.h>
#include <hardware/timer.h>
#include <pico/time.h>

// GLOBAL scope - EXACTLY like working minimal test
Adafruit_TLV320DAC3100 codec;
I2S i2s(OUTPUT);

// Timer variables for RP2350
static struct repeating_timer audio_timer;
static volatile bool timer_running = false;
static volatile uint32_t timer_call_count = 0;  // For debugging
static volatile uint32_t buffer_full_skips = 0;  // Track buffer overflow events
static volatile uint32_t buffer_empty_count = 0;  // Track underrun events
static volatile uint32_t recovery_writes = 0;    // Track recovery burst writes

// Pin definitions
#define PIN_RESET 22
#define PIN_BCLK 26  
#define PIN_WS 27
#define PIN_DATA 24

// Audio state
static bool audio_ready = false;
static uint32_t audio_start_time = 0;
static uint32_t sample_rate = 44100;
static uint8_t bit_depth = 16;
static uint32_t actual_buffer_size = 32768;  // Actual I2S buffer size
static AudioOutputMode current_output_mode = AUDIO_OUT_BOTH;
static bool last_headphone_state = false;

// Multi-channel oscillator array - EXACTLY like mixedtones
struct Oscillator {
    uint32_t phase;
    uint32_t phase_increment;
    uint16_t amplitude;
    uint32_t duration_samples;
    uint32_t samples_played;
    uint32_t start_time_ms;
    uint16_t envelope;  // Fade in/out: 0-256 (prevents clicks)
    bool active;
    bool scheduled;
};

static volatile Oscillator oscillators[MAX_CHANNELS];  // Volatile for interrupt access

// Track which channels are active to speed up mixing
static volatile uint8_t active_channel_indices[MAX_CHANNELS];
static volatile uint8_t active_channel_count = 0;

// DC blocker to remove clicks (high-pass filter at ~10 Hz)
static int32_t dc_filter_x1 = 0;
static int32_t dc_filter_y1 = 0;

// High-quality sine wave lookup table - EXACTLY like mixedtones
inline int16_t fastSine(uint8_t phase) {
    static const int8_t sine_lut[64] = {
        0, 6, 12, 18, 25, 31, 37, 43, 49, 54, 60, 65, 71, 76, 81, 85,
        90, 94, 98, 102, 106, 109, 112, 115, 117, 120, 122, 123, 125, 126, 126, 127,
        127, 127, 126, 126, 125, 123, 122, 120, 117, 115, 112, 109, 106, 102, 98, 94,
        90, 85, 81, 76, 71, 65, 60, 54, 49, 43, 37, 31, 25, 18, 12, 6
    };
    
    uint8_t quad = phase >> 6;
    uint8_t index = phase & 0x3F;
    
    int8_t val = sine_lut[index];
    
    if (quad == 1) val = sine_lut[63 - index];
    else if (quad == 2) val = -sine_lut[index];
    else if (quad == 3) val = -sine_lut[63 - index];
    
    return val;
}

// RP2350 timer callback - returns true to keep repeating
// Fires at 2kHz with adaptive sample count to match I2S exactly
bool timerCallback_I2S(struct repeating_timer *t) {
    timer_call_count++;
    
    int available = i2s.availableForWrite();
    
    // CORRECTED ACCUMULATOR MATH:
    // Timer fires 2000 times per second
    // Need to generate sample_rate samples per second
    // So per tick: sample_rate / 2000
    // 
    // For 44100 Hz: 44100 / 2000 = 22.05 samples/tick
    // For 22050 Hz: 22050 / 2000 = 11.025 samples/tick
    // For 11025 Hz: 11025 / 2000 = 5.5125 samples/tick
    //
    // Use fixed-point math: multiply by 1000 to keep precision
    
    static uint32_t accumulator = 0;
    const uint32_t samples_per_tick_x1000 = (sample_rate * 1000) / 2000;
    
    accumulator += samples_per_tick_x1000;
    int samples_to_write = accumulator / 1000;
    accumulator = accumulator % 1000;
    
    // ADAPTIVE: Adjust based on buffer level to prevent drift
    // Target: keep buffer around 30-70% full
    uint32_t threshold_full = actual_buffer_size * 30 / 100;   // 30% full
    uint32_t threshold_empty = actual_buffer_size * 70 / 100;  // 70% empty
    
    if (available < threshold_full) {
        // Buffer >70% full - write fewer samples
        if (samples_to_write > 0) samples_to_write--;
    } else if (available > threshold_empty) {
        // Buffer >70% empty - write more samples  
        samples_to_write++;
    }
    
    // EMERGENCY RECOVERY: Buffer critically low (>85% empty)
    if (available > (actual_buffer_size * 85 / 100)) {
        buffer_empty_count++;
        // Write extra burst
        for (int burst = 0; burst < 40; burst++) {
            if (i2s.availableForWrite() < 4) break;
            recovery_writes++;
            
            // Generate and write a recovery sample
            int32_t mixed_sample = 0;
            int active_count = 0;
            
            for (int idx = 0; idx < active_channel_count; idx++) {
                int i = active_channel_indices[idx];
                if (oscillators[i].active && oscillators[i].amplitude > 0) {
                    uint8_t phase_byte = oscillators[i].phase >> 24;
                    int16_t sample = fastSine(phase_byte);
                    int32_t amplitude_sample = (sample * oscillators[i].amplitude) >> 8;
                    
                    if (oscillators[i].envelope < 256) {
                        oscillators[i].envelope += 4;
                        if (oscillators[i].envelope > 256) oscillators[i].envelope = 256;
                    }
                    if (oscillators[i].duration_samples > 0) {
                        uint32_t remaining = oscillators[i].duration_samples - oscillators[i].samples_played;
                        if (remaining <= 64) oscillators[i].envelope = remaining * 4;
                    }
                    
                    amplitude_sample = (amplitude_sample * oscillators[i].envelope) >> 8;
                    mixed_sample += amplitude_sample;
                    oscillators[i].phase += oscillators[i].phase_increment;
                    oscillators[i].samples_played++;
                    
                    if (oscillators[i].duration_samples > 0 && 
                        oscillators[i].samples_played >= oscillators[i].duration_samples) {
                        oscillators[i].active = false;
                        oscillators[i].scheduled = false;
                        oscillators[i].envelope = 0;
                    }
                    active_count++;
                }
            }
            
            if (active_count > 0) mixed_sample /= active_count;
            mixed_sample *= 2;
            mixed_sample *= 96;
            
            static uint32_t dither_state = 0x12345678;
            dither_state ^= dither_state << 13;
            dither_state ^= dither_state >> 17;
            dither_state ^= dither_state << 5;
            int16_t dither = (int16_t)(dither_state & 0x03) - 2;
            mixed_sample += dither;
            
            if (mixed_sample > 32767) mixed_sample = 32767;
            if (mixed_sample < -32768) mixed_sample = -32768;
            
            int16_t output = (int16_t)mixed_sample;
            i2s.write(output);
            i2s.write(output);
        }
    }
    
    // Write adaptive number of samples per interrupt
    for (int samplenum = 0; samplenum < samples_to_write; samplenum++) {
        
        // Check buffer space before writing
        if (i2s.availableForWrite() < 4) {
            buffer_full_skips++;
            continue;  // Skip this sample if buffer full
        }
        
        int32_t mixed_sample = 0;
        int active_count = 0;
        
        // Mix all active channels
        for (int idx = 0; idx < active_channel_count; idx++) {
            int i = active_channel_indices[idx];
            
            if (oscillators[i].active) {
                // Skip if volume is zero
                if (oscillators[i].amplitude == 0) {
                    oscillators[i].phase += oscillators[i].phase_increment;
                    oscillators[i].samples_played++;
                    
                    if (oscillators[i].duration_samples > 0 && 
                        oscillators[i].samples_played >= oscillators[i].duration_samples) {
                        oscillators[i].active = false;
                        oscillators[i].scheduled = false;
                    }
                    continue;
                }

                // Generate sine wave sample
                uint8_t phase_byte = oscillators[i].phase >> 24;
                int16_t sample = fastSine(phase_byte);
                int32_t amplitude_sample = (sample * oscillators[i].amplitude) >> 8;
                
                // Apply envelope (fade in/out)
                if (oscillators[i].envelope < 256) {
                    oscillators[i].envelope += 4;
                    if (oscillators[i].envelope > 256) oscillators[i].envelope = 256;
                }
                
                if (oscillators[i].duration_samples > 0) {
                    uint32_t remaining = oscillators[i].duration_samples - oscillators[i].samples_played;
                    if (remaining <= 64) {
                        oscillators[i].envelope = remaining * 4;
                    }
                }
                
                amplitude_sample = (amplitude_sample * oscillators[i].envelope) >> 8;
                mixed_sample += amplitude_sample;
                
                // Advance oscillator
                oscillators[i].phase += oscillators[i].phase_increment;
                oscillators[i].samples_played++;
                
                if (oscillators[i].duration_samples > 0 && 
                    oscillators[i].samples_played >= oscillators[i].duration_samples) {
                    oscillators[i].active = false;
                    oscillators[i].scheduled = false;
                    oscillators[i].envelope = 0;
                }
                
                active_count++;
            }
        }
        
        // Average if multiple channels active
        if (active_count > 0) {
            mixed_sample /= active_count;
        }
        
        // Scale to 16-bit range (75% to prevent distortion)
        mixed_sample *= 2;
        mixed_sample *= 96;
        
        // Dithering
        static uint32_t dither_state = 0x12345678;
        dither_state ^= dither_state << 13;
        dither_state ^= dither_state >> 17;
        dither_state ^= dither_state << 5;
        int16_t dither = (int16_t)(dither_state & 0x03) - 2;
        mixed_sample += dither;
        
        // Clamp
        if (mixed_sample > 32767) mixed_sample = 32767;
        if (mixed_sample < -32768) mixed_sample = -32768;
        
        int16_t output = (int16_t)mixed_sample;
        
        // Write stereo sample
        i2s.write(output);
        i2s.write(output);
    }
    
    return true;
}

bool setupI2SAudio(uint32_t sample_rate_arg, AudioOutputMode output_mode, uint32_t buffer_size_bytes) {
    sample_rate = sample_rate_arg;
    current_output_mode = output_mode;
    
    Serial.println("\n=== I2STONES SETUP ===");
    
    // Step 1: Hardware reset (CRITICAL!)
    Serial.println("1. Hardware reset...");
    pinMode(PIN_RESET, OUTPUT);
    digitalWrite(PIN_RESET, LOW);
    delay(100);
    digitalWrite(PIN_RESET, HIGH);
    delay(50);
    
    // Step 2: Initialize I2C
    Serial.println("2. Initialize codec...");
    if (!codec.begin()) {
        Serial.println("ERROR: codec.begin() failed!");
        return false;
    }
    delay(10);
    
    // Step 3: Software reset
    Serial.println("3. Software reset...");
    codec.reset();
    
    // Step 4: Configure codec
    Serial.println("4. Configuring codec...");
    
    // Configure interface
    if (!codec.setCodecInterface(TLV320DAC3100_FORMAT_I2S, TLV320DAC3100_DATA_LEN_16)) {
        Serial.println("ERROR: setCodecInterface failed!");
        return false;
    }
    
    // Configure clocks
    if (!codec.setCodecClockInput(TLV320DAC3100_CODEC_CLKIN_PLL) ||
        !codec.setPLLClockInput(TLV320DAC3100_PLL_CLKIN_BCLK)) {
        Serial.println("ERROR: clock config failed!");
        return false;
    }
    
    // PLL values  
    if (!codec.setPLLValues(1, 1, 8, 0)) {
        Serial.println("ERROR: setPLLValues failed!");
        return false;
    }
    
    // DAC dividers
    if (!codec.setNDAC(true, 8) ||
        !codec.setMDAC(true, 2) ||
        !codec.setDOSR(128)) {
        Serial.println("ERROR: DAC dividers failed!");
        return false;
    }
    
    // Power up PLL
    if (!codec.powerPLL(true)) {
        Serial.println("ERROR: powerPLL failed!");
        return false;
    }
    
    // DAC data path
    if (!codec.setDACDataPath(true, true, 
                               TLV320_DAC_PATH_NORMAL,
                               TLV320_DAC_PATH_NORMAL,
                               TLV320_VOLUME_STEP_1SAMPLE)) {
        Serial.println("ERROR: setDACDataPath failed!");
        return false;
    }
    
    // Route to mixer
    if (!codec.configureAnalogInputs(TLV320_DAC_ROUTE_MIXER,
                                     TLV320_DAC_ROUTE_MIXER,
                                     false, false, false, false)) {
        Serial.println("ERROR: configureAnalogInputs failed!");
        return false;
    }
    
    // DAC volume
    if (!codec.setDACVolumeControl(false, false, TLV320_VOL_INDEPENDENT) ||
        !codec.setChannelVolume(false, 0) ||
        !codec.setChannelVolume(true, 0)) {
        Serial.println("ERROR: DAC volume failed!");
        return false;
    }
    
    // ALWAYS configure both outputs (hardware needs full init)
    // Headphone drivers
    if (!codec.configureHeadphoneDriver(true, true,
                                        TLV320_HP_COMMON_1_35V,
                                        false) ||
        !codec.configureHPL_PGA(0, true) ||
        !codec.configureHPR_PGA(0, true) ||
        !codec.setHPLVolume(true, 6) ||
        !codec.setHPRVolume(true, 6)) {
        Serial.println("ERROR: Headphone config failed!");
        return false;
    }
    
    // Speaker
    if (!codec.enableSpeaker(true) ||
        !codec.configureSPK_PGA(TLV320_SPK_GAIN_6DB, true) ||
        !codec.setSPKVolume(true, 0)) {
        Serial.println("ERROR: Speaker config failed!");
        return false;
    }
    
    // Now MUTE unwanted outputs using PGA unmute parameter
    switch(output_mode) {
        case AUDIO_OUT_HEADPHONES_ONLY:
            codec.configureSPK_PGA(TLV320_SPK_GAIN_6DB, false);  // Mute speaker
            Serial.println("   ✓ Headphone output only");
            break;
            
        case AUDIO_OUT_SPEAKER_ONLY:
            codec.configureHPL_PGA(0, false);  // Mute headphones
            codec.configureHPR_PGA(0, false);
            Serial.println("   ✓ Speaker output only");
            break;
            
        case AUDIO_OUT_BOTH:
            // Already configured above
            Serial.println("   ✓ Both outputs enabled");
            break;
            
        case AUDIO_OUT_AUTO_DETECT:
            // Start with speaker only
            codec.configureHPL_PGA(0, false);  // Mute headphones
            codec.configureHPR_PGA(0, false);
            Serial.println("   ✓ Auto-detect mode (speaker default)");
            break;
    }
    
    Serial.println("5. Codec configured successfully!");
    
    // Step 5: Setup I2S
    Serial.println("6. Setting up I2S...");
    i2s.setBCLK(PIN_BCLK);
    i2s.setDATA(PIN_DATA);
    i2s.setBitsPerSample(16);
    
    // Calculate buffer configuration from desired total size
    // Strategy: Use 512-byte chunks (good balance between latency and efficiency)
    uint32_t buffer_count = buffer_size_bytes / 512;
    if (buffer_count < 4) buffer_count = 4;    // Minimum 4 buffers (2KB)
    if (buffer_count > 64) buffer_count = 64;  // Maximum 64 buffers (32KB)
    
    uint32_t actual_buffer_size = buffer_count * 512;
    
    Serial.print("   Requested buffer: ");
    Serial.print(buffer_size_bytes);
    Serial.println(" bytes");
    Serial.print("   Configuring: ");
    Serial.print(buffer_count);
    Serial.print(" buffers × 512 bytes = ");
    Serial.print(actual_buffer_size);
    Serial.println(" bytes");
    
    i2s.setBuffers(buffer_count, 512);
    
    Serial.println("   About to call i2s.begin()...");
    
    if (!i2s.begin(sample_rate)) {
        Serial.println("ERROR: i2s.begin() failed!");
        return false;
    }
    
    actual_buffer_size = i2s.availableForWrite();
    
    Serial.print("   I2S buffer size: ");
    Serial.print(actual_buffer_size);
    Serial.println(" bytes");
    
    double latency_ms = (actual_buffer_size / 4.0) / sample_rate * 1000.0;
    Serial.print("   Buffer latency: ");
    Serial.print(latency_ms, 1);
    Serial.println(" ms");
    
    Serial.println("=== SETUP COMPLETE ===");
    
    // Initialize oscillators - EXACTLY like mixedtones
    for (int i = 0; i < MAX_CHANNELS; i++) {
        oscillators[i].active = false;
        oscillators[i].scheduled = false;
        oscillators[i].phase = 0;
        oscillators[i].amplitude = 0;
        oscillators[i].duration_samples = 0;
        oscillators[i].samples_played = 0;
        oscillators[i].start_time_ms = 0;
    }
    
    // PRE-FILL I2S BUFFER: Write silence to create buffer headroom
    // This gives us breathing room before timer starts generating samples
    Serial.println("Pre-filling I2S buffer with silence...");
    int16_t silence_sample = 0;
    int samples_written = 0;

    // Fill buffer to 75% capacity (leave 25% headroom for timer startup)
    uint32_t max_samples = (actual_buffer_size * 3) / (4 * 4);  // *3/4 for 75%, /4 for stereo 16-bit

    while (i2s.availableForWrite() >= 4 && samples_written < max_samples) {
        i2s.write(silence_sample);  // Left
        i2s.write(silence_sample);  // Right
        samples_written++;
    }
    Serial.print("Pre-filled ");
    Serial.print(samples_written);
    Serial.print(" samples (");
    Serial.print((samples_written * 4 * 100) / actual_buffer_size);
    Serial.println("% of buffer)");
    
    // Setup RP2350 repeating timer - fires at 2kHz (every 500μs)
    Serial.println("7. Setting up audio generation timer...");
    Serial.println("   Timer: 2000 Hz (every 500 μs)");
    Serial.println("   Generates ~22 samples per tick = 44,000 Hz effective");
    
    int64_t interval_us = -500;  // 2kHz timer (every 500 microseconds)
    
    if (!add_repeating_timer_us(interval_us, timerCallback_I2S, NULL, &audio_timer)) {
        Serial.println("ERROR: Failed to setup timer!");
        Serial.println("Falling back to polling mode - call updateI2SAudio() frequently!");
        timer_running = false;
    } else {
        timer_running = true;
        Serial.println("8. Timer interrupt enabled successfully!");
    }
    
    audio_start_time = millis();
    audio_ready = true;
    
    return true;
}

// Helper function to rebuild active channel list
// LOCK-FREE: Interrupt reads count first, then indices. We write indices first, then count.
// This guarantees interrupt always sees a consistent (possibly slightly stale) view.
static inline void rebuildActiveChannelList() {
    uint8_t temp_indices[MAX_CHANNELS];
    uint8_t temp_count = 0;
    
    // Scan all channels (on main thread, no interrupt blocking needed)
    for (int i = 0; i < MAX_CHANNELS; i++) {
        if (oscillators[i].active || oscillators[i].scheduled) {
            temp_indices[temp_count++] = i;
        }
    }
    
    // Update shared data - write indices FIRST, then count
    // Interrupt reads count first, so it never sees partially-updated indices
    for (int i = 0; i < temp_count; i++) {
        active_channel_indices[i] = temp_indices[i];  // Safe: interrupt uses old count
    }
    active_channel_count = temp_count;  // Atomic on RP2350 - now visible to interrupt
}

void updateI2SAudio() {
    if (!audio_ready) return;
    
    uint32_t now = millis();
    
#ifdef I2STONES_DEBUG
    // Print diagnostics every 5 seconds (only if I2STONES_DEBUG is defined)
    static uint32_t last_diagnostic_time = 0;
    if (now - last_diagnostic_time >= 5000) {
        printI2SAudioDiagnostics();
        last_diagnostic_time = now;
    }
#endif
    
    // Handle auto-detection mode
    if (current_output_mode == AUDIO_OUT_AUTO_DETECT) {
        bool headphone_plugged = isHeadphonePluggedIn();
        
        // Only update if state changed
        if (headphone_plugged != last_headphone_state) {
            last_headphone_state = headphone_plugged;
            
            if (headphone_plugged) {
                // Headphones plugged - unmute headphones, mute speaker
                codec.configureHPL_PGA(0, true);
                codec.configureHPR_PGA(0, true);
                codec.configureSPK_PGA(TLV320_SPK_GAIN_6DB, false);
            } else {
                // Headphones unplugged - mute headphones, unmute speaker
                codec.configureHPL_PGA(0, false);
                codec.configureHPR_PGA(0, false);
                codec.configureSPK_PGA(TLV320_SPK_GAIN_6DB, true);
            }
        }
    }
    
    // Update scheduled tones - check if it's time to start them
    for (int i = 0; i < MAX_CHANNELS; i++) {
        if (oscillators[i].scheduled && !oscillators[i].active) {
            if (now >= oscillators[i].start_time_ms) {
                oscillators[i].phase = 0;
                oscillators[i].samples_played = 0;
                oscillators[i].envelope = 0;  // Start with fade-in
                oscillators[i].active = true;
                oscillators[i].scheduled = false;
            }
        }
    }
    
    // OPTIMIZATION: Rebuild list of active channels to speed up timer interrupt
    // This way the interrupt only loops through 3-4 channels instead of all 64
    rebuildActiveChannelList();
    
    // If timer isn't running, manually generate samples (fallback mode)
    if (!timer_running) {
        while (i2s.availableForWrite() >= 4) {
            int32_t mixed_sample = 0;
            int active_count = 0;
            
            // Use active channel list in fallback mode too
            for (int idx = 0; idx < active_channel_count; idx++) {
                int i = active_channel_indices[idx];
                
                if (oscillators[i].active) {
                    if (oscillators[i].amplitude == 0) {
                        oscillators[i].phase += oscillators[i].phase_increment;
                        oscillators[i].samples_played++;
                        
                        if (oscillators[i].duration_samples > 0 && 
                            oscillators[i].samples_played >= oscillators[i].duration_samples) {
                            oscillators[i].active = false;
                            oscillators[i].scheduled = false;
                        }
                        continue;
                    }

                    uint8_t phase_byte = oscillators[i].phase >> 24;
                    int16_t sample = fastSine(phase_byte);
                    mixed_sample += (sample * oscillators[i].amplitude) >> 8;
                    
                    oscillators[i].phase += oscillators[i].phase_increment;
                    oscillators[i].samples_played++;
                    
                    if (oscillators[i].duration_samples > 0 && 
                        oscillators[i].samples_played >= oscillators[i].duration_samples) {
                        oscillators[i].active = false;
                        oscillators[i].scheduled = false;
                    }
                    
                    active_count++;
                }
            }
            
            // ALWAYS average if there are active channels
            if (active_count > 0) {
                mixed_sample /= active_count;
            }
            
            mixed_sample *= 2;
            mixed_sample *= 128;
            
            if (mixed_sample > 32767) mixed_sample = 32767;
            if (mixed_sample < -32768) mixed_sample = -32768;
            
            int16_t output = (int16_t)mixed_sample;
            
            i2s.write(output);
            i2s.write(output);
        }
    }
}

uint8_t getMaxChannels() {
    return MAX_CHANNELS;
}

int8_t findFreeChannel() {
    for (int i = 0; i < MAX_CHANNELS; i++) {
        if (!oscillators[i].active && !oscillators[i].scheduled) {
            return i;
        }
    }
    return -1;
}

int8_t playTone(float frequency, uint8_t volume, float duration_sec, float delay_sec) {
    int8_t channel = findFreeChannel();
    if (channel < 0) return -1;
    
    oscillators[channel].phase_increment = (uint32_t)((frequency * 4294967296.0) / sample_rate);
    oscillators[channel].amplitude = volume;
    
    if (duration_sec > 0) {
        oscillators[channel].duration_samples = (uint32_t)(sample_rate * duration_sec);
    } else {
        oscillators[channel].duration_samples = 0;
    }
    
    if (delay_sec > 0) {
        oscillators[channel].start_time_ms = millis() + (uint32_t)(delay_sec * 1000);
        oscillators[channel].scheduled = true;
        oscillators[channel].active = false;
        oscillators[channel].phase = 0;
        oscillators[channel].samples_played = 0;
    } else {
        oscillators[channel].start_time_ms = 0;
        oscillators[channel].scheduled = false;
        oscillators[channel].active = true;
        oscillators[channel].phase = 0;
        oscillators[channel].samples_played = 0;
    }
    
    oscillators[channel].envelope = 0;  // Start with fade-in
    
    // Rebuild active list
    rebuildActiveChannelList();
    
    return channel;
}

int8_t playToneOnChannel(uint8_t channel, float frequency, uint8_t volume, float duration_sec, float delay_sec) {
    if (channel >= MAX_CHANNELS) return -1;
    
    // Compensate for actual timer rate: 43478 Hz instead of 44100 Hz
    float compensated_freq = frequency * 1.014302f;
    
    oscillators[channel].phase_increment = (uint32_t)((compensated_freq * 4294967296.0) / sample_rate);
    oscillators[channel].amplitude = volume;
    
    if (duration_sec > 0) {
        oscillators[channel].duration_samples = (uint32_t)(sample_rate * duration_sec);
    } else {
        oscillators[channel].duration_samples = 0;
    }
    
    if (delay_sec > 0) {
        oscillators[channel].start_time_ms = millis() + (uint32_t)(delay_sec * 1000);
        oscillators[channel].scheduled = true;
        oscillators[channel].active = false;
        oscillators[channel].phase = 0;
        oscillators[channel].samples_played = 0;
    } else {
        oscillators[channel].start_time_ms = 0;
        oscillators[channel].scheduled = false;
        oscillators[channel].active = true;
        oscillators[channel].phase = 0;
        oscillators[channel].samples_played = 0;
    }
    
    oscillators[channel].envelope = 0;  // Start with fade-in
    
    // Rebuild active list
    rebuildActiveChannelList();
    
    return channel;
}

void cancelScheduled(int8_t channel) {
    if (channel >= 0 && channel < MAX_CHANNELS) {
        oscillators[channel].scheduled = false;
        oscillators[channel].active = false;
        rebuildActiveChannelList();
    }
}

void stopChannel(int8_t channel) {
    if (channel >= 0 && channel < MAX_CHANNELS) {
        // Don't instantly stop - trigger fade out instead
        // Set duration to fade out over next 64 samples
        if (oscillators[channel].active) {
            oscillators[channel].duration_samples = oscillators[channel].samples_played + 64;
            // The timer callback will fade out and then deactivate
        } else {
            // Not active yet, just cancel it
            oscillators[channel].active = false;
            oscillators[channel].scheduled = false;
            oscillators[channel].amplitude = 0;
            rebuildActiveChannelList();
        }
    }
}

void stopAllTones() {
    for (int i = 0; i < MAX_CHANNELS; i++) {
        // Trigger fade out for active channels
        if (oscillators[i].active) {
            oscillators[i].duration_samples = oscillators[i].samples_played + 64;
        } else {
            oscillators[i].active = false;
            oscillators[i].scheduled = false;
            oscillators[i].amplitude = 0;
        }
    }
    // Don't clear active list - let channels fade out naturally
}

bool isChannelActive(int8_t channel) {
    if (channel < 0 || channel >= MAX_CHANNELS) return false;
    return oscillators[channel].active || oscillators[channel].scheduled;
}

uint8_t getActiveChannelCount() {
    uint8_t count = 0;
    for (int i = 0; i < MAX_CHANNELS; i++) {
        if (oscillators[i].active || oscillators[i].scheduled) count++;
    }
    return count;
}

uint8_t getPlayingChannelCount() {
    uint8_t count = 0;
    for (int i = 0; i < MAX_CHANNELS; i++) {
        if (oscillators[i].active) count++;
    }
    return count;
}

uint32_t getAudioStartTime() {
    return audio_start_time;
}

uint32_t getSampleRate() {
    return sample_rate;
}

uint8_t getBitDepth() {
    return bit_depth;
}

uint32_t getTimerCallCount() {
    return timer_call_count;
}

uint32_t getBufferSkipCount() {
    return buffer_full_skips;
}

uint32_t getBufferUnderrunCount() {
    return buffer_empty_count;
}

uint32_t getRecoveryWriteCount() {
    return recovery_writes;
}

bool isHeadphonePluggedIn() {
    // NOTE: Headphone detection requires writing to TLV320 registers
    // which the Adafruit Arduino library doesn't currently expose.
    // For now, this returns false (headphones not detected).
    // AUTO_DETECT mode won't switch automatically - use other modes instead.
    return false;
}

void setAudioOutput(AudioOutputMode mode) {
    if (!audio_ready) return;
    
    current_output_mode = mode;
    
    // Use PGA unmute parameter to control outputs
    switch(mode) {
        case AUDIO_OUT_HEADPHONES_ONLY:
            codec.configureHPL_PGA(0, true);   // Unmute headphones
            codec.configureHPR_PGA(0, true);
            codec.configureSPK_PGA(TLV320_SPK_GAIN_6DB, false);  // Mute speaker
            break;
            
        case AUDIO_OUT_SPEAKER_ONLY:
            codec.configureHPL_PGA(0, false);  // Mute headphones
            codec.configureHPR_PGA(0, false);
            codec.configureSPK_PGA(TLV320_SPK_GAIN_6DB, true);   // Unmute speaker
            break;
            
        case AUDIO_OUT_BOTH:
            codec.configureHPL_PGA(0, true);   // Unmute both
            codec.configureHPR_PGA(0, true);
            codec.configureSPK_PGA(TLV320_SPK_GAIN_6DB, true);
            break;
            
        case AUDIO_OUT_AUTO_DETECT:
            // Will be handled in updateI2SAudio()
            break;
    }
}

AudioOutputMode getAudioOutputMode() {
    return current_output_mode;
}

void printI2SAudioDiagnostics() {
    noInterrupts();  // Atomic read
    uint32_t calls = timer_call_count;
    uint32_t skips = buffer_full_skips;
    uint32_t empties = buffer_empty_count;
    uint32_t recoveries = recovery_writes;
    uint8_t active = active_channel_count;
    interrupts();
    
    int available = i2s.availableForWrite();
    
    Serial.println("\n=== I2S AUDIO DIAGNOSTICS ===");
    Serial.print("Timer calls: ");
    Serial.println(calls);
    Serial.print("Buffer overflow skips: ");
    Serial.print(skips);
    Serial.print(" (");
    Serial.print((skips * 100.0) / calls, 2);
    Serial.println("%)");
    Serial.print("Buffer underrun events: ");
    Serial.print(empties);
    Serial.print(" (");
    Serial.print((empties * 100.0) / calls, 2);
    Serial.println("%)");
    Serial.print("Recovery burst writes: ");
    Serial.println(recoveries);
    Serial.print("Active channels: ");
    Serial.println(active);
    // Buffer health assessment
    Serial.print("Buffer available: ");
    Serial.print(available);
    Serial.print(" bytes (");
    Serial.print((available * 100) / actual_buffer_size);
    Serial.println("% empty)");
    
    Serial.print("Buffer health: ");
    uint32_t critical_threshold = actual_buffer_size * 85 / 100;
    uint32_t warning_threshold = actual_buffer_size * 60 / 100;
    uint32_t nearly_full_threshold = actual_buffer_size * 18 / 100;
    
    if (available > critical_threshold) {
        Serial.println("CRITICAL - UNDERRUN IMMINENT");
    } else if (available > warning_threshold) {
        Serial.println("WARNING - Running low");
    } else if (available < 4) {
        Serial.println("FULL - Can't write");
    } else if (available < nearly_full_threshold) {
        Serial.println("HEALTHY - Nearly full");
    } else {
        Serial.println("HEALTHY");
    }
    
    Serial.print("Timer running: ");
    Serial.println(timer_running ? "YES" : "NO");
    Serial.println("============================\n");
}

void resetI2SAudioDiagnostics() {
    noInterrupts();
    timer_call_count = 0;
    buffer_full_skips = 0;
    buffer_empty_count = 0;
    recovery_writes = 0;
    interrupts();
}

uint32_t getActualBufferSize() {
    return actual_buffer_size;
}

uint32_t getBufferAvailable() {
    return i2s.availableForWrite();
}