// speak_node.cpp
#include <memory>
#include <string>
#include <chrono>
#include <thread>
#include <mutex>
#include <pybind11/embed.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <SDL2/SDL.h>
#include <SDL2/SDL_audio.h>
#include <filesystem>

namespace py = pybind11;

class AudioPlayer {
public:
    // In AudioPlayer constructor
    AudioPlayer() {
        if (SDL_Init(SDL_INIT_AUDIO) < 0) {
            throw std::runtime_error("SDL audio init failed: " + std::string(SDL_GetError()));
        }
        
        // Force ALSA driver for Linux compatibility
        SDL_setenv("SDL_AUDIODRIVER", "alsa", 1);
    }

    ~AudioPlayer() {
        SDL_CloseAudio();
        SDL_Quit();
    }
    void pause(bool should_pause) {
        if (device_id != 0) {
            SDL_PauseAudioDevice(device_id, should_pause ? 1 : 0);
        }
    }

    bool playWavFile(const std::string& filename) {
        // Close existing device before reopening
        if (device_id != 0) {
            SDL_CloseAudioDevice(device_id);
            device_id = 0;
        }

        SDL_AudioSpec loadedSpec;
        if (SDL_LoadWAV(filename.c_str(), &loadedSpec, &wavBuffer, &wavLength) == nullptr) {
            return false;
        }

        // Reset playback position
        audioPos = 0;
        playing = true;

        SDL_AudioSpec desiredSpec = loadedSpec;
        desiredSpec.callback = audioCallback;
        desiredSpec.userdata = this;

        // Open new audio device
        device_id = SDL_OpenAudioDevice(nullptr, 0, &desiredSpec, nullptr, 0);
        if (device_id == 0) {
            return false;
        }

        SDL_PauseAudioDevice(device_id, 0);
        return true;
    }

    void stop() {
        if (device_id != 0) {
            SDL_PauseAudioDevice(device_id, 1);
            SDL_CloseAudioDevice(device_id);
            device_id = 0;
        }
        SDL_FreeWAV(wavBuffer);
        playing = false;
    }

    bool isPlaying() const {
        return playing;
    }

private:
    SDL_AudioSpec wavSpec;
    Uint8* wavBuffer = nullptr;
    Uint32 wavLength = 0;
    Uint32 audioPos = 0;
    bool playing = false;
    SDL_AudioDeviceID device_id = 0; // Add this member

    static void audioCallback(void* userdata, Uint8* stream, int len) {
        AudioPlayer* player = static_cast<AudioPlayer*>(userdata);
        if (!player->playing) return;
    
        Uint32 remaining = player->wavLength - player->audioPos;
        if (remaining == 0) {
            player->playing = false;
            return;
        }
    
        Uint32 bytesToCopy = (remaining > (Uint32)len) ? len : remaining;
        SDL_memcpy(stream, &player->wavBuffer[player->audioPos], bytesToCopy);
        player->audioPos += bytesToCopy;
    }
};

#pragma GCC visibility push(default)

class SpeakNode : public rclcpp::Node {
public:
    SpeakNode() : Node("speak_node") {
        // Parameter declaration with descriptor
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
        param_desc.description = "Audio output device name";
        
        // Declare parameter properly
        this->declare_parameter("audio_device", "default", param_desc);
        
        // Retrieve parameter value
        std::string audio_device = this->get_parameter("audio_device").as_string();

        // Initialize Python interpreter
        py::initialize_interpreter();

        // Import melo.api and create TTS instance
        auto melo = py::module::import("melo.api");
        tts = melo.attr("TTS")("EN", "auto");

        // Initialize subscriptions
        response_sub_ = create_subscription<std_msgs::msg::String>(
            "/generated_response", 10,
            std::bind(&SpeakNode::responseCallback, this, std::placeholders::_1));

        listening_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/is_listening", 10,
            std::bind(&SpeakNode::listeningCallback, this, std::placeholders::_1));

        continue_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/continue_playing", 10,
            std::bind(&SpeakNode::continueCallback, this, std::placeholders::_1));

        player_ = std::make_unique<AudioPlayer>();
        current_audio_file_ = "";
        is_paused_ = false;
    }

    ~SpeakNode() {
        cleanupAudioFile();
        py::finalize_interpreter();
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr response_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr listening_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr continue_sub_;
    std::unique_ptr<AudioPlayer> player_;
    py::object tts;
    std::string current_audio_file_;
    std::mutex audio_mutex_;
    bool is_paused_;

    void responseCallback(const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(audio_mutex_);
        
        // Stop current playback if any
        if (player_->isPlaying()) {
            player_->stop();
        }

        // Clean up previous audio file
        cleanupAudioFile();

        // Generate new audio file
        current_audio_file_ = "temp_" + std::to_string(std::chrono::system_clock::now().time_since_epoch().count()) + ".wav";
        
        try {
            // Convert text to speech using melo TTS
            tts.attr("tts_to_file")(
                msg->data,
                tts.attr("hps").attr("data").attr("spk2id")["EN-US"],
                current_audio_file_,
                0.8  // speed
            );

            // Play the new audio file
            if (!player_->playWavFile(current_audio_file_)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to play audio file");
                return;
            }
        } catch (const py::error_already_set& e) {
            RCLCPP_ERROR(this->get_logger(), "Python error: %s", e.what());
        }
    }

    void listeningCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(audio_mutex_);
        
        if (msg->data && player_->isPlaying()) {
            // User is speaking, pause playback
            player_->pause(true);
            is_paused_ = true;
        } else if (is_paused_) {
            // Resume playback if it was paused
            player_->pause(false);
            is_paused_ = false;
        }
    }
    
    void continueCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(audio_mutex_);
        
        if (msg->data && is_paused_ && player_->isPlaying()) {
            // Resume playback
            player_->pause(false);
            is_paused_ = false;
        }
    }
    
    void cleanupAudioFile() {
        if (!current_audio_file_.empty() && std::filesystem::exists(current_audio_file_)) {
            try {
                std::filesystem::remove(current_audio_file_);
            } catch (const std::filesystem::filesystem_error& e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to remove audio file: %s", e.what());
            }
        }
    }
};
#pragma GCC visibility pop

int main(int argc, char** argv) {
    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SpeakNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}