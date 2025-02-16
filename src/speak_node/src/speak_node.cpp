#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <SDL2/SDL.h>
#include <pybind11/embed.h>
#include <filesystem>
#include <thread>
#include <atomic>
#include <mutex>

namespace py = pybind11;
using namespace std::chrono_literals;

class SpeechNode : public rclcpp::Node {
public:
    SpeechNode() : Node("speak_node"), guard_(std::make_unique<py::scoped_interpreter>()) {
        // Initialize Python objects before SDL
        try {
            py::module melo = py::module::import("melo.api");
            tts_class_ = melo.attr("TTS");
            
            // Create TTS instance first
            py::object tts_instance = tts_class_(
                py::arg("language") = "EN",
                py::arg("device") = "auto"
            );
            
            // Get hps from instance instead of class
            speaker_ids_ = tts_instance.attr("hps").attr("data").attr("spk2id");
            speaker_id_ = speaker_ids_["EN-US"];
        } catch (const py::error_already_set& e) {
            RCLCPP_FATAL(get_logger(), "Python error: %s", e.what());
            throw std::runtime_error("Python module initialization failed");
        }

        // Initialize SDL after Python
        if (SDL_Init(SDL_INIT_AUDIO) != 0) {
            RCLCPP_FATAL(get_logger(), "SDL initialization failed: %s", SDL_GetError());
            throw std::runtime_error("SDL init failed");
        }

        // Create subscribers
        text_sub_ = create_subscription<std_msgs::msg::String>(
            "/generated_response", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                RCLCPP_INFO(get_logger(), "Received text: '%s'", msg->data.c_str());
                handle_new_text(msg->data);
            });
                    
        bool_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/is_speaking", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                handle_interrupt(msg->data);
            });
    }

    ~SpeechNode() {
        {
            std::lock_guard<std::mutex> lock(audio_mutex_);
            if (playing_) {
                SDL_CloseAudioDevice(device_);
            }
        }
        SDL_Quit();
    }

private:
    void handle_new_text(const std::string& text) {
        std::lock_guard<std::mutex> lock(audio_mutex_);
        
        RCLCPP_INFO(get_logger(), "Starting to handle new text: '%s'", text.c_str());
        
        // Stop current playback
        if (playing_) {
            RCLCPP_INFO(get_logger(), "Stopping current playback");
            SDL_PauseAudioDevice(device_, 1);
            SDL_CloseAudioDevice(device_);
            playing_ = false;
        }
        
        // Generate new audio file in a Python-safe thread
        playback_thread_ = std::thread([this, text]() {
            RCLCPP_INFO(get_logger(), "Starting audio generation thread");
            py::gil_scoped_acquire acquire;
            std::string filename = generate_audio(text);
            
            if (!filename.empty()) {
                RCLCPP_INFO(get_logger(), "Audio generated successfully, playing file: %s", filename.c_str());
                play_audio(filename);
                std::filesystem::remove(filename);
            } else {
                RCLCPP_ERROR(get_logger(), "Failed to generate audio file");
            }
        });
        playback_thread_.detach();
    }

    void handle_interrupt(bool user_spoke) {
        std::lock_guard<std::mutex> lock(audio_mutex_);
        
        if (user_spoke && playing_) {
            SDL_PauseAudioDevice(device_, 1);
            interrupted_ = true;
        }
        else if (!user_spoke && interrupted_) {
            SDL_PauseAudioDevice(device_, 0);
            interrupted_ = false;
        }
    }

    std::string generate_audio(const std::string& text) {
        py::gil_scoped_acquire acquire;
        std::string filename = "/tmp/audio_" + std::to_string(++file_counter_) + ".wav"; // Use /tmp for safer cleanup
        RCLCPP_INFO(get_logger(), "Starting generate_audio fn ...");
        try {
            RCLCPP_INFO(get_logger(), "Generating audio for: '%s'", text.c_str());
            
            // Create FRESH TTS instance each time
            py::object tts = tts_class_(
                py::arg("language") = "EN",
                py::arg("device") = "auto"
            );
            
            // Verify file creation
            if(std::filesystem::exists(filename)) {
                std::filesystem::remove(filename);
            }

            tts.attr("tts_to_file")(text, speaker_id_, filename, py::arg("speed") = 0.8);
            
            // Add file verification
            if(!std::filesystem::exists(filename)) {
                RCLCPP_ERROR(get_logger(), "File creation failed!");
                return "";
            }
            RCLCPP_INFO(get_logger(), "Created: %s (Size: %ld bytes)", 
                    filename.c_str(), std::filesystem::file_size(filename));

        } catch (const py::error_already_set& e) {
            RCLCPP_ERROR(get_logger(), "TTS Error: %s", e.what());
            return "";
        }
        return filename;
    }
    
    void play_audio(const std::string& filename) {
        // Add existence check
        if(!std::filesystem::exists(filename)) {
            RCLCPP_ERROR(get_logger(), "Audio file missing: %s", filename.c_str());
            return;
        }
    
        // SDL INITIALIZATION VERIFICATION
        SDL_AudioSpec wav_spec;
        Uint32 wav_length;
        Uint8* wav_buffer = nullptr;
    
        if(!SDL_LoadWAV(filename.c_str(), &wav_spec, &wav_buffer, &wav_length)) {
            RCLCPP_ERROR(get_logger(), "SDL Load Failed: %s", SDL_GetError());
            
            // Fallback to aplay with logging
            RCLCPP_INFO(get_logger(), "Using aplay fallback");
            std::string cmd = "aplay -v \"" + filename + "\"";
            int result = system(cmd.c_str());
            if(result != 0) {
                RCLCPP_ERROR(get_logger(), "aplay failed (%d)", result);
            }
            return;
        }
    
        // BUFFER SIZE ADJUSTMENT (Prevent underruns)
        wav_spec.samples = 4096; // Increase buffer size
        
        SDL_AudioDeviceID device = SDL_OpenAudioDevice(nullptr, 0, &wav_spec, nullptr, 0);
        if(!device) {
            RCLCPP_ERROR(get_logger(), "SDL Open Failed: %s", SDL_GetError());
            SDL_FreeWAV(wav_buffer);
            return;
        }
    
        // QUEUE MANAGEMENT
        SDL_ClearQueuedAudio(device); // Clear previous data
        SDL_QueueAudio(device, wav_buffer, wav_length);
        SDL_PauseAudioDevice(device, 0); // Start playback
    
        // MONITOR PLAYBACK
        const auto start_time = std::chrono::steady_clock::now();
        while(SDL_GetQueuedAudioSize(device) > 0) {
            std::this_thread::sleep_for(50ms);
            
            // Timeout after 30 seconds
            if(std::chrono::steady_clock::now() - start_time > 30s) {
                RCLCPP_WARN(get_logger(), "Audio playback timeout");
                break;
            }
        }
    
        // CLEANUP
        SDL_CloseAudioDevice(device);
        SDL_FreeWAV(wav_buffer);
    }
    
    // Member variables
    std::unique_ptr<py::scoped_interpreter> guard_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr text_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bool_sub_;
    std::thread playback_thread_;
    std::atomic<bool> playing_{false};
    std::atomic<bool> interrupted_{false};
    std::mutex audio_mutex_;
    SDL_AudioDeviceID device_;
    py::object tts_class_;
    py::dict speaker_ids_;
    py::object speaker_id_;
    std::atomic<uint32_t> file_counter_{0};
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SpeechNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
