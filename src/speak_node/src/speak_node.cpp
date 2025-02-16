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
        
        // Stop current playback
        if (playing_) {
            SDL_PauseAudioDevice(device_, 1);
            SDL_CloseAudioDevice(device_);
            playing_ = false;
        }
        
        // Generate new audio file in a Python-safe thread
        playback_thread_ = std::thread([this, text]() {
            py::gil_scoped_acquire acquire;
            std::string filename = generate_audio(text);
            play_audio(filename);
            std::filesystem::remove(filename);
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
        std::string filename = "temp_audio_" + std::to_string(++file_counter_) + ".wav";
        
        try {
            py::object tts = tts_class_(
                py::arg("language") = "EN",
                py::arg("device") = "auto"
            );
            
            tts.attr("tts_to_file")(
                text,
                speaker_id_,
                filename,
                py::arg("speed") = 0.8
            );
        } catch (const py::error_already_set& e) {
            RCLCPP_ERROR(get_logger(), "TTS generation failed: %s", e.what());
        }
        
        return filename;
    }

    void play_audio(const std::string& filename) {
        SDL_AudioSpec wav_spec;
        Uint32 wav_length;
        Uint8 *wav_buffer = nullptr;
        
        if (!SDL_LoadWAV(filename.c_str(), &wav_spec, &wav_buffer, &wav_length)) {
            RCLCPP_ERROR(get_logger(), "Failed to load WAV file: %s", SDL_GetError());
            return;
        }
        
        SDL_AudioDeviceID device = SDL_OpenAudioDevice(nullptr, 0, &wav_spec, nullptr, 0);
        if (device == 0) {
            RCLCPP_ERROR(get_logger(), "Failed to open audio device: %s", SDL_GetError());
            SDL_FreeWAV(wav_buffer);
            return;
        }
        
        {
            std::lock_guard<std::mutex> lock(audio_mutex_);
            device_ = device;
            playing_ = true;
            interrupted_ = false;
        }
        
        SDL_QueueAudio(device, wav_buffer, wav_length);
        SDL_PauseAudioDevice(device, 0);
        
        while (true) {
            std::this_thread::sleep_for(100ms);
            std::lock_guard<std::mutex> lock(audio_mutex_);
            if (!playing_ || interrupted_ || SDL_GetQueuedAudioSize(device) == 0) {
                break;
            }
        }
        
        {
            std::lock_guard<std::mutex> lock(audio_mutex_);
            if (playing_) {
                SDL_CloseAudioDevice(device);
                playing_ = false;
            }
        }
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
