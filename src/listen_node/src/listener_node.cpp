#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <memory>
#include <thread>
#include <atomic>
#include <filesystem>
#include <regex>
#include <chrono>
#include <cstdlib> 
#include <portaudio.h>
#include <speex/speex_echo.h>
#include <speex/speex_preprocess.h>

class EchoCanceller {
public:
    EchoCanceller(int sample_rate, int frame_size, int filter_length) :
        frame_size_(frame_size) {
        // Initialize Speex echo canceller
        echo_state_ = speex_echo_state_init(frame_size, filter_length);
        preprocess_state_ = speex_preprocess_state_init(frame_size, sample_rate);
        
        speex_echo_ctl(echo_state_, SPEEX_ECHO_SET_SAMPLING_RATE, &sample_rate);
        speex_preprocess_ctl(preprocess_state_, SPEEX_PREPROCESS_SET_ECHO_STATE, echo_state_);
    }

    void process(const float* mic_input, const float* speaker_output, float* out, int frames) {
        // Convert float arrays to spx_int16_t
        std::vector<spx_int16_t> mic_int(frames);
        std::vector<spx_int16_t> spk_int(frames);
        std::vector<spx_int16_t> out_int(frames);
        
        // Convert float [-1,1] to int16
        for(int i=0; i<frames; i++) {
            mic_int[i] = static_cast<spx_int16_t>(mic_input[i] * 32767.0f);
            spk_int[i] = static_cast<spx_int16_t>(speaker_output[i] * 32767.0f);
        }
        
        speex_echo_cancellation(echo_state_, mic_int.data(), spk_int.data(), out_int.data());
        speex_preprocess_run(preprocess_state_, out_int.data());
        
        // Convert back to float
        for(int i=0; i<frames; i++) {
            out[i] = out_int[i] / 32768.0f;
        }
    }
    
    ~EchoCanceller() {
        speex_echo_state_destroy(echo_state_);
        speex_preprocess_state_destroy(preprocess_state_);
    }

private:
    SpeexEchoState* echo_state_;
    SpeexPreprocessState* preprocess_state_;
    int frame_size_;
};

class WhisperNode : public rclcpp::Node {
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr transcript_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr listening_pub_;
    std::atomic<bool> is_running_{true};
    FILE* whisper_pipe_{nullptr};
    std::thread whisper_thread_;

    // Audio processing members
    PaStream* audio_stream_;
    std::unique_ptr<EchoCanceller> echo_canceller_;
    std::mutex audio_mutex_;
    std::vector<float> system_audio_buffer_;
    
    // New method for audio capture and processing
    void setupAudioCapture() {
        Pa_Initialize();
        
        PaStreamParameters input_params, ref_params;
        input_params.device = Pa_GetDefaultInputDevice();
        input_params.channelCount = 1;
        input_params.sampleFormat = paFloat32;
        input_params.suggestedLatency = Pa_GetDeviceInfo(input_params.device)->defaultLowInputLatency;
        input_params.hostApiSpecificStreamInfo = nullptr;

        // System output (loopback) device
        ref_params.device = Pa_GetDefaultOutputDevice(); 
        ref_params.channelCount = 1;
        ref_params.sampleFormat = paFloat32;
        ref_params.suggestedLatency = Pa_GetDeviceInfo(ref_params.device)->defaultLowOutputLatency;
        ref_params.hostApiSpecificStreamInfo = nullptr;

        // Initialize echo canceller (using Speex or WebRTC implementation)
        echo_canceller_ = std::make_unique<EchoCanceller>(16000, 1024, 10);

        Pa_OpenStream(&audio_stream_,
                        &input_params,
                        &ref_params,
                        16000,
                        1024,
                        paClipOff,
                        [](const void* input, void* output,
                        unsigned long frame_count,
                        const PaStreamCallbackTimeInfo* time_info,
                        PaStreamCallbackFlags status_flags,
                        void* user_data) {
                            return static_cast<WhisperNode*>(user_data)->audioCallback(
                                static_cast<const float*>(input), frame_count);
                        },
                        this);

        Pa_StartStream(audio_stream_);
    }

    int audioCallback(const float* input, unsigned long frame_count) {
        std::lock_guard<std::mutex> lock(audio_mutex_);
        
        // Process audio through echo canceller
        std::vector<float> processed_audio(frame_count);
        echo_canceller_->process(input, system_audio_buffer_.data(), processed_audio.data(), frame_count);
        
        // Write processed audio to whisper-stream via pipe
        fwrite(processed_audio.data(), sizeof(float), frame_count, whisper_pipe_);
        
        return paContinue;
    }

    void playSound(bool listening) {
        if (listening) {
            // Play start listening sound (short beep)
            system("play -q -v 0.3 -n synth 0.1 sine 1000 2>/dev/null || beep -f 1000 -l 100 2>/dev/null || echo -e '\a'");
        } else {
            // Play stop listening sound (two short beeps)
            system("play -q -v 0.3  -n synth 0.1 sine 800 0.1 sine 1000 2>/dev/null || (beep -f 800 -l 100 && beep -f 1000 -l 100) 2>/dev/null || (echo -e '\a' && sleep 0.1 && echo -e '\a')");
        }
    }

    void publishListeningState(bool state) {
        auto msg = std_msgs::msg::Bool();
        msg.data = state;
        listening_pub_->publish(msg);
        // playSound(state);
    }

    void processWhisperStream() {
        setupAudioCapture();  // Initialize audio capture

        const char* model_env = std::getenv("WHISPER_MODEL");
        if (!model_env) {
            RCLCPP_ERROR(this->get_logger(), "MODEL environment variable not set");
            return;
        }
    
        std::string model_name = model_env ? model_env : "ggml-small.en.bin";

        std::string cmd = "stdbuf -oL /usr/local/src/whisper.cpp/build/bin/whisper-stream -m " + 
                         std::string("/usr/local/src/whisper.cpp/models/") + 
                         std::string(model_env) + " --capture 1";

        whisper_pipe_ = popen(cmd.c_str(), "r");
        if (!whisper_pipe_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to start whisper-stream");
            return;
        }

        char buffer[1024];
        std::string accumulated_text;
        std::chrono::steady_clock::time_point last_speech_time;
        bool is_collecting = false;
        
        const auto silence_threshold = std::chrono::seconds(3);
        std::regex ansiEscape(R"(\x1B\[[0-9;]*[A-Za-z])");
        std::regex brackets_only(R"(^\s*[\[\{\(].*[\]\}\)]\s*$)");

        publishListeningState(true);
        RCLCPP_INFO(this->get_logger(), "Started listening...");

        int start_listening = false; 

        while (is_running_ && fgets(buffer, sizeof(buffer), whisper_pipe_)) {
            std::string currentLine = buffer;
            
            if (!currentLine.empty()) {
                // Process ANSI escape sequences
                std::string processedLine;
                size_t lastPos = 0;
                std::sregex_iterator it(currentLine.begin(), currentLine.end(), ansiEscape);
                std::sregex_iterator end;

                for (; it != end; ++it) {
                    std::smatch match = *it;
                    size_t escapeStart = match.position();
                    size_t escapeLength = match.length();

                    size_t lastNewline = currentLine.rfind('\n', escapeStart);
                    
                    if (lastNewline != std::string::npos) {
                        processedLine += currentLine.substr(lastPos, lastNewline - lastPos + 1);
                        lastPos = lastNewline + 1;
                    } else {
                        lastPos = escapeStart;
                    }
                    lastPos = escapeStart + escapeLength;
                }

                processedLine += currentLine.substr(lastPos);
                currentLine = std::regex_replace(processedLine, ansiEscape, "");
                std::cout<<currentLine<<std::endl;
            }
            
            // Skip empty lines or lines with only bracketed content
            if (currentLine.empty() || std::regex_match(currentLine, brackets_only)) {
                start_listening = false;
                if (is_collecting) {
                    auto now = std::chrono::steady_clock::now();
                    if (now - last_speech_time > silence_threshold) {
                        // Publish accumulated text if we have meaningful content
                        if (!accumulated_text.empty()) {
                            auto msg = std_msgs::msg::String();
                            msg.data = accumulated_text;
                            transcript_pub_->publish(msg);
                            RCLCPP_INFO(this->get_logger(), "Published: '%s'", accumulated_text.c_str());
                        }
                        accumulated_text.clear();
                        is_collecting = false;
                    }
                }
                continue;
            }
            else {
                if(start_listening == false) {
                    // playSound(1);
                    start_listening = true;
                }
            }

            // We have speech content
            if (!is_collecting) {
                is_collecting = true;
            }
            last_speech_time = std::chrono::steady_clock::now();
            
            // Clean up the line before adding
            currentLine = std::regex_replace(currentLine, std::regex(R"(\s+)"), " ");
            currentLine = std::regex_replace(currentLine, std::regex(R"(^\s+|\s+$)"), "");
            
            // Add to accumulated text
            if (!accumulated_text.empty()) {
                accumulated_text += " ";
            }
            accumulated_text += currentLine;
        }

        publishListeningState(false);
        if (whisper_pipe_) {
            pclose(whisper_pipe_);
            whisper_pipe_ = nullptr;
        }
    }

public:
    WhisperNode() : Node("whisper_node") {
        transcript_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/spoken_str", 10);
        listening_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/whisper_listening", 10);

        whisper_thread_ = std::thread(&WhisperNode::processWhisperStream, this);
        
        RCLCPP_INFO(this->get_logger(), "Whisper Node initialized");
    }

    ~WhisperNode() {
        // Clean up audio resources
        if (audio_stream_) {
            Pa_StopStream(audio_stream_);
            Pa_CloseStream(audio_stream_);
        }
        Pa_Terminate();
        
        // Clean up whisper stream resources
        is_running_ = false;
        if (whisper_pipe_) {
            pclose(whisper_pipe_);
        }
        if (whisper_thread_.joinable()) {
            whisper_thread_.join();
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WhisperNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
