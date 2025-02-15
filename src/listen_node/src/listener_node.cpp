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

class WhisperNode : public rclcpp::Node {
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr transcript_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr listening_pub_;
    std::atomic<bool> is_running_{true};
    FILE* whisper_pipe_{nullptr};
    std::thread whisper_thread_;

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
        playSound(state);
    }

    void processWhisperStream() {
        const char* model_env = std::getenv("MODEL");
        if (!model_env) {
            RCLCPP_ERROR(this->get_logger(), "MODEL environment variable not set");
            return;
        }
    
        std::string model_name = model_env ? model_env : "base.en";

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
                    playSound(1);
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
            "spoken_str", 10);
        listening_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "whisper_listening", 10);

        whisper_thread_ = std::thread(&WhisperNode::processWhisperStream, this);
        
        RCLCPP_INFO(this->get_logger(), "Whisper Node initialized");
    }

    ~WhisperNode() {
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
