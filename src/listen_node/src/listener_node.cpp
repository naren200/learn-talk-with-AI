#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <memory>
#include <thread>
#include <atomic>
#include <filesystem>
#include <regex>

class WhisperNode : public rclcpp::Node {
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr transcript_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr listening_pub_;
    std::atomic<bool> is_running_{true};
    FILE* whisper_pipe_{nullptr};
    std::thread whisper_thread_;
    
    void publishListeningState(bool state) {
        auto msg = std_msgs::msg::Bool();
        msg.data = state;
        listening_pub_->publish(msg);
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
                         std::string(model_env) + " --capture 2";

        whisper_pipe_ = popen(cmd.c_str(), "r");
        if (!whisper_pipe_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to start whisper-stream");
            return;
        }

        char buffer[1024];
        std::string current_line;
        std::regex ansi_escape(R"(\x1B\[[0-9;]*[A-Za-z])");

        publishListeningState(true);
        RCLCPP_INFO(this->get_logger(), "Started listening...");

        while (is_running_ && fgets(buffer, sizeof(buffer), whisper_pipe_)) {
            current_line = buffer;

            // Clean up the transcribed text
            current_line = std::regex_replace(current_line, ansi_escape, "");
            current_line = std::regex_replace(current_line, std::regex(R"(\s*[\[\{\(\]}\)])"), "");
            current_line = std::regex_replace(current_line, std::regex(R"(\s+)"), " ");
            current_line = std::regex_replace(current_line, std::regex(R"(^\s+|\s+$)"), "");

            if (!current_line.empty() && 
                current_line.find_first_not_of(' ') != std::string::npos) {
                auto msg = std_msgs::msg::String();
                msg.data = current_line;
                transcript_pub_->publish(msg);
                RCLCPP_INFO(this->get_logger(), "Published: '%s'", current_line.c_str());
            }
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
