// #include "think_node/think_node.hpp"
#include "think_node/ollama.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <mutex>
#include <atomic>
#include <queue>
#include <condition_variable>

using namespace std::chrono_literals;

class ThinkNode : public rclcpp::Node
{
public:
  ThinkNode() : Node("think_node"), current_model_("gemma2:2b")
  {
    // system("OLLAMA_HOST=0.0.0.0:11434 ollama serve &");  // Force port specification

    // Initialize Ollama with proper host configuration
    ollama::setServerURL("http://0.0.0.0:11434");  // Allow external connections
    ollama::allow_exceptions(true);  // Enable proper error reporting
    // system("ollama serve &");  // Start server in background
    std::this_thread::sleep_for(2s);  // Wait for server initialization
    
    // Verify model existence first
    if(!verify_model_exists(current_model_)) {
        // Set longer timeout for large model downloads
        ollama::setReadTimeout(300);  // 5 minutes timeout
        
        try {
            // First ensure server is running
            if(!ollama::is_running()) {
                RCLCPP_ERROR(get_logger(), "Ollama server not running");
                throw std::runtime_error("Ollama server not available");
            }

            // Attempt model pull with retries
            int max_retries = 3;
            bool pull_success = false;
            
            for(int i = 0; i < max_retries && !pull_success; i++) {
                try {
                    pull_success = ollama::pull_model(current_model_, true);
                    if(pull_success) {
                        RCLCPP_INFO(get_logger(), "Successfully downloaded model: %s", current_model_.c_str());
                        break;
                    }
                }
                catch(const ollama::exception& e) {
                    RCLCPP_WARN(get_logger(), "Download attempt %d failed: %s", i+1, e.what());
                    std::this_thread::sleep_for(std::chrono::seconds(2));
                }
            }

            if(!pull_success) {
                RCLCPP_FATAL(get_logger(), "Failed to download model after %d attempts", max_retries);
                throw std::runtime_error("Model download failed");
            }
        }
        catch(const std::exception& e) {
            RCLCPP_FATAL(get_logger(), "Critical error during model setup: %s", e.what());
            throw;
        }
    }
      
    // Initialize publishers and subscribers
    listener_sub_ = create_subscription<std_msgs::msg::String>(
      "/spoken_str", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
        handle_new_input(msg->data);
      });
    
    output_pub_ = create_publisher<std_msgs::msg::String>("/generated_response", 10);
    
    // Initialize Ollama
    // ollama::setServerURL("http://localhost:11434");
    load_model(current_model_);

    // Start processing thread
    processing_thread_ = std::thread(&ThinkNode::processing_loop, this);
  }
  
  ~ThinkNode()
  {
    stop_processing_ = true;
    stop_ollama_model();
    cv_.notify_all();
    if(processing_thread_.joinable()) processing_thread_.join();
  }

private:
  // ROS2 members
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr listener_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr output_pub_;
  
  // Threading members
  std::thread processing_thread_;
  std::mutex queue_mutex_;
  std::condition_variable cv_;
  std::queue<std::string> input_queue_;
  std::atomic<bool> stop_processing_{false};
  std::atomic<bool> generation_in_progress_{false};
  
  // Ollama members
  std::string current_model_;
  std::mutex ollama_mutex_;
  ollama::response current_context_;
  
  bool verify_model_exists(const std::string& model)
  {
    auto models = ollama::list_models();
    return std::find(models.begin(), models.end(), model) != models.end();
  }
  bool check_server_connection() {
      try {
          // First check basic server availability
          if (!ollama::is_running()) {
              return false;
          }
          
          // Then verify model status
          auto model_info = ollama::show_model_info(current_model_);
          return !model_info.is_null() && model_info.contains("modelfile");
      } 
      catch (const std::exception& e) {
          RCLCPP_ERROR(get_logger(), "Connection check failed: %s", e.what());
          return false;
      }
  }

  void handle_new_input(const std::string& input)
  {
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      input_queue_.push(input);
      std::cout<<"input queque pushed"<<input<<std::endl;
    }
    cv_.notify_one();
    
    if(generation_in_progress_) {
      RCLCPP_INFO(get_logger(), "Interrupting current generation");
      load_model(current_model_);
    }
  }

  // Modified load_model function
  void load_model(const std::string& model)
  {
    std::lock_guard<std::mutex> lock(ollama_mutex_);
    
    if(!ollama::load_model(model)) {
      // Attempt to pull model if not found
      if(!verify_model_exists(model)) {
        RCLCPP_INFO(get_logger(), "Model not found, attempting to pull: %s", model.c_str());
        if(!ollama::pull_model(model)) {
          RCLCPP_ERROR(get_logger(), "Failed to pull model: %s", model.c_str());
          throw std::runtime_error("Model pull failed");
        }
      }
      
      if(!ollama::load_model(model)) {
        RCLCPP_ERROR(get_logger(), "Final model load failed: %s", model.c_str());
        throw std::runtime_error("Model loading failed");
      }
    }
  }
  void stop_ollama_model()
  {
      std::lock_guard<std::mutex> lock(ollama_mutex_);
      try {
          // Check if model is running first
          auto running = ollama::list_running_models();
          if(std::find(running.begin(), running.end(), current_model_) != running.end()) {
              std::string cmd = "ollama stop " + current_model_;
              if(system(cmd.c_str()) == 0) {
                  RCLCPP_INFO(get_logger(), "Successfully stopped model: %s", current_model_.c_str());
              } else {
                  RCLCPP_ERROR(get_logger(), "Failed to stop model: %s", current_model_.c_str());
              }
          }
      }
      catch(const std::exception& e) {
          RCLCPP_ERROR(get_logger(), "Failed to stop model: %s", e.what());
      }
  }

  void processing_loop()
  {
    while(!stop_processing_) {

      // Use in processing_loop():
      if (!check_server_connection()) {
        RCLCPP_ERROR(get_logger(), "Server connection failed");
        throw std::runtime_error("Connection validation failed");
      }

      std::string current_input;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        cv_.wait(lock, [this](){ 
          return !input_queue_.empty() || stop_processing_; 
        });
        
        if(stop_processing_) break;
        
        // Add model readiness check
        if (!ollama::is_running()) {
            throw std::runtime_error("Ollama server not responding");
        }
        
        // Aggregate all pending inputs
        current_input.clear();
        while(!input_queue_.empty()) {
          current_input += input_queue_.front() + " ";
          input_queue_.pop();
        }
      }

      generation_in_progress_ = true;
      
      try {
        auto start = std::chrono::high_resolution_clock::now();
        
        // Generate response with context
        ollama::request req(current_model_, current_input); 
        auto response = ollama::generate(req);
        
        auto end = std::chrono::high_resolution_clock::now();
        double latency = std::chrono::duration<double>(end - start).count();
        
        
        // Publish response
        auto output_msg = std_msgs::msg::String();
        output_msg.data = response;
        output_pub_->publish(output_msg);

        RCLCPP_INFO(get_logger(), "Generated response in %.2fs >> %s", latency, response.as_simple_string().c_str());

        // Update context
        std::lock_guard<std::mutex> lock(ollama_mutex_);
        current_context_ = response;
      }
      catch(const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Generation failed: %s", e.what());
      }
      
      generation_in_progress_ = false;
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ThinkNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
