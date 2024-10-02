#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <mutex>
#include <algorithm>

class JointStateProcessor : public rclcpp::Node {
public:
    JointStateProcessor() : Node("joint_state_remapper") {
        this->declare_parameter<std::string>("joint_states_topic_1", "/a/ur/joint_states");
        this->declare_parameter<std::string>("joint_states_topic_2", "/a/tool/joint_states");
        this->declare_parameter<std::string>("processed_joint_states_topic", "/joint_states");
        this->declare_parameter<std::string>("prefix1","a_ur");
        this->declare_parameter<std::string>("prefix2","a_tool_");
        this->declare_parameter<std::string>("prefix1_out","");
        this->declare_parameter<std::string>("prefix2_out","tool_");

        std::string joint_states_topic_1, joint_states_topic_2, processed_joint_states_topic;
        std::string prefix1, prefix2, prefix1_out, prefix2_out;
        
        this->get_parameter("joint_states_topic_1", joint_states_topic_1);
        this->get_parameter("joint_states_topic_2", joint_states_topic_2);
        this->get_parameter("processed_joint_states_topic", processed_joint_states_topic);
        this->get_parameter("prefix1", prefix1);
        this->get_parameter("prefix2", prefix2);
        this->get_parameter("prefix1_out", prefix1_out);
        this->get_parameter("prefix2_out", prefix2_out);
        prefix1_ = prefix1;
        prefix2_ = prefix2;
        prefix1_out_ = prefix1_out;
        prefix2_out_ = prefix2_out;
        auto callback_1 = [this](const sensor_msgs::msg::JointState::SharedPtr msg) -> void {
            this->joint_state_callback(msg, 1);
        };
        auto callback_2 = [this](const sensor_msgs::msg::JointState::SharedPtr msg) -> void {
            this->joint_state_callback(msg, 2);
        };

        // Create a QoS configuration with Best Effort reliability
        // auto qos = rclcpp::QoS(rclcpp::ReliabilityPolicy::BestEffort).history(rclcpp::KeepLast(10));// Use this QoS setting when creating the subscription

        subscription_1_ = this->create_subscription<sensor_msgs::msg::JointState>(
            joint_states_topic_1, rclcpp::QoS(1).best_effort() ,callback_1);

        subscription_2_ = this->create_subscription<sensor_msgs::msg::JointState>(
            joint_states_topic_2, rclcpp::QoS(1).best_effort() ,callback_2);

        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(processed_joint_states_topic, 10);
    }

private:
    std::string prefix1_, prefix2_, prefix1_out_, prefix2_out_;
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg, int source_id) {
        std::lock_guard<std::mutex> lock(mutex_);
        // Combine and store joint states based on the source ID
        if (source_id == 1) {
            last_joint_state_1_ = *msg;
        } else if (source_id == 2) {
            last_joint_state_2_ = *msg;
        }

        // Process and publish combined and remapped joint states
        process_and_publish_joint_states();
    }

    void process_and_publish_joint_states() {
        // Combine last_joint_state_1_ and last_joint_state_2_ first
        sensor_msgs::msg::JointState combined;
        combine_joint_states(last_joint_state_1_, last_joint_state_2_, combined);

        // Now apply the remapping logic
        sensor_msgs::msg::JointState processed;
        remap_joints(combined, processed);

        // Publish the processed message
        publisher_->publish(processed);
    }

    void combine_joint_states(const sensor_msgs::msg::JointState& state1,
                              const sensor_msgs::msg::JointState& state2,
                              sensor_msgs::msg::JointState& combined) {
        // Assuming state1 and state2 contain unique joint names, simply concatenate
        for (size_t i = 0; i < state1.name.size(); ++i){
            std::string newName = state1.name[i];
            if (newName.find(prefix1_) == 0) {
                newName.replace(0, prefix1_.size(), prefix1_out_);
            }
            else if (newName.find(prefix2_) == 0) {
                newName.replace(0, prefix2_.size(), prefix2_out_);
            }
            combined.name.push_back(newName);
            combined.position.push_back(state1.position[i]);
        }
        for (size_t i = 0; i < state2.name.size(); ++i){
            std::string newName = state2.name[i];
            if (newName.find(prefix1_) == 0) {
                newName.replace(0, prefix1_.size(), prefix1_out_);
            }
            else if (newName.find(prefix2_) == 0) {
                newName.replace(0, prefix2_.size(), prefix2_out_);
            }
            combined.name.push_back(newName);
            combined.position.push_back(state2.position[i]);
        }
    }

    void remap_joints(const sensor_msgs::msg::JointState& original,
                      sensor_msgs::msg::JointState& remapped) {
        // Implement your remapping logic here
        // This is a placeholder; you'll need to adapt it to your specific logic
        double yaw1 = 0.0;
        double yaw2 = 0.0;
        double yaw0,open1;
        for (size_t i = 0; i < original.name.size(); ++i) {
            // check if yaw1 in the name
            if (original.name[i].find("yaw1") != std::string::npos) {
                yaw1 = original.position[i];
            }
            else if (original.name[i].find("yaw2") != std::string::npos) {
                yaw2 = original.position[i];
            }
            else{
                remapped.name.push_back(original.name[i]);
                remapped.position.push_back(original.position[i]);
            }
        }
        remapped.name.push_back(prefix2_out_ + "yaw1");
        remapped.position.push_back(yaw1);
        remapped.name.push_back(prefix2_out_ + "yaw2");
        remapped.position.push_back(yaw2);
        // yaw0 = (yaw1 - yaw2) / 2;
        // open1 = (yaw1 + yaw0) / 2;
        // remapped.name.push_back(prefix2_out_ + "yaw0");
        // remapped.position.push_back(yaw0);
        // remapped.name.push_back(prefix2_out_ + "open1");
        // remapped.position.push_back(open1);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_1_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_2_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    std::mutex mutex_;
    sensor_msgs::msg::JointState last_joint_state_1_;
    sensor_msgs::msg::JointState last_joint_state_2_;

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointStateProcessor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
