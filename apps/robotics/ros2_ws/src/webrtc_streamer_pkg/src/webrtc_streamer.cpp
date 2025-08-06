    #include <rclcpp/rclcpp.hpp>
    #include <sensor_msgs/msg/image.hpp>
    #include <std_msgs/msg/string.hpp>

    #include <gst/gst.h>
    #include <gst/webrtc/webrtc.h>
    #include <gst/app/gstappsrc.h>

    #include <mutex>
    #include <string>
    #include <vector>
    #include <atomic>

    class WebRTCStreamerNode : public rclcpp::Node
    {
    public:
        WebRTCStreamerNode()
        : Node("webrtc_streamer_node"),
        answer_received_(false)
        {
            gst_init(nullptr, nullptr);

            subscription_image_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/camera/image_raw", 10,
                std::bind(&WebRTCStreamerNode::image_callback, this, std::placeholders::_1));

            subscription_answer_sdp_ = this->create_subscription<std_msgs::msg::String>(
                "/webrtc/answer_sdp", 10,
                std::bind(&WebRTCStreamerNode::answer_sdp_callback, this, std::placeholders::_1));

            subscription_ice_candidate_ = this->create_subscription<std_msgs::msg::String>(
                "/webrtc/ice_candidate", 10,
                std::bind(&WebRTCStreamerNode::ice_candidate_callback, this, std::placeholders::_1));

            // 💡 Offer SDP publish
            publisher_offer_sdp_ = this->create_publisher<std_msgs::msg::String>("/ros2/sdp/offer", 10);

            publisher_ice_candidate_ = this->create_publisher<std_msgs::msg::String>("/ros2/ice-candidate", 10);

            std::string pipeline_desc =
                "appsrc name=appsrc is-live=true format=GST_FORMAT_TIME caps=video/x-raw,format=RGB,width=640,height=480,framerate=30/1 "
                "! videoconvert "
                "! x264enc tune=zerolatency bitrate=500 speed-preset=ultrafast "
                "! rtph264pay config-interval=1 pt=96 "
                "! webrtcbin name=webrtcbin";

            GError* error = nullptr;
            pipeline_ = gst_parse_launch(pipeline_desc.c_str(), &error);
            if (error) {
                RCLCPP_ERROR(this->get_logger(), "Failed to create pipeline: %s", error->message);
                g_error_free(error);
                throw std::runtime_error("Failed to create pipeline");
            }

            appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline_), "appsrc");
            webrtcbin_ = gst_bin_get_by_name(GST_BIN(pipeline_), "webrtcbin");

            g_signal_connect(webrtcbin_, "on-ice-candidate", G_CALLBACK(+[](
                GstElement* webrtc, guint mlineindex, gchar* candidate, gpointer user_data) {
                
                auto* self = static_cast<WebRTCStreamerNode*>(user_data);
                std_msgs::msg::String msg;
                msg.data = std::string(candidate);
                self->publisher_ice_candidate_->publish(msg);

                RCLCPP_INFO(self->get_logger(), "Published local ICE candidate");
            }), this);
            
            // 💡 시그널 연결
            g_signal_connect(webrtcbin_, "on-negotiation-needed", G_CALLBACK(+[](GstElement* webrtc, gpointer user_data) {
                auto* self = static_cast<WebRTCStreamerNode*>(user_data);

                GstPromise* promise = gst_promise_new_with_change_func(
                    +[](GstPromise* promise, gpointer user_data) {
                        const GstStructure* reply = gst_promise_get_reply(promise);
                        GstWebRTCSessionDescription* offer = nullptr;

                        gst_structure_get(reply, "offer", GST_TYPE_WEBRTC_SESSION_DESCRIPTION, &offer, NULL);
                        g_signal_emit_by_name(static_cast<WebRTCStreamerNode*>(user_data)->webrtcbin_, "set-local-description", offer, nullptr);

                        gchar* sdp_str = gst_sdp_message_as_text(offer->sdp);
                        std_msgs::msg::String msg;
                        msg.data = std::string(sdp_str);
                        static_cast<WebRTCStreamerNode*>(user_data)->publisher_offer_sdp_->publish(msg);

                        RCLCPP_INFO(static_cast<WebRTCStreamerNode*>(user_data)->get_logger(), "Published SDP offer to /ros2/sdp/offer");

                        g_free(sdp_str);
                        gst_webrtc_session_description_free(offer);
                        gst_promise_unref(promise);
                    },
                    user_data,
                    nullptr
                );

                g_signal_emit_by_name(webrtc, "create-offer", nullptr, promise);
            }), this);

            gst_element_set_state(pipeline_, GST_STATE_PLAYING);
            timestamp_ = 0;
        }

        ~WebRTCStreamerNode()
        {
            gst_element_set_state(pipeline_, GST_STATE_NULL);
            gst_object_unref(appsrc_);
            gst_object_unref(webrtcbin_);
            gst_object_unref(pipeline_);
        }

    private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_image_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_answer_sdp_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_ice_candidate_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_offer_sdp_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_ice_candidate_;

        GstElement* pipeline_;
        GstElement* appsrc_;
        GstElement* webrtcbin_;

        std::mutex mutex_;
        GstClockTime timestamp_;

        std::string remote_sdp_;
        std::vector<std::string> remote_ice_candidates_;
        std::atomic<bool> answer_received_{false};
        bool webrtc_started_ = false;

        void try_start_webrtc()
        {
            if (!answer_received_ || remote_ice_candidates_.empty() || webrtc_started_) return;

            webrtc_started_ = true;

            GstSDPMessage* sdp_msg = nullptr;
            if (gst_sdp_message_new(&sdp_msg) != GST_SDP_OK) {
                RCLCPP_ERROR(this->get_logger(), "Failed to create SDP message");
                return;
            }

            if (gst_sdp_message_parse_buffer((guint8*)remote_sdp_.c_str(), remote_sdp_.size(), sdp_msg) != GST_SDP_OK) {
                RCLCPP_ERROR(this->get_logger(), "Failed to parse remote SDP");
                gst_sdp_message_free(sdp_msg);
                return;
            }

            GstWebRTCSessionDescription* answer = gst_webrtc_session_description_new(GST_WEBRTC_SDP_TYPE_ANSWER, sdp_msg);
            g_signal_emit_by_name(webrtcbin_, "set-remote-description", answer, nullptr);
            gst_webrtc_session_description_free(answer);

            for (const auto& candidate : remote_ice_candidates_) {
                g_signal_emit_by_name(webrtcbin_, "add-ice-candidate", 0, candidate.c_str());
            }

            RCLCPP_INFO(this->get_logger(), "WebRTC started (remote description + ICE set)");
        }

        void answer_sdp_callback(const std_msgs::msg::String::SharedPtr msg)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            remote_sdp_ = msg->data;
            answer_received_ = true;
            try_start_webrtc();
        }

        void ice_candidate_callback(const std_msgs::msg::String::SharedPtr msg)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            remote_ice_candidates_.push_back(msg->data);
            try_start_webrtc();
        }

        void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (msg->encoding != "rgb8") return;

            GstBuffer* buffer = gst_buffer_new_allocate(nullptr, msg->data.size(), nullptr);
            gst_buffer_fill(buffer, 0, msg->data.data(), msg->data.size());

            GST_BUFFER_PTS(buffer) = timestamp_;
            GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(1, GST_SECOND, 30);
            timestamp_ += GST_BUFFER_DURATION(buffer);

            GstFlowReturn ret;
            g_signal_emit_by_name(appsrc_, "push-buffer", buffer, &ret);
            gst_buffer_unref(buffer);

            if (ret != GST_FLOW_OK) {
                RCLCPP_ERROR(this->get_logger(), "Failed to push buffer to appsrc");
            }
        }
    };

    int main(int argc, char* argv[])
    {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<WebRTCStreamerNode>();
        rclcpp::spin(node);
        rclcpp::shutdown();
        return 0;
    }
