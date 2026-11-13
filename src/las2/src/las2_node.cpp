/**
 * @file las2_node.cpp
 * @brief Adapt stereo DMA-BUF descriptors to LAS2 depth and preview outputs.
 */
#include "las2/center_depth_overlay.h"
#include "las2/nv12_center_hud.h"
#include "las2/offline_fifo.h"
#include "las2/pose_gated_queue.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <filesystem> // NOLINT(build/c++17): This target requires C++17.
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <sys/sys_api.h>
#include <sys/vb_api.h>
#include <sys/vb_type.h>

#include <stereo_pipeline.h>
#include <mpp_msgs/msg/mpp_video_frame_desc.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <common/msg/dma_depth_frame_desc.hpp>

namespace las2
{

using InputFrame = mpp_msgs::msg::MppVideoFrameDesc;
using OutputFrame = common::msg::DmaDepthFrameDesc;

constexpr std::uint32_t kDepthEncodingFloat32Meters = 1;
constexpr std::uint32_t kRgbEncodingRgb8 = 1;

std::vector<std::string> split_affinity_groups(const std::string& value)
{
    std::vector<std::string> groups;
    if (value.find('/') == std::string::npos)
    {
        std::size_t start = 0;
        while (start <= value.size())
        {
            const std::size_t comma = value.find(',', start);
            const std::string group =
                value.substr(start, comma == std::string::npos ? std::string::npos : comma - start);
            if (!group.empty())
                groups.push_back(group);
            if (comma == std::string::npos)
                break;
            start = comma + 1;
        }
        return groups;
    }

    std::size_t start = 0;
    while (start <= value.size())
    {
        const std::size_t slash = value.find('/', start);
        std::string group =
            value.substr(start, slash == std::string::npos ? std::string::npos : slash - start);
        std::replace(group.begin(), group.end(), ',', ';');
        if (!group.empty())
            groups.push_back(std::move(group));
        if (slash == std::string::npos)
            break;
        start = slash + 1;
    }
    return groups;
}

class ImportedBuffer
{
public:
    explicit ImportedBuffer(std::uint64_t token)
    {
        if (token == 0 || VB_Import(token, &handle_) != 0 || handle_ == 0)
        {
            throw std::runtime_error("VB_Import failed for LAS2 input");
        }
        if (VB_GetDmaBufFd(handle_, &dma_fd_) != 0 || dma_fd_ < 0)
        {
            (void)VB_ReleaseBuffer(handle_);
            handle_ = 0;
            throw std::runtime_error("VB_GetDmaBufFd failed for LAS2 input");
        }
    }

    ~ImportedBuffer()
    {
        if (handle_ != 0)
            (void)VB_ReleaseBuffer(handle_);
    }

    ImportedBuffer() = default;
    ImportedBuffer(const ImportedBuffer&) = delete;
    ImportedBuffer& operator=(const ImportedBuffer&) = delete;
    ImportedBuffer(ImportedBuffer&& other) noexcept
        : handle_(std::exchange(other.handle_, 0)), dma_fd_(std::exchange(other.dma_fd_, -1))
    {
    }
    ImportedBuffer& operator=(ImportedBuffer&& other) noexcept
    {
        if (this != &other)
        {
            if (handle_ != 0)
                (void)VB_ReleaseBuffer(handle_);
            handle_ = std::exchange(other.handle_, 0);
            dma_fd_ = std::exchange(other.dma_fd_, -1);
        }
        return *this;
    }

    int dma_fd() const { return dma_fd_; }

    void release_checked()
    {
        const auto handle = std::exchange(handle_, 0);
        dma_fd_ = -1;
        if (handle != 0 && VB_ReleaseBuffer(handle) != 0)
            throw std::runtime_error("VB_ReleaseBuffer failed for offline LAS2 input");
    }

private:
    UL handle_{0};
    S32 dma_fd_{-1};
};

struct WorkItem
{
    InputFrame descriptor;
    ImportedBuffer input;
};

struct OutputHandles
{
    UL depth{0};
    UL rgb{0};
};

using OfflineQueue = OfflineFifo<std::shared_ptr<WorkItem>, OutputHandles>;

struct OutputAllocation
{
    UL depth{0}, rgb{0};
    bool depth_exported{false}, rgb_exported{false}, committed{false};
    ~OutputAllocation()
    {
        if (!committed)
        {
            if (depth_exported)
                (void)VB_Unexport(depth);
            if (rgb_exported)
                (void)VB_Unexport(rgb);
        }
        if (depth)
            (void)VB_ReleaseBuffer(depth);
        if (rgb)
            (void)VB_ReleaseBuffer(rgb);
    }
};

class RobotLas2Node final : public rclcpp::Node
{
public:
    explicit RobotLas2Node(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : rclcpp::Node("las2", options)
    {
        input_topic_ = declare_parameter<std::string>("input_topic", "/camera/full/frame_desc");
        output_topic_ = declare_parameter<std::string>("output_topic", "/las2/depth_desc");
        ready_topic_ = declare_parameter<std::string>("ready_topic", "/las2/ready");
        preview_topic_ =
            declare_parameter<std::string>("preview_topic", "/las2/preview/frame_desc");
        publish_preview_ = declare_parameter<bool>("publish_preview", true);
        preview_center_depth_overlay_ =
            declare_parameter<bool>("preview_center_depth_overlay", false);
        preview_depth_roi_size_ = declare_parameter<int>("preview_depth_roi_size", 9);
        preview_depth_roi_min_valid_fraction_ =
            declare_parameter<double>("preview_depth_roi_min_valid_fraction", 0.5);
        preview_visualization_ =
            declare_parameter<std::string>("preview_visualization", "disparity");
        preview_disparity_min_ = declare_parameter<double>("preview_disparity_min", 0.0);
        preview_disparity_max_ = declare_parameter<double>("preview_disparity_max", 128.0);
        preview_depth_min_m_ = declare_parameter<double>("preview_depth_min_m", 0.0);
        preview_depth_max_m_ = declare_parameter<double>("preview_depth_max_m", 3.0);
        calibration_file_ = declare_parameter<std::string>("calibration_file", "");
        model_path_ = declare_parameter<std::string>("model_path", "");
        core_count_ = declare_parameter<int>("core_count", 4);
        threads_per_instance_ = declare_parameter<int>("threads_per_instance", 2);
        warmup_iterations_ = declare_parameter<int>("warmup_iterations", 2);
        core_affinity_ = declare_parameter<std::string>("core_affinity", "8;9/10;11/12;13/14;15");
        queue_depth_ = declare_parameter<int>("queue_depth", 4);
        offline_lossless_ = declare_parameter<bool>("offline_lossless", false);
        const int offline_window = declare_parameter<int>("offline_window_size", 16);
        const double offline_timeout = declare_parameter<double>("offline_timeout_sec", 180.0);
        offline_ack_topic_ =
            declare_parameter<std::string>("offline_ack_topic", "/offline_map/fusion_ack");
        offline_summary_file_ = declare_parameter<std::string>("offline_summary_file", "");
        const auto offline_expected = declare_parameter<std::int64_t>("offline_expected_frames", 0);
        offline_complete_topic_ =
            declare_parameter<std::string>("offline_complete_topic", "/offline_las2/complete");
        if (offline_lossless_)
        {
            if (offline_window < core_count_ || offline_window > 256 ||
                !std::isfinite(offline_timeout) || offline_timeout <= 0.0 ||
                offline_timeout > 86400.0 || offline_ack_topic_.empty() || offline_expected < 0 ||
                (offline_expected > 0 && offline_complete_topic_.empty()))
                throw std::runtime_error("invalid offline window/timeout/ACK settings");
            offline_expected_frames_ = static_cast<std::uint64_t>(offline_expected);
            offline_queue_ =
                std::make_unique<OfflineQueue>(static_cast<std::size_t>(offline_window));
            offline_timeout_ = std::chrono::duration_cast<OfflineQueue::Clock::duration>(
                std::chrono::duration<double>(offline_timeout));
            if (!offline_summary_file_.empty())
            {
                const auto path = std::filesystem::absolute(offline_summary_file_);
                if (std::filesystem::exists(path) ||
                    !std::filesystem::is_directory(path.parent_path()))
                    throw std::runtime_error(
                        "offline summary requires a fresh file and existing parent");
            }
        }
        pose_gate_topic_ = declare_parameter<std::string>("pose_gate_topic", "");
        const int pose_gate_image_depth = declare_parameter<int>("pose_gate_image_depth", 8);
        if (pose_gate_image_depth < 1 || pose_gate_image_depth > 32)
        {
            throw std::runtime_error("pose_gate_image_depth must be in [1, 32]");
        }
        if (!pose_gate_topic_.empty() && !offline_lossless_)
        {
            pose_gate_ = std::make_unique<PoseGatedQueue<std::shared_ptr<WorkItem>>>(
                static_cast<std::size_t>(pose_gate_image_depth), 64);
        }
        const int output_pool_depth = declare_parameter<int>("output_pool_depth",
            offline_lossless_ ? offline_window + core_count_ + 2 : queue_depth_ + core_count_ + 2);
        if (offline_lossless_ && output_pool_depth < offline_window + core_count_)
            throw std::runtime_error(
                "offline output_pool_depth must cover window plus inference workers");
        output_export_limit_ = static_cast<std::size_t>(output_pool_depth);

        if (calibration_file_.empty() || model_path_.empty())
        {
            throw std::runtime_error("las2 requires calibration_file and model_path");
        }
        if (core_count_ < 1 || core_count_ > 16 || threads_per_instance_ < 1 ||
            threads_per_instance_ > 8 || queue_depth_ < 2 || queue_depth_ > 32 ||
            warmup_iterations_ < 0 || warmup_iterations_ > 100 || output_pool_depth < 2 ||
            preview_disparity_max_ <= preview_disparity_min_ || preview_depth_min_m_ < 0.0 ||
            preview_depth_max_m_ <= preview_depth_min_m_ || preview_depth_roi_size_ < 1 ||
            preview_depth_roi_size_ > 31 || preview_depth_roi_size_ % 2 == 0 ||
            !std::isfinite(preview_depth_roi_min_valid_fraction_) ||
            preview_depth_roi_min_valid_fraction_ < 0.0 ||
            preview_depth_roi_min_valid_fraction_ > 1.0 ||
            (preview_visualization_ != "depth" && preview_visualization_ != "disparity"))
        {
            throw std::runtime_error("invalid LAS2 dispatcher parameters");
        }
        affinity_groups_ = split_affinity_groups(core_affinity_);
        if (static_cast<int>(affinity_groups_.size()) != core_count_)
        {
            throw std::runtime_error("core_affinity must contain one group per LAS2 instance");
        }

        const S32 sys_result = SYS_Init();
        if (sys_result != 0)
        {
            throw std::runtime_error("SYS_Init failed for LAS2: " + std::to_string(sys_result));
        }
        sys_initialized_ = true;
        const S32 vb_result = VB_Init();
        if (vb_result != 0)
        {
            (void)SYS_Exit();
            sys_initialized_ = false;
            throw std::runtime_error("VB_Init failed for LAS2: " + std::to_string(vb_result));
        }
        vb_initialized_ = true;

        stereos_.reserve(static_cast<std::size_t>(core_count_));
        for (const auto& affinity : affinity_groups_)
        {
            auto stereo = std::make_unique<las2::ExternalStereo>();
            if (!stereo->initialize(calibration_file_.c_str(), 1, threads_per_instance_,
                    affinity.c_str(), model_path_.c_str()))
            {
                throw std::runtime_error(
                    std::string("LAS2 initialize failed: ") + stereo->last_error());
            }
            stereos_.push_back(std::move(stereo));
        }
        if (stereos_.empty() || !stereos_.front()->depth_camera_calibration(depth_camera_))
        {
            throw std::runtime_error("LAS2 depth calibration unavailable");
        }
        if (depth_camera_.width <= 0 || depth_camera_.height <= 0)
        {
            throw std::runtime_error("LAS2 returned invalid depth geometry");
        }
        if (preview_depth_roi_size_ > depth_camera_.width ||
            preview_depth_roi_size_ > depth_camera_.height)
        {
            throw std::runtime_error("preview_depth_roi_size exceeds LAS2 depth geometry");
        }
        create_output_pool(output_pool_depth);
        if (publish_preview_ && preview_center_depth_overlay_)
        {
            center_hud_ = std::make_unique<Nv12CenterHud>(
                preview_output_pool_, output_export_limit_ + 2, preview_width_, preview_height_);
        }

        for (int index = 0; index < core_count_; ++index)
        {
            const auto started = std::chrono::steady_clock::now();
            auto& stereo = *stereos_[static_cast<std::size_t>(index)];
            if (!stereo.warmup(warmup_iterations_))
            {
                throw std::runtime_error("LAS2 warmup failed for instance " +
                    std::to_string(index) + ": " + stereo.last_error());
            }
            const double elapsed_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - started)
                                            .count();
            RCLCPP_INFO(get_logger(), "LAS2 warmup: instance=%d iterations=%d total=%.2fms", index,
                warmup_iterations_, elapsed_ms);
        }

        const auto data_qos = offline_lossless_
            ? rclcpp::QoS(offline_window + core_count_).reliable()
            : rclcpp::SensorDataQoS().keep_last(8);
        output_pub_ = create_publisher<OutputFrame>(output_topic_, data_qos);
        const auto timing_topic =
            declare_parameter<std::string>("preprocess_timing_topic", "/las2/preprocess_gpu_ms");
        if (!timing_topic.empty())
        {
            timing_pub_ = create_publisher<std_msgs::msg::Float64>(
                timing_topic, rclcpp::SensorDataQoS().keep_last(64));
        }
        if (publish_preview_)
        {
            preview_pub_ =
                create_publisher<InputFrame>(preview_topic_, rclcpp::SensorDataQoS().keep_last(4));
        }
        rclcpp::SubscriptionOptions input_options;
        if (offline_lossless_)
        {
            offline_input_group_ =
                create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            offline_ack_group_ =
                create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            input_options.callback_group = offline_input_group_;
            auto ack_options = rclcpp::SubscriptionOptions();
            ack_options.callback_group = offline_ack_group_;
            const auto ack_qos = rclcpp::QoS(256).reliable().transient_local();
            offline_error_pub_ =
                create_publisher<std_msgs::msg::String>(offline_ack_topic_, ack_qos);
            if (offline_expected_frames_ > 0)
                offline_complete_pub_ =
                    create_publisher<std_msgs::msg::String>(offline_complete_topic_, ack_qos);
            offline_ack_sub_ = create_subscription<std_msgs::msg::String>(offline_ack_topic_,
                ack_qos, std::bind(&RobotLas2Node::offline_ack, this, std::placeholders::_1),
                ack_options);
            worker_completed_.resize(static_cast<std::size_t>(core_count_), 0);
        }
        input_sub_ = create_subscription<InputFrame>(input_topic_,
            offline_lossless_ ? data_qos : rclcpp::SensorDataQoS(),
            std::bind(&RobotLas2Node::input_callback, this, std::placeholders::_1), input_options);
        if (pose_gate_)
        {
            pose_sub_ = create_subscription<nav_msgs::msg::Odometry>(pose_gate_topic_,
                rclcpp::SensorDataQoS().keep_last(64),
                std::bind(&RobotLas2Node::pose_callback, this, std::placeholders::_1));
            RCLCPP_INFO(get_logger(), "LAS2 pose gate: topic=%s retained_images=%d latest-only",
                pose_gate_topic_.c_str(), pose_gate_image_depth);
        }

        stopping_.store(false);
        workers_.reserve(static_cast<std::size_t>(core_count_));
        for (int index = 0; index < core_count_; ++index)
        {
            workers_.emplace_back(&RobotLas2Node::worker_loop, this, index);
        }
        if (!ready_topic_.empty())
        {
            const auto ready_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
            ready_pub_ = create_publisher<std_msgs::msg::Bool>(ready_topic_, ready_qos);
            std_msgs::msg::Bool ready;
            ready.data = true;
            ready_pub_->publish(ready);
        }
        RCLCPP_INFO(get_logger(),
            "LAS2 ready: input=%s output=%s instances=%d threads=%d warmup=%d affinity=%s "
            "depth=%dx%d queue=%d preview=%s visualization=%s range=[%.3f, %.3f] "
            "center_roi=%s size=%dx%d min_valid=%.2f Z=m",
            input_topic_.c_str(), output_topic_.c_str(), core_count_, threads_per_instance_,
            warmup_iterations_, core_affinity_.c_str(), depth_camera_.width, depth_camera_.height,
            queue_depth_, publish_preview_ ? preview_topic_.c_str() : "off",
            preview_visualization_.c_str(),
            preview_visualization_ == "depth" ? preview_depth_min_m_ : preview_disparity_min_,
            preview_visualization_ == "depth" ? preview_depth_max_m_ : preview_disparity_max_,
            publish_preview_ && preview_center_depth_overlay_ ? "on" : "off",
            preview_depth_roi_size_, preview_depth_roi_size_,
            preview_depth_roi_min_valid_fraction_);
        if (offline_lossless_)
            RCLCPP_INFO(get_logger(),
                "LAS2 offline lossless: window=%d instances=%d threads=%d ack=%s "
                "output_pool=%d; pose gate bypassed, exact FIFO dispatch",
                offline_window, core_count_, threads_per_instance_, offline_ack_topic_.c_str(),
                output_pool_depth);
    }

    ~RobotLas2Node() override
    {
        stop_workers();
        write_offline_summary();
        // Release all OpenCL references before returning quarantined buffers.
        center_hud_.reset();
        {
            std::lock_guard<std::mutex> lock(output_mutex_);
            while (!output_handles_.empty())
            {
                const OutputHandles handles = output_handles_.front();
                (void)VB_Unexport(handles.depth);
                (void)VB_Unexport(handles.rgb);
                output_handles_.pop_front();
            }
        }
        {
            std::lock_guard<std::mutex> lock(preview_mutex_);
            while (!preview_handles_.empty())
            {
                (void)VB_Unexport(preview_handles_.front());
                preview_handles_.pop_front();
            }
            while (!preview_quarantined_handles_.empty())
            {
                (void)VB_ReleaseBuffer(preview_quarantined_handles_.front());
                preview_quarantined_handles_.pop_front();
            }
        }
        if (depth_output_pool_ != 0)
        {
            (void)VB_DestroyPool(depth_output_pool_);
            depth_output_pool_ = 0;
        }
        if (rgb_output_pool_ != 0)
        {
            (void)VB_DestroyPool(rgb_output_pool_);
            rgb_output_pool_ = 0;
        }
        if (preview_output_pool_ != 0)
        {
            (void)VB_DestroyPool(preview_output_pool_);
            preview_output_pool_ = 0;
        }
        shutdown_stereos();
        stereos_.clear();
        if (vb_initialized_)
        {
            (void)VB_Exit();
            vb_initialized_ = false;
        }
        if (sys_initialized_)
        {
            (void)SYS_Exit();
            sys_initialized_ = false;
        }
    }

private:
    void fail_offline(const std::string& reason) noexcept
    {
        if (!offline_lossless_ || offline_failed_.exchange(true))
            return;
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            offline_error_ = reason;
        }
        queue_cv_.notify_all();
        try
        {
            std_msgs::msg::String message;
            message.data = "error:LAS2 offline: " + reason;
            offline_error_pub_->publish(message);
            RCLCPP_ERROR(get_logger(), "%s", message.data.c_str());
        }
        catch (...)
        {
        }
    }

    void offline_ack(std_msgs::msg::String::ConstSharedPtr message)
    {
        if (offline_failed_.load() || stopping_.load())
            return;
        try
        {
            if (message->data.rfind("error:", 0) == 0)
                throw std::runtime_error("consumer " + message->data);
            const auto key = parse_offline_ack(message->data);
            OutputHandles handles;
            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                handles = offline_queue_->acknowledge(key);
                ++offline_exports_releasing_;
            }
            const auto depth_result = VB_Unexport(handles.depth);
            const auto rgb_result = VB_Unexport(handles.rgb);
            if (depth_result != 0 || rgb_result != 0)
                throw std::runtime_error("VB_Unexport failed after exact fusion ACK");
            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                --offline_exports_releasing_;
            }
            queue_cv_.notify_all();
            maybe_complete_offline();
        }
        catch (const std::exception& error)
        {
            fail_offline(error.what());
        }
    }

    void maybe_complete_offline()
    {
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            if (offline_failed_.load() || stopping_.load() || offline_complete_sent_ ||
                !offline_queue_->complete(offline_expected_frames_, offline_exports_releasing_))
                return;
            offline_complete_sent_ = true;
        }
        std_msgs::msg::String message;
        message.data = "complete";
        offline_complete_pub_->publish(message);
        RCLCPP_INFO(get_logger(),
            "LAS2 offline drained: %" PRIu64 " expected identities, all imports/exports released",
            offline_expected_frames_);
    }

    OfflineKey offline_key(const InputFrame& frame) const
    {
        if (frame.header.stamp.sec < 0 || frame.header.stamp.nanosec >= 1000000000U ||
            frame.frame_index > static_cast<std::uint32_t>(std::numeric_limits<int>::max()))
            throw std::runtime_error("offline image identity outside SDK source range");
        const auto stamp = stamp_ns(frame.header.stamp);
        if (stamp <= 0)
            throw std::runtime_error("offline image timestamp must be positive");
        return {frame.frame_index, stamp};
    }

    void retain_offline_input(const InputFrame& frame)
    {
        const auto key = offline_key(frame);
        std::unique_lock<std::mutex> lock(queue_mutex_);
        offline_queue_->validate_next(key);
        if (offline_expected_frames_ > 0 &&
            offline_queue_->stats().accepted >= offline_expected_frames_)
            throw std::runtime_error("received more offline targets than offline_expected_frames");
        const auto deadline = OfflineQueue::Clock::now() + offline_timeout_;
        while (!offline_queue_->has_credit())
        {
            if (stopping_.load() || offline_failed_.load())
                throw std::runtime_error("offline input interrupted");
            if (queue_cv_.wait_until(lock, deadline) == std::cv_status::timeout &&
                !offline_queue_->has_credit())
                throw std::runtime_error("timeout waiting for local fusion-ACK window credit");
        }
        if (stopping_.load() || offline_failed_.load())
            return;
        // Import only after credit. The independent ACK callback group can
        // release a credit while this input callback waits with metadata only.
        auto item = std::make_shared<WorkItem>();
        item->descriptor = frame;
        item->input = ImportedBuffer(frame.token);
        offline_queue_->admit(key, std::move(item));
        lock.unlock();
        queue_cv_.notify_one();
    }

    void offline_worker_loop(int worker_index)
    {
        try
        {
            for (;;)
            {
                std::shared_ptr<WorkItem> item;
                {
                    std::unique_lock<std::mutex> lock(queue_mutex_);
                    while (!stopping_.load() && !offline_failed_.load() && !offline_queue_->ready())
                    {
                        const auto oldest = offline_queue_->oldest_admission();
                        if (oldest)
                        {
                            const auto deadline = *oldest + offline_timeout_;
                            if (OfflineQueue::Clock::now() >= deadline)
                                throw std::runtime_error(
                                    "timeout waiting for offline inference/fusion ACK");
                            queue_cv_.wait_until(lock, deadline);
                        }
                        else
                            queue_cv_.wait(lock);
                    }
                    if (stopping_.load() || offline_failed_.load())
                        return;
                    item = std::move(offline_queue_->take()->item);
                }
                const auto key = offline_key(item->descriptor);
                process_item(*item, worker_index);
                item->input.release_checked();
                item.reset();
                {
                    std::lock_guard<std::mutex> lock(queue_mutex_);
                    offline_queue_->released_input(key);
                    ++worker_completed_.at(static_cast<std::size_t>(worker_index));
                }
                maybe_complete_offline();
            }
        }
        catch (const std::exception& error)
        {
            fail_offline(error.what());
        }
        catch (...)
        {
            fail_offline("unknown inference worker failure");
        }
    }

    UL output_buffer(UL pool)
    {
        if (!offline_lossless_)
            return VB_GetBuffer(pool, 0);
        const auto deadline = OfflineQueue::Clock::now() + offline_timeout_;
        while (!stopping_.load() && !offline_failed_.load())
        {
            const auto handle = VB_GetBuffer(pool, 100);
            if (handle)
                return handle;
            if (OfflineQueue::Clock::now() >= deadline)
                throw std::runtime_error("timeout waiting for offline output VB buffer");
        }
        throw std::runtime_error("offline output allocation interrupted");
    }

    void write_offline_summary() noexcept
    {
        if (!offline_queue_ || offline_summary_file_.empty())
            return;
        try
        {
            const auto stats = offline_final_stats_;
            const bool complete = !offline_failed_.load() && stats.accepted > 0 &&
                stats.accepted == stats.acknowledged && stats.outstanding == 0 &&
                stats.inputs_in_use == 0 && offline_exports_releasing_ == 0 &&
                (offline_expected_frames_ == 0 || offline_complete_sent_);
            std::ofstream output(offline_summary_file_, std::ios::out);
            if (!output)
                throw std::runtime_error("cannot open offline LAS2 summary");
            output << "{\"schema\":\"SPACEMIT_ROBOT_LAS2_LOSSLESS_SUMMARY_V1\",\"complete\":"
                    << (complete ? "true" : "false")
                    << ",\"failed\":" << (offline_failed_.load() ? "true" : "false")
                    << ",\"accepted\":" << stats.accepted << ",\"dispatched\":" << stats.dispatched
                    << ",\"published\":" << stats.published
                    << ",\"acknowledged\":" << stats.acknowledged << ",\"pending\":" << stats.pending
                    << ",\"active\":" << stats.active << ",\"waiting_ack\":" << stats.waiting_ack
                    << ",\"outstanding\":" << stats.outstanding
                    << ",\"inputs_in_use\":" << stats.inputs_in_use
                    << ",\"expected_frames\":" << offline_expected_frames_
                    << ",\"complete_sent\":" << (offline_complete_sent_ ? "true" : "false")
                    << ",\"maximum_active\":" << stats.maximum_active
                    << ",\"maximum_outstanding\":" << stats.maximum_outstanding
                    << ",\"worker_completed\":[";
            for (std::size_t index = 0; index < worker_completed_.size(); ++index)
                output << (index ? "," : "") << worker_completed_[index];
            output << "]}\n";
            output.close();
            if (!output)
                throw std::runtime_error("cannot write offline LAS2 summary");
        }
        catch (const std::exception& error)
        {
            RCLCPP_ERROR(get_logger(), "Offline summary failed: %s", error.what());
        }
    }

    void shutdown_stereos() noexcept
    {
        std::vector<std::thread> shutdown_threads;
        shutdown_threads.reserve(stereos_.size());
        for (auto& stereo : stereos_)
        {
            shutdown_threads.emplace_back(
                [stereo_ptr = stereo.get()]
                {
                    if (stereo_ptr != nullptr)
                        stereo_ptr->shutdown();
                });
        }
        for (auto& thread : shutdown_threads)
        {
            if (thread.joinable())
                thread.join();
        }
    }

    void create_output_pool(int depth)
    {
        VbPoolCfg config{};
        config.u32BufSize = static_cast<U32>(static_cast<std::size_t>(depth_camera_.width) *
            static_cast<std::size_t>(depth_camera_.height) * sizeof(float));
        config.u32BufCnt = static_cast<U32>(depth);
        config.eModId = MPP_ID_V2D;
        config.eRemapMode = VBUF_REMAP_MODE_NOCACHE;
        depth_output_pool_ = VB_CreatePool(&config);
        if (depth_output_pool_ == 0)
        {
            throw std::runtime_error("VB_CreatePool failed for LAS2 depth");
        }
        config.u32BufSize = static_cast<U32>(static_cast<std::size_t>(depth_camera_.width) *
            static_cast<std::size_t>(depth_camera_.height) * 3U);
        rgb_output_pool_ = VB_CreatePool(&config);
        if (rgb_output_pool_ == 0)
        {
            (void)VB_DestroyPool(depth_output_pool_);
            depth_output_pool_ = 0;
            throw std::runtime_error("VB_CreatePool failed for LAS2 RGB");
        }
        if (publish_preview_)
        {
            preview_width_ = depth_camera_.width * 2;
            preview_height_ = depth_camera_.height;
            if ((preview_width_ & 1) != 0 || (preview_height_ & 1) != 0)
            {
                throw std::runtime_error("LAS2 preview NV12 dimensions must be even");
            }
            /* Keep two allocation slots outside the exported descriptor
             * window.  A renderer may still own the oldest imported frame
             * when its export is retired; without this slack an otherwise
             * healthy asynchronous consumer causes a periodic pool miss. */
            config.u32BufCnt = static_cast<U32>(depth + 2);
            config.u32BufSize = static_cast<U32>(
                static_cast<std::size_t>(preview_width_) * preview_height_ * 3U / 2U);
            preview_output_pool_ = VB_CreatePool(&config);
            if (preview_output_pool_ == 0)
            {
                throw std::runtime_error("VB_CreatePool failed for LAS2 preview");
            }
        }
    }

    static std::int64_t stamp_ns(const builtin_interfaces::msg::Time& stamp)
    {
        return static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
            static_cast<std::int64_t>(stamp.nanosec);
    }

    void pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr& message)
    {
        if (!message)
        {
            return;
        }

        const auto& position = message->pose.pose.position;
        const auto& orientation = message->pose.pose.orientation;
        const double orientation_norm = orientation.x * orientation.x +
            orientation.y * orientation.y + orientation.z * orientation.z +
            orientation.w * orientation.w;
        if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
            !std::isfinite(position.z) || !std::isfinite(orientation_norm) ||
            orientation_norm < 1.0e-12)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "dropping invalid image-time VIO pose from LAS2 gate");
            return;
        }

        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            if (stopping_.load() || !pose_gate_)
            {
                return;
            }
            pose_gate_->push_pose(stamp_ns(message->header.stamp));
        }
        queue_cv_.notify_one();
    }

    void input_callback(const InputFrame::ConstSharedPtr& message)
    {
        if (!message || message->pixel_format != MPP_PIXEL_FORMAT_NV12 || message->width == 0 ||
            message->height == 0 || message->plane_num < 2 ||
            message->plane_stride[0] < message->width || message->plane_stride[1] < message->width)
        {
            if (offline_lossless_)
            {
                fail_offline("invalid full-frame NV12 descriptor");
                return;
            }
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000, "dropping invalid full-frame NV12 descriptor");
            return;
        }
        try
        {
            if (offline_lossless_)
            {
                retain_offline_input(*message);
                return;
            }
            auto item = std::make_shared<WorkItem>();
            item->descriptor = *message;
            item->input = ImportedBuffer(message->token);
            std::lock_guard<std::mutex> lock(queue_mutex_);
            if (stopping_.load())
                return;
            if (pose_gate_)
            {
                pose_gate_->push_image(stamp_ns(message->header.stamp), std::move(item));
                dropped_inputs_.store(pose_gate_->dropped_images());
            }
            else
            {
                while (static_cast<int>(queue_.size()) >= queue_depth_)
                {
                    queue_.pop_front();
                    ++dropped_inputs_;
                }
                queue_.push_back(std::move(item));
            }
        }
        catch (const std::exception& error)
        {
            if (offline_lossless_)
            {
                fail_offline(error.what());
                return;
            }
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000, "failed to retain LAS2 input: %s", error.what());
            return;
        }
        queue_cv_.notify_one();
    }

    void worker_loop(int worker_index)
    {
        if (offline_lossless_)
        {
            offline_worker_loop(worker_index);
            return;
        }
        (void)worker_index;
        while (true)
        {
            std::shared_ptr<WorkItem> item;
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                queue_cv_.wait(lock,
                    [&]
                    {
                        return stopping_.load() ||
                            (pose_gate_ ? pose_gate_->ready() : !queue_.empty());
                    });
                if (stopping_.load())
                    return;
                if (pose_gate_)
                {
                    auto selected = pose_gate_->take_latest();
                    if (!selected)
                    {
                        continue;
                    }
                    item = std::move(*selected);
                    dropped_inputs_.store(pose_gate_->dropped_images());
                }
                else
                {
                    item = std::move(queue_.front());
                    queue_.pop_front();
                }
            }
            RCLCPP_DEBUG(get_logger(), "LAS2 dispatch frame=%u stamp=%" PRId64 " pose_gated=%d",
                item->descriptor.frame_index, stamp_ns(item->descriptor.header.stamp),
                pose_gate_ != nullptr);
            process_item(*item, worker_index);
        }
    }

    void process_item(const WorkItem& item, int worker_index)
    {
        const auto start = std::chrono::steady_clock::now();
        las2::Nv12DmaFrame input;
        input.dma_fd = item.input.dma_fd();
        input.width = static_cast<int>(item.descriptor.width);
        input.height = static_cast<int>(item.descriptor.height);
        input.y_stride = static_cast<int>(item.descriptor.plane_stride[0]);
        input.uv_stride = static_cast<int>(item.descriptor.plane_stride[1]);
        input.total_size = item.descriptor.total_size;
        input.timestamp_ns =
            static_cast<std::uint64_t>(item.descriptor.header.stamp.sec) * 1000000000ULL +
            item.descriptor.header.stamp.nanosec;
        input.frame_id = static_cast<int>(item.descriptor.frame_index);

        las2::Frame result;
        auto& stereo = *stereos_[static_cast<std::size_t>(worker_index)];
        if (!stereo.process(input, result))
        {
            if (offline_lossless_)
                throw std::runtime_error(
                    std::string("LAS2 processing failed: ") + stereo.last_error());
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "LAS2 processing failed: %s",
                stereo.last_error());
            return;
        }
        if (offline_lossless_ && result.frame_id != input.frame_id)
            throw std::runtime_error("LAS2 result source index differs from dispatched input");
        if (!result.depth.data || result.depth.width != depth_camera_.width ||
            result.depth.height != depth_camera_.height ||
            result.depth.stride < result.depth.width * static_cast<int>(sizeof(float)))
        {
            if (offline_lossless_)
                throw std::runtime_error("LAS2 returned invalid depth view");
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000, "LAS2 returned invalid depth view");
            return;
        }
        if (!result.rgb.data || result.rgb.width != result.depth.width ||
            result.rgb.height != result.depth.height || result.rgb.stride < result.rgb.width * 3)
        {
            if (offline_lossless_)
                throw std::runtime_error("LAS2 returned invalid aligned RGB view");
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000, "LAS2 returned invalid aligned RGB view");
            return;
        }
        CenterDepthSample center;
        if (publish_preview_ && preview_center_depth_overlay_)
        {
            // Measure the actual metric depth, regardless of whether preview
            // colors represent depth or disparity.
            center = sample_depth_roi(result.depth.data, result.depth.width, result.depth.height,
                result.depth.stride, result.depth.width / 2, result.depth.height / 2,
                preview_depth_roi_size_, preview_depth_roi_min_valid_fraction_);
        }
        PublishDepth(item.descriptor, result);
        // One event per successful stereo completion across all workers.
        // GPU profiling excludes CPU staging, queue waits, inference and preview.
        if (timing_pub_)
        {
            std_msgs::msg::Float64 timing;
            timing.data = stereo.last_opencl_ms();
            timing_pub_->publish(timing);
        }
        if (publish_preview_)
        {
            publish_gpu_preview(item.descriptor, stereo, center);
        }
        const double elapsed_ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start)
                .count();
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "LAS2 frame=%u %.2fms queue=%zu dropped=%" PRIu64, item.descriptor.frame_index,
            elapsed_ms, queue_size(), dropped_inputs_.load());
    }

    std::size_t queue_size() const
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        return offline_queue_ ? offline_queue_->stats().pending
                                : (pose_gate_ ? pose_gate_->size() : queue_.size());
    }

    void PublishDepth(const InputFrame& input, const las2::Frame& result)
    {
        /* Revoke the oldest export before asking the finite output pool for
         * another block.  Reclaiming only after VB_GetBuffer() would deadlock
         * a pool whose block count equals output_export_limit_: the ninth
         * frame cannot be allocated, so the post-publish reclaim is never
         * reached.  Existing importers retain their own VB reference after
         * unexport, while new consumers can only observe the bounded window.
         */
        if (!offline_lossless_)
        {
            std::lock_guard<std::mutex> lock(output_mutex_);
            while (output_handles_.size() >= output_export_limit_)
            {
                const OutputHandles handles = output_handles_.front();
                output_handles_.pop_front();
                const S32 depth_result = VB_Unexport(handles.depth);
                const S32 rgb_result = VB_Unexport(handles.rgb);
                if (depth_result != 0 || rgb_result != 0)
                {
                    RCLCPP_WARN(
                        get_logger(), "VB_Unexport failed while reclaiming LAS2 output handles");
                }
            }
        }

        OutputAllocation allocation;
        const UL depth_buffer = allocation.depth = output_buffer(depth_output_pool_);
        if (depth_buffer == 0)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "LAS2 depth output pool exhausted; dropping frame");
            return;
        }
        const UL rgb_buffer = allocation.rgb = output_buffer(rgb_output_pool_);
        if (rgb_buffer == 0)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000, "LAS2 RGB output pool exhausted; dropping frame");
            return;
        }
        void* depth_address = nullptr;
        void* rgb_address = nullptr;
        const std::size_t depth_row_bytes =
            static_cast<std::size_t>(result.depth.width) * sizeof(float);
        const std::size_t rgb_row_bytes = static_cast<std::size_t>(result.rgb.width) * 3U;
        if (VB_GetVirAddr(depth_buffer, &depth_address) != 0 || depth_address == nullptr ||
            VB_GetVirAddr(rgb_buffer, &rgb_address) != 0 || rgb_address == nullptr)
        {
            if (offline_lossless_)
                throw std::runtime_error("VB_GetVirAddr failed for offline LAS2 depth/RGB");
            RCLCPP_WARN(get_logger(), "VB_GetVirAddr failed for LAS2 depth");
            return;
        }
        for (int row = 0; row < result.depth.height; ++row)
        {
            const auto* source = reinterpret_cast<const std::uint8_t*>(result.depth.data) +
                static_cast<std::size_t>(row) * result.depth.stride;
            auto* destination = static_cast<std::uint8_t*>(depth_address) +
                static_cast<std::size_t>(row) * depth_row_bytes;
            std::memcpy(destination, source, depth_row_bytes);
            const auto* rgb_source =
                result.rgb.data + static_cast<std::size_t>(row) * result.rgb.stride;
            auto* rgb_destination = static_cast<std::uint8_t*>(rgb_address) +
                static_cast<std::size_t>(row) * rgb_row_bytes;
            std::memcpy(rgb_destination, rgb_source, rgb_row_bytes);
        }

        const auto update_frame =
            [&](UL buffer, UL pool, void* address, std::size_t row_bytes, int height)
        {
            VideoFrameInfo frame{};
            frame.eFrameType = FRAME_TYPE_COMMON;
            frame.eModId = MPP_ID_V2D;
            frame.ulPoolId = pool;
            frame.ulBufferId = buffer;
            frame.u32Idx = input.frame_index;
            frame.stCommFrameInfo.u32Width = static_cast<U32>(result.depth.width);
            frame.stCommFrameInfo.u32Height = static_cast<U32>(height);
            frame.stCommFrameInfo.u32Align = 1;
            frame.stCommFrameInfo.ePixelFormat = MPP_PIXEL_FORMAT_UNKNOWN;
            frame.stVFrame.u64PTS = input.pts;
            frame.stVFrame.u32PlaneNum = 1;
            frame.stVFrame.u32TotalSize = static_cast<U32>(row_bytes * height);
            frame.stVFrame.u32PlaneStride[0] = static_cast<U32>(row_bytes);
            frame.stVFrame.u32PlaneSize[0] = frame.stVFrame.u32TotalSize;
            frame.stVFrame.u32PlaneSizeValid[0] = frame.stVFrame.u32TotalSize;
            frame.stVFrame.ulPlaneVirAddr[0] = reinterpret_cast<UL>(address);
            const auto status = VB_UpdateBufferFrameInfo(buffer, &frame);
            if (offline_lossless_ && status != 0)
                throw std::runtime_error("VB_UpdateBufferFrameInfo failed for offline LAS2 output");
            return frame.stVFrame.u32TotalSize;
        };
        const U32 depth_total_size = update_frame(
            depth_buffer, depth_output_pool_, depth_address, depth_row_bytes, result.depth.height);
        const U32 rgb_total_size = update_frame(
            rgb_buffer, rgb_output_pool_, rgb_address, rgb_row_bytes, result.rgb.height);

        U64 depth_token = 0;
        U64 rgb_token = 0;
        allocation.depth_exported = VB_Export(depth_buffer, &depth_token) == 0;
        if (allocation.depth_exported)
            allocation.rgb_exported = VB_Export(rgb_buffer, &rgb_token) == 0;
        if (!allocation.depth_exported || !allocation.rgb_exported ||
            (offline_lossless_ && (depth_token == 0 || rgb_token == 0)))
        {
            if (offline_lossless_)
                throw std::runtime_error("VB_Export failed for offline LAS2 depth/RGB");
            RCLCPP_WARN(get_logger(), "VB_Export failed for LAS2 depth/RGB");
            return;
        }
        OutputFrame message;
        message.header = input.header;
        message.version = 2;
        message.token = depth_token;
        message.pool_id = depth_output_pool_;
        message.buffer_id = depth_buffer;
        message.frame_index = input.frame_index;
        message.width = static_cast<std::uint32_t>(result.depth.width);
        message.height = static_cast<std::uint32_t>(result.depth.height);
        message.stride_bytes = static_cast<std::uint32_t>(depth_row_bytes);
        message.encoding = kDepthEncodingFloat32Meters;
        message.pts = input.pts;
        message.total_size = depth_total_size;
        message.rgb_token = rgb_token;
        message.rgb_pool_id = rgb_output_pool_;
        message.rgb_buffer_id = rgb_buffer;
        message.rgb_width = static_cast<std::uint32_t>(result.rgb.width);
        message.rgb_height = static_cast<std::uint32_t>(result.rgb.height);
        message.rgb_stride_bytes = static_cast<std::uint32_t>(rgb_row_bytes);
        message.rgb_encoding = kRgbEncodingRgb8;
        message.rgb_total_size = rgb_total_size;
        message.fx = depth_camera_.fx;
        message.fy = depth_camera_.fy;
        message.cx = depth_camera_.cx;
        message.cy = depth_camera_.cy;
        for (std::size_t index = 0; index < message.left_from_depth.size(); ++index)
        {
            message.left_from_depth[index] = depth_camera_.left_from_depth[index];
        }
        if (offline_lossless_)
        {
            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                if (offline_failed_.load() || stopping_.load())
                    throw std::runtime_error("offline publication interrupted");
                offline_queue_->published(offline_key(input), {depth_buffer, rgb_buffer});
                allocation.committed = true;
            }
            output_pub_->publish(std::move(message));
            queue_cv_.notify_all();
        }
        else
        {
            std::lock_guard<std::mutex> lock(output_mutex_);
            /* Export tokens are transport credentials. VB_Unexport takes the
             * local buffer handle that was passed to VB_Export. */
            output_handles_.push_back({depth_buffer, rgb_buffer});
            allocation.committed = true;
            output_pub_->publish(std::move(message));
            while (output_handles_.size() > output_export_limit_)
            {
                const OutputHandles handles = output_handles_.front();
                (void)VB_Unexport(handles.depth);
                (void)VB_Unexport(handles.rgb);
                output_handles_.pop_front();
            }
        }
    }

    void publish_gpu_preview(
        const InputFrame& input, las2::ExternalStereo& stereo, const CenterDepthSample& center)
    {
        if (preview_hud_failed_.load())
            return;
        {
            std::lock_guard<std::mutex> lock(preview_mutex_);
            while (preview_handles_.size() >= output_export_limit_)
            {
                (void)VB_Unexport(preview_handles_.front());
                preview_handles_.pop_front();
            }
        }
        const UL output = VB_GetBuffer(preview_output_pool_, 0);
        if (output == 0)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000, "LAS2 preview pool exhausted; dropping frame");
            return;
        }

        void* address = nullptr;
        S32 dma_fd = -1;
        if (VB_GetVirAddr(output, &address) != 0 || address == nullptr ||
            VB_GetDmaBufFd(output, &dma_fd) != 0 || dma_fd < 0)
        {
            (void)VB_ReleaseBuffer(output);
            RCLCPP_WARN(get_logger(), "failed to map LAS2 NV12 preview output");
            return;
        }

        const int y_stride_bytes = preview_width_;
        const int uv_stride_bytes = preview_width_;
        const std::uint64_t y_size = static_cast<std::uint64_t>(y_stride_bytes) * preview_height_;
        const std::uint64_t uv_size =
            static_cast<std::uint64_t>(uv_stride_bytes) * (preview_height_ / 2);
        const std::uint64_t total_size = y_size + uv_size;
        const bool depth = preview_visualization_ == "depth";
        const auto mode = depth ? las2::PreviewMode::Depth : las2::PreviewMode::Disparity;
        const float minimum =
            static_cast<float>(depth ? preview_depth_min_m_ : preview_disparity_min_);
        const float maximum =
            static_cast<float>(depth ? preview_depth_max_m_ : preview_disparity_max_);
        const las2::Nv12PreviewTarget target{dma_fd, preview_width_, preview_height_,
            y_stride_bytes, uv_stride_bytes, total_size, minimum, maximum};
        if (!stereo.render_nv12_preview(target, mode, minimum, maximum))
        {
            (void)VB_ReleaseBuffer(output);
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "LAS2 GPU %s preview failed: %s",
                preview_visualization_.c_str(), stereo.last_error());
            return;
        }

        if (preview_center_depth_overlay_)
        {
            // The SDK completes its GPU write before this GPU acquire. The
            // output belongs only to this preview pool; input pixels are intact.
            std::string error;
            if (!center_hud_->render(preview_output_pool_, output, dma_fd, y_stride_bytes,
                    uv_stride_bytes, total_size, center, error))
            {
                preview_hud_failed_.store(true);
                // Driver synchronization may be uncertain. Keep this VB lease
                // until the HUD is destroyed; never publish or recycle it.
                std::lock_guard<std::mutex> lock(preview_mutex_);
                preview_quarantined_handles_.push_back(output);
                RCLCPP_ERROR(
                    get_logger(), "LAS2 GPU center HUD failed; preview stopped: %s", error.c_str());
                return;
            }
        }

        VideoFrameInfo frame{};
        frame.eFrameType = FRAME_TYPE_COMMON;
        frame.eModId = MPP_ID_V2D;
        frame.ulPoolId = preview_output_pool_;
        frame.ulBufferId = output;
        frame.u32Idx = input.frame_index;
        frame.stCommFrameInfo.u32Width = preview_width_;
        frame.stCommFrameInfo.u32Height = preview_height_;
        frame.stCommFrameInfo.u32Align = 1;
        frame.stCommFrameInfo.ePixelFormat = MPP_PIXEL_FORMAT_NV12;
        frame.stVFrame.u64PTS = input.pts;
        frame.stVFrame.u32PlaneNum = 2;
        frame.stVFrame.u32TotalSize = static_cast<U32>(total_size);
        frame.stVFrame.u32PlaneStride[0] = static_cast<U32>(y_stride_bytes);
        frame.stVFrame.u32PlaneStride[1] = static_cast<U32>(uv_stride_bytes);
        frame.stVFrame.u32PlaneSize[0] = static_cast<U32>(y_size);
        frame.stVFrame.u32PlaneSize[1] = static_cast<U32>(uv_size);
        frame.stVFrame.u32PlaneSizeValid[0] = frame.stVFrame.u32PlaneSize[0];
        frame.stVFrame.u32PlaneSizeValid[1] = frame.stVFrame.u32PlaneSize[1];
        frame.stVFrame.ulPlaneVirAddr[0] = reinterpret_cast<UL>(address);
        (void)VB_UpdateBufferFrameInfo(output, &frame);

        U64 token = 0;
        if (VB_Export(output, &token) != 0)
        {
            (void)VB_ReleaseBuffer(output);
            RCLCPP_WARN(get_logger(), "VB_Export failed for LAS2 GPU preview");
            return;
        }
        InputFrame message;
        message.header = input.header;
        message.version = 1;
        message.token = token;
        message.pool_id = preview_output_pool_;
        message.buffer_id = output;
        message.frame_index = input.frame_index;
        message.width = preview_width_;
        message.height = preview_height_;
        message.align = 1;
        message.pixel_format = MPP_PIXEL_FORMAT_NV12;
        message.pts = input.pts;
        message.total_size = frame.stVFrame.u32TotalSize;
        message.plane_num = 2;
        message.plane_stride[0] = static_cast<std::uint32_t>(y_stride_bytes);
        message.plane_stride[1] = static_cast<std::uint32_t>(uv_stride_bytes);
        message.plane_size[0] = frame.stVFrame.u32PlaneSize[0];
        message.plane_size[1] = frame.stVFrame.u32PlaneSize[1];
        message.plane_size_valid[0] = message.plane_size[0];
        message.plane_size_valid[1] = message.plane_size[1];
        {
            std::lock_guard<std::mutex> lock(preview_mutex_);
            preview_handles_.push_back(output);
            (void)VB_ReleaseBuffer(output);
            preview_pub_->publish(std::move(message));
        }
    }

    void stop_workers()
    {
        const bool already_stopping = stopping_.exchange(true);
        if (!already_stopping)
            queue_cv_.notify_all();
        for (auto& worker : workers_)
        {
            if (worker.joinable())
                worker.join();
        }
        workers_.clear();
        std::lock_guard<std::mutex> lock(queue_mutex_);
        queue_.clear();
        if (pose_gate_)
        {
            pose_gate_->clear();
        }
        if (offline_queue_)
        {
            offline_final_stats_ = offline_queue_->stats();
            for (const auto& handles : offline_queue_->clear())
            {
                (void)VB_Unexport(handles.depth);
                (void)VB_Unexport(handles.rgb);
            }
        }
    }

    std::string input_topic_;
    std::string output_topic_;
    std::string ready_topic_;
    std::string preview_topic_;
    std::string calibration_file_;
    std::string model_path_;
    std::string core_affinity_;
    std::string pose_gate_topic_;
    bool offline_lossless_{false};
    std::atomic<bool> offline_failed_{false};
    std::string offline_ack_topic_, offline_summary_file_, offline_error_, offline_complete_topic_;
    std::uint64_t offline_expected_frames_{0};
    std::size_t offline_exports_releasing_{0};
    bool offline_complete_sent_{false};
    OfflineQueue::Clock::duration offline_timeout_{std::chrono::seconds(180)};
    std::unique_ptr<OfflineQueue> offline_queue_;
    OfflineQueue::Stats offline_final_stats_;
    std::vector<std::uint64_t> worker_completed_;
    rclcpp::CallbackGroup::SharedPtr offline_input_group_, offline_ack_group_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr offline_error_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr offline_complete_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr offline_ack_sub_;
    std::vector<std::string> affinity_groups_;
    int core_count_{4};
    int threads_per_instance_{2};
    int warmup_iterations_{2};
    int queue_depth_{4};
    bool publish_preview_{true};
    bool preview_center_depth_overlay_{false};
    int preview_depth_roi_size_{9};
    double preview_depth_roi_min_valid_fraction_{0.5};
    std::string preview_visualization_{"disparity"};
    double preview_disparity_min_{0.0};
    double preview_disparity_max_{128.0};
    double preview_depth_min_m_{0.0};
    double preview_depth_max_m_{3.0};
    int preview_width_{0};
    int preview_height_{0};
    std::size_t output_export_limit_{8};

    std::vector<std::unique_ptr<las2::ExternalStereo>> stereos_;
    las2::DepthCameraCalibration depth_camera_;
    UL depth_output_pool_{0};
    UL rgb_output_pool_{0};
    UL preview_output_pool_{0};
    std::deque<OutputHandles> output_handles_;
    std::mutex output_mutex_;
    std::deque<UL> preview_handles_;
    std::deque<UL> preview_quarantined_handles_;
    std::mutex preview_mutex_;
    std::unique_ptr<Nv12CenterHud> center_hud_;
    std::atomic<bool> preview_hud_failed_{false};
    bool sys_initialized_{false};
    bool vb_initialized_{false};

    rclcpp::Subscription<InputFrame>::SharedPtr input_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
    rclcpp::Publisher<OutputFrame>::SharedPtr output_pub_;
    rclcpp::Publisher<InputFrame>::SharedPtr preview_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr timing_pub_;
    mutable std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::deque<std::shared_ptr<WorkItem>> queue_;
    std::unique_ptr<PoseGatedQueue<std::shared_ptr<WorkItem>>> pose_gate_;
    std::vector<std::thread> workers_;
    std::atomic<bool> stopping_{false};
    std::atomic<std::uint64_t> dropped_inputs_{0};
};

} // namespace las2

RCLCPP_COMPONENTS_REGISTER_NODE(las2::RobotLas2Node)
