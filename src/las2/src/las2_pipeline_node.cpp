/**
 * @file las2_pipeline_node.cpp
 * @brief Run independent LAS2 workers for packed four-camera stereo pairs.
 */
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <stereo_pipeline.h>
#include <memory>
#include <mpp_msgs/msg/mpp_video_frame_desc.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <stdexcept>
#include <sys/sys_api.h>
#include <sys/vb_api.h>
#include <sys/vb_type.h>
#include <thread>
#include <common/msg/dma_depth_frame_desc.hpp>
#include <utility>

namespace las2
{
using Input = mpp_msgs::msg::MppVideoFrameDesc;
using Output = common::msg::DmaDepthFrameDesc;
using Clock = std::chrono::steady_clock;

struct VbLease
{
    VbLease() = default;
    VbLease(const VbLease&) = delete;
    VbLease& operator=(const VbLease&) = delete;
    UL handle = 0;
    U64 token = 0;
    S32 fd = -1;
    void* data = nullptr;
    ~VbLease()
    {
        if (token)
            (void)VB_Unexport(handle);
        if (handle)
            (void)VB_ReleaseBuffer(handle);
    }
    void import(U64 source)
    {
        if (!source || VB_Import(source, &handle) || !handle || VB_GetDmaBufFd(handle, &fd) ||
            fd < 0)
            throw std::runtime_error("VB input import failed");
    }
    void allocate(UL pool)
    {
        handle = VB_GetBuffer(pool, 0);
        if (!handle || VB_GetDmaBufFd(handle, &fd) || fd < 0 || VB_GetVirAddr(handle, &data) ||
            !data)
            throw std::runtime_error("VB output allocation failed");
    }
    void export_frame(UL pool, int width, int height, int stride, const Input& source)
    {
        VideoFrameInfo info{};
        info.eFrameType = FRAME_TYPE_COMMON;
        info.eModId = MPP_ID_V2D;
        info.ulPoolId = pool;
        info.ulBufferId = handle;
        info.u32Idx = source.frame_index;
        info.stCommFrameInfo.u32Width = width;
        info.stCommFrameInfo.u32Height = height;
        info.stCommFrameInfo.u32Align = 1;
        info.stCommFrameInfo.ePixelFormat = MPP_PIXEL_FORMAT_UNKNOWN;
        info.stVFrame.u64PTS = source.pts;
        info.stVFrame.u32PlaneNum = 1;
        info.stVFrame.u32TotalSize = stride * height;
        info.stVFrame.u32PlaneStride[0] = stride;
        info.stVFrame.u32PlaneSize[0] = stride * height;
        info.stVFrame.u32PlaneSizeValid[0] = stride * height;
        info.stVFrame.ulPlaneVirAddr[0] = reinterpret_cast<UL>(data);
        if (VB_UpdateBufferFrameInfo(handle, &info) || VB_Export(handle, &token) || !token)
            throw std::runtime_error("VB output export failed");
    }
};
struct InputWork
{
    Input descriptor;
    VbLease lease;
    Clock::time_point received;
};
struct PreparedWork
{
    Clock::time_point prepared_at;
    Input descriptor;
    las2::PreparedStereoFrame lease;
    Clock::time_point received;
};
struct OutputWork
{
    VbLease depth, rgb;
};

class PipelineNode : public rclcpp::Node
{
public:
    PipelineNode() : Node("las2_pipeline")
    {
        const auto model = declare_parameter<std::string>("model_path", "");
        const auto calibration = declare_parameter<std::string>("calibration_file", "");
        const auto affinity = declare_parameter<std::string>("core_affinity", "8;9");
        const int threads = declare_parameter<int>("threads_per_instance", 2);
        const auto input_topic =
            declare_parameter<std::string>("input_topic", "/camera/quad/frame_desc");
        const auto output_topic =
            declare_parameter<std::string>("output_topic", "/las2/front/depth_desc");
        const auto ready_topic = declare_parameter<std::string>("ready_topic", "/las2/front/ready");
        overlap_ = declare_parameter<bool>("overlap", true);
        prefer_latest_ = declare_parameter<bool>("prefer_latest", false);
        export_limit_ = declare_parameter<int>("export_retention", 8);
        const int warmup = declare_parameter<int>("warmup_iterations", 2);
        config_.frame_width = declare_parameter<int>("frame_width", 1088);
        config_.frame_height = declare_parameter<int>("frame_height", 5120);
        config_.eye_width = declare_parameter<int>("eye_width", 1088);
        config_.eye_height = declare_parameter<int>("eye_height", 1280);
        config_.crop_x = 0;
        config_.swap_lr = false;
        config_.produce_rgb888 = false;
        config_.vertical_omni = true;
        config_.pipeline_slots = overlap_ && prefer_latest_ ? 3 : 2;
        config_.left_camera = declare_parameter<int>("left_camera", 0);
        config_.right_camera = declare_parameter<int>("right_camera", 1);
        config_.rectified_width = declare_parameter<int>("rectified_width", 384);
        config_.rectified_height = declare_parameter<int>("rectified_height", 288);
        config_.crop_bottom_ratio = declare_parameter<double>("crop_bottom_ratio", 0.0);
        config_.horizontal_fov_degrees = declare_parameter<double>("horizontal_fov_degrees", 100.0);
        if (export_limit_ < 2 || export_limit_ > 32 || model.empty() || calibration.empty())
            throw std::runtime_error("invalid model/calibration or export retention");
        try
        {
            if (SYS_Init())
                throw std::runtime_error("SYS_Init failed");
            sys_ready_ = true;
            if (VB_Init())
                throw std::runtime_error("VB_Init failed");
            vb_ready_ = true;
            stereo_ = std::make_unique<las2::ExternalStereo>();
            if (!stereo_->initialize(
                    calibration.c_str(), 1, threads, affinity.c_str(), model.c_str(), config_))
                throw std::runtime_error(stereo_->last_error());
            if (!stereo_->depth_camera_calibration(calibration_))
                throw std::runtime_error("missing depth calibration");
            depth_pool_ = create_pool(calibration_.width * calibration_.height * 4);
            rgb_pool_ = create_pool(calibration_.width * calibration_.height * 3);
            if (!stereo_->warmup(warmup))
                throw std::runtime_error(stereo_->last_error());
            output_ = create_publisher<Output>(output_topic, rclcpp::SensorDataQoS().keep_last(2));
            input_ = create_subscription<Input>(input_topic, rclcpp::SensorDataQoS().keep_last(1),
                [this](Input::ConstSharedPtr frame) { receive(*frame); });
            prepare_thread_ = std::thread([this] { prepare_loop(); });
            infer_thread_ = std::thread([this] { infer_loop(); });
            ready_ = create_publisher<std_msgs::msg::Bool>(
                ready_topic, rclcpp::QoS(1).reliable().transient_local());
            std_msgs::msg::Bool ready;
            ready.data = true;
            ready_->publish(ready);
            RCLCPP_INFO(get_logger(),
                "ready pair=%d/%d affinity=%s overlap=%d direct_vb_output=true",
                config_.left_camera, config_.right_camera, affinity.c_str(), overlap_);
        }
        catch (...)
        {
            shutdown();
            throw;
        }
    }
    ~PipelineNode() override { shutdown(); }

private:
    UL create_pool(int bytes)
    {
        VbPoolCfg config{};
        config.u32BufSize = bytes;
        config.u32BufCnt = export_limit_ + 2;
        config.eModId = MPP_ID_V2D;
        config.eRemapMode = VBUF_REMAP_MODE_NOCACHE;
        const UL pool = VB_CreatePool(&config);
        if (!pool)
            throw std::runtime_error("VB_CreatePool failed");
        return pool;
    }
    void receive(const Input& frame)
    {
        if (stop_)
            return;
        if (frame.version != 1 || !frame.token || frame.plane_num < 2 ||
            frame.pixel_format != static_cast<std::uint32_t>(MPP_PIXEL_FORMAT_NV12) ||
            frame.width != static_cast<unsigned>(config_.frame_width) ||
            frame.height != static_cast<unsigned>(config_.frame_height))
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000, "invalid packed NV12 descriptor");
            return;
        }
        try
        {
            auto work = std::make_unique<InputWork>();
            work->descriptor = frame;
            work->received = Clock::now();
            ++input_count_;
            const auto source_ns =
                std::int64_t(frame.header.stamp.sec) * 1000000000LL + frame.header.stamp.nanosec;
            const auto input_age = now().nanoseconds() - source_ns;
            if (input_age > 0)
                input_age_ns_ += input_age;
            const auto callback_begin = Clock::now();
            work->lease.import(frame.token);
            const auto imported = Clock::now();
            import_ns_ +=
                std::chrono::duration_cast<std::chrono::nanoseconds>(imported - callback_begin)
                    .count();
            {
                std::lock_guard<std::mutex> guard(mutex_);
                if (pending_input_)
                    ++dropped_;
                pending_input_ = std::move(work);
            }
            receive_ns_ +=
                std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - callback_begin)
                    .count();
            cv_.notify_all();
        }
        catch (const std::exception& e)
        {
            ++errors_;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", e.what());
        }
    }
    void prepare_loop()
    {
        while (true)
        {
            std::unique_ptr<InputWork> input;
            {
                std::unique_lock<std::mutex> guard(mutex_);
                cv_.wait(guard,
                    [this]
                    {
                        return stop_ ||
                            (pending_input_ &&
                                ((overlap_ && prefer_latest_) ||
                                    (!pending_prepared_ && (overlap_ || !infer_busy_))));
                    });
                if (stop_)
                    return;
                input = std::move(pending_input_);
            }
            try
            {
                const auto& d = input->descriptor;
                las2::Nv12DmaFrame frame;
                frame.dma_fd = input->lease.fd;
                frame.width = d.width;
                frame.height = d.height;
                frame.y_stride = d.plane_stride[0];
                frame.uv_stride = d.plane_stride[1];
                frame.total_size = d.total_size;
                frame.frame_id = d.frame_index;
                frame.timestamp_ns =
                    static_cast<std::uint64_t>(d.header.stamp.sec) * 1000000000ULL +
                    d.header.stamp.nanosec;
                auto output = std::make_unique<PreparedWork>();
                output->descriptor = d;
                output->received = input->received;
                output->lease = stereo_->prepare(frame);
                output->prepared_at = Clock::now();
                ++prepare_count_;
                input.reset(); // GPU completed: release decoder VB before inference.
                if (!output->lease)
                {
                    ++dropped_;
                    continue;
                }
                {
                    std::lock_guard<std::mutex> guard(mutex_);
                    // Keep the queued frame available until its fresher replacement is
                    // ready; the inference-owned slot is never modified.
                    if (pending_prepared_)
                        ++dropped_;
                    pending_prepared_ = std::move(output);
                }
                cv_.notify_all();
            }
            catch (const std::exception& e)
            {
                ++errors_;
                RCLCPP_ERROR(get_logger(), "prepare: %s", e.what());
            }
        }
    }
    void infer_loop()
    {
        auto epoch = Clock::now();
        std::uint64_t count = 0;
        double pre = 0, infer = 0, post = 0, latency = 0, prepared_wait = 0;
        std::uint64_t previous_inputs = 0, previous_prepares = 0, previous_age = 0,
                        previous_import = 0, previous_receive = 0;
        while (true)
        {
            std::unique_ptr<PreparedWork> work;
            {
                std::unique_lock<std::mutex> guard(mutex_);
                cv_.wait(guard, [this] { return stop_ || pending_prepared_; });
                if (stop_)
                    return;
                work = std::move(pending_prepared_);
                infer_busy_ = true;
            }
            cv_.notify_all();
            try
            {
                prepared_wait +=
                    std::chrono::duration<double, std::milli>(Clock::now() - work->prepared_at)
                        .count();
                while (exports_.size() >= static_cast<std::size_t>(export_limit_))
                    exports_.pop_front();
                auto output = std::make_unique<OutputWork>();
                output->depth.allocate(depth_pool_);
                output->rgb.allocate(rgb_pool_);
                const int width = calibration_.width, height = calibration_.height;
                las2::StereoOutputTarget target;
                target.depth_fd = output->depth.fd;
                target.rgb_fd = output->rgb.fd;
                target.depth_data = static_cast<float*>(output->depth.data);
                target.rgb_data = static_cast<std::uint8_t*>(output->rgb.data);
                target.depth_stride = width * 4;
                target.rgb_stride = width * 3;
                target.depth_size = width * height * 4;
                target.rgb_size = width * height * 3;
                const auto result = stereo_->infer_prepared(work->lease, target);
                output->depth.export_frame(depth_pool_, width, height, width * 4, work->descriptor);
                output->rgb.export_frame(rgb_pool_, width, height, width * 3, work->descriptor);
                Output message;
                message.header = work->descriptor.header;
                message.version = 2;
                message.frame_index = work->descriptor.frame_index;
                message.pts = work->descriptor.pts;
                message.token = output->depth.token;
                message.pool_id = depth_pool_;
                message.buffer_id = output->depth.handle;
                message.width = width;
                message.height = height;
                message.stride_bytes = width * 4;
                message.encoding = 1;
                message.total_size = width * height * 4;
                message.rgb_token = output->rgb.token;
                message.rgb_pool_id = rgb_pool_;
                message.rgb_buffer_id = output->rgb.handle;
                message.rgb_width = width;
                message.rgb_height = height;
                message.rgb_stride_bytes = width * 3;
                message.rgb_encoding = 1;
                message.rgb_total_size = width * height * 3;
                message.fx = calibration_.fx;
                message.fy = calibration_.fy;
                message.cx = calibration_.cx;
                message.cy = calibration_.cy;
                message.left_from_depth = calibration_.left_from_depth;
                exports_.push_back(std::move(output));
                output_->publish(message);
                pre += result.preprocess_ms;
                infer += result.inference_ms;
                post += result.postprocess_ms;
                latency += std::chrono::duration<double, std::milli>(Clock::now() - work->received)
                                .count();
                if (++count == 50)
                {
                    const double seconds =
                        std::chrono::duration<double>(Clock::now() - epoch).count();
                    const auto inputs = input_count_.load(), preparations = prepare_count_.load(),
                                ages = input_age_ns_.load();
                    const auto received = inputs - previous_inputs;
                    const auto imported_ns = import_ns_.load(), callback_ns = receive_ns_.load();
                    RCLCPP_INFO(get_logger(),
                        "STAGES input_fps=%.3f prepare_fps=%.3f input_source_age_ms=%.3f "
                        "prepared_wait_ms=%.3f vb_import_ms=%.3f callback_ms=%.3f",
                        received / seconds, (preparations - previous_prepares) / seconds,
                        received ? (ages - previous_age) / (1e6 * received) : 0.0,
                        prepared_wait / count,
                        received ? (imported_ns - previous_import) / (1e6 * received) : 0.0,
                        received ? (callback_ns - previous_receive) / (1e6 * received) : 0.0);
                    previous_import = imported_ns;
                    previous_receive = callback_ns;
                    previous_inputs = inputs;
                    previous_prepares = preparations;
                    previous_age = ages;
                    prepared_wait = 0;
                    RCLCPP_INFO(get_logger(),
                        "PERF fps=%.3f pre_ms=%.3f infer_ms=%.3f post_ms=%.3f "
                        "receive_to_output_ms=%.3f dropped=%lu errors=%lu frame=%u",
                        count / seconds, pre / count, infer / count, post / count, latency / count,
                        static_cast<unsigned long>(dropped_.load()),
                        static_cast<unsigned long>(errors_.load()), message.frame_index);
                    epoch = Clock::now();
                    count = 0;
                    pre = infer = post = latency = 0;
                }
            }
            catch (const std::exception& e)
            {
                ++errors_;
                RCLCPP_ERROR(get_logger(), "infer: %s", e.what());
            }
            work.reset();
            {
                std::lock_guard<std::mutex> guard(mutex_);
                infer_busy_ = false;
            }
            cv_.notify_all();
        }
    }
    void shutdown()
    {
        stop_ = true;
        cv_.notify_all();
        if (prepare_thread_.joinable())
            prepare_thread_.join();
        if (infer_thread_.joinable())
            infer_thread_.join();
        pending_input_.reset();
        pending_prepared_.reset();
        stereo_.reset();
        exports_.clear();
        if (depth_pool_)
        {
            (void)VB_DestroyPool(depth_pool_);
            depth_pool_ = 0;
        }
        if (rgb_pool_)
        {
            (void)VB_DestroyPool(rgb_pool_);
            rgb_pool_ = 0;
        }
        if (vb_ready_)
        {
            (void)VB_Exit();
            vb_ready_ = false;
        }
        if (sys_ready_)
        {
            (void)SYS_Exit();
            sys_ready_ = false;
        }
    }
    las2::ExternalStereoInputConfig config_;
    las2::DepthCameraCalibration calibration_;
    std::unique_ptr<las2::ExternalStereo> stereo_;
    UL depth_pool_ = 0, rgb_pool_ = 0;
    int export_limit_ = 8;
    bool sys_ready_ = false, vb_ready_ = false, overlap_ = true, infer_busy_ = false;
    bool prefer_latest_ = false;
    std::atomic<std::uint64_t> input_count_{0}, prepare_count_{0}, input_age_ns_{0}, import_ns_{0},
        receive_ns_{0};
    std::atomic<bool> stop_{false};
    std::atomic<std::uint64_t> dropped_{0}, errors_{0};
    std::mutex mutex_;
    std::condition_variable cv_;
    std::unique_ptr<InputWork> pending_input_;
    std::unique_ptr<PreparedWork> pending_prepared_;
    std::deque<std::unique_ptr<OutputWork>> exports_;
    std::thread prepare_thread_, infer_thread_;
    rclcpp::Subscription<Input>::SharedPtr input_;
    rclcpp::Publisher<Output>::SharedPtr output_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_;
};
} // namespace las2

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try
    {
        auto node = std::make_shared<las2::PipelineNode>();
        rclcpp::spin(node);
    }
    catch (const std::exception& error)
    {
        RCLCPP_ERROR(rclcpp::get_logger("las2_pipeline"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
