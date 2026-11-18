/*
 * SPDX-FileCopyrightText: The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <thread>
#include <vector>

namespace aidl {
namespace android {
namespace hardware {
namespace vibrator {

/*
 * Streams int8 PCM to Awinic haptic_hv chips through the TikTap shared
 * memory ring exported by the kernel driver at /proc/tiktap_buf.
 *
 * The ring is made of AW_TIKTAP_MMAP_BUF_SUM buffers. Userspace fills a
 * buffer and marks it valid, the kernel worker started by TIKTAP_RTP_MODE
 * copies valid buffers into the chip RTP FIFO in order, marks them finished
 * and stops at the first buffer that is not valid.
 */
class TiktapDevice {
  public:
    static constexpr uint32_t kPlayRateHz = 24000;

    // Time from play() until the kernel worker has pushed the first buffer
    // into the FIFO: stop of the previous playback, RTP mode switch, boost
    // and trim setup. Measured ~7 ms on aw8693x.
    static constexpr uint32_t kStartupLatencyMs = 10;

    // Total time until a stream of the given size has been played out
    static uint32_t playbackMs(size_t samples);

    TiktapDevice() = default;
    ~TiktapDevice();

    // Opens and maps the TikTap ring, can be retried until it succeeds since
    // the kernel module may be probed after the HAL starts.
    bool open();
    bool isPresent() const { return mBuf != nullptr; }

    // LRA resonant frequency in Hz * 10 as reported by the driver, 0 if unknown
    uint32_t getF0() const { return mF0; }

    // Starts streaming asynchronously, stopping any previous stream
    int play(std::vector<int8_t> pcm);
    void stop();

  private:
    void stopLocked();
    void streamLoop(std::vector<int8_t> pcm);

    int mFd = -1;
    uint8_t* mBuf = nullptr;
    size_t mMapSize = 0;
    uint32_t mF0 = 0;

    std::mutex mLock;
    std::thread mThread;
    std::atomic<bool> mAbort{false};

    // When the current stream is expected to be fully played out. Stopping
    // after that is skipped: TIKTAP_STOP_MODE waits up to ~25 ms for the
    // chip to enter standby and would cut the brake of a finished effect.
    std::chrono::steady_clock::time_point mPlaybackEnd;
};

}  // namespace vibrator
}  // namespace hardware
}  // namespace android
}  // namespace aidl
