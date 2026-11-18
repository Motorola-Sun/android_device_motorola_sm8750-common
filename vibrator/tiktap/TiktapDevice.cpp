/*
 * SPDX-FileCopyrightText: The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "vibrator.moto-tiktap"

#include "TiktapDevice.h"

#include <android-base/logging.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstring>

namespace aidl {
namespace android {
namespace hardware {
namespace vibrator {

namespace {

constexpr char kTiktapPath[] = "/proc/tiktap_buf";

// Must match haptic_hv.h in the kernel driver
constexpr size_t kBufCount = 16;      // AW_TIKTAP_MMAP_BUF_SUM
constexpr size_t kBufDataSize = 1000; // AW_TIKTAP_MMAP_BUF_SIZE
constexpr int kMmapPageOrder = 2;     // AW_TIKTAP_MMAP_PAGE_ORDER

constexpr uint8_t kBufDataValid = 0x55;
constexpr uint8_t kBufDataFinished = 0xaa;

constexpr unsigned int kTiktapIoctlGroup = 0xff;
constexpr unsigned int kTiktapGetF0 = _IO(kTiktapIoctlGroup, 0x01);
constexpr unsigned int kTiktapSettingGain = _IO(kTiktapIoctlGroup, 0x04);
constexpr unsigned int kTiktapRtpMode = _IO(kTiktapIoctlGroup, 0x09);
constexpr unsigned int kTiktapStopMode = _IO(kTiktapIoctlGroup, 0x0b);

constexpr unsigned long kMaxGain = 0x80;

#pragma pack(push, 4)
struct mmap_buf_format {
    uint8_t status;
    uint8_t bit;
    int16_t length;
    uint64_t kernel_next;  // kernel pointer, never dereferenced here
    uint64_t user_next;
    uint8_t reg_addr;
    int8_t data[kBufDataSize];
};
#pragma pack(pop)

static_assert(sizeof(mmap_buf_format) == 1024, "mmap_buf_format layout mismatch");
static_assert(offsetof(mmap_buf_format, data) == 21, "mmap_buf_format layout mismatch");

// The kernel worker gives up after 800 ms without new data, don't wait longer
constexpr auto kBufWaitTimeout = std::chrono::milliseconds(800);
constexpr useconds_t kPollIntervalUs = 1000;

}  // namespace

bool TiktapDevice::open() {
    if (isPresent()) return true;

    mFd = TEMP_FAILURE_RETRY(::open(kTiktapPath, O_RDWR | O_CLOEXEC));
    if (mFd < 0) {
        PLOG(ERROR) << "Failed to open " << kTiktapPath;
        return false;
    }

    mMapSize = static_cast<size_t>(sysconf(_SC_PAGESIZE)) << kMmapPageOrder;
    if (mMapSize < kBufCount * sizeof(mmap_buf_format)) {
        LOG(ERROR) << "Unexpected tiktap mmap size " << mMapSize;
        close(mFd);
        mFd = -1;
        return false;
    }

    void* buf = mmap(nullptr, mMapSize, PROT_READ | PROT_WRITE, MAP_SHARED, mFd, 0);
    if (buf == MAP_FAILED) {
        PLOG(ERROR) << "Failed to mmap " << kTiktapPath;
        close(mFd);
        mFd = -1;
        return false;
    }
    mBuf = static_cast<uint8_t*>(buf);

    unsigned int f0 = 0;
    if (ioctl(mFd, kTiktapGetF0, &f0) == 0) {
        mF0 = f0;
    } else {
        PLOG(WARNING) << "Failed to read f0";
    }

    LOG(INFO) << "TikTap ready, f0 = " << mF0;
    return true;
}

TiktapDevice::~TiktapDevice() {
    stop();
    if (mBuf) munmap(mBuf, mMapSize);
    if (mFd >= 0) close(mFd);
}

int TiktapDevice::play(std::vector<int8_t> pcm) {
    if (!isPresent()) return -ENODEV;
    if (pcm.empty()) return -EINVAL;

    std::lock_guard<std::mutex> lock(mLock);
    stopLocked();

    // Effects are scaled in the PCM data, always play them at full gain.
    if (ioctl(mFd, kTiktapSettingGain, kMaxGain) < 0) {
        PLOG(WARNING) << "Failed to set gain";
    }

    // Invalidates every buffer and queues the kernel worker, which waits up
    // to 100 ms for the first buffer to become valid.
    if (ioctl(mFd, kTiktapRtpMode) < 0) {
        PLOG(ERROR) << "Failed to enter RTP mode";
        return -errno;
    }

    mPlaybackEnd = std::chrono::steady_clock::now() +
                   std::chrono::milliseconds(playbackMs(pcm.size()));
    mAbort = false;
    mThread = std::thread(&TiktapDevice::streamLoop, this, std::move(pcm));

    return 0;
}

void TiktapDevice::stop() {
    std::lock_guard<std::mutex> lock(mLock);
    stopLocked();
}

uint32_t TiktapDevice::playbackMs(size_t samples) {
    return kStartupLatencyMs +
           static_cast<uint32_t>((samples * 1000 + kPlayRateHz - 1) / kPlayRateHz);
}

void TiktapDevice::stopLocked() {
    if (!mThread.joinable()) return;

    mAbort = true;
    mThread.join();

    // Let finished effects ring out with their brake instead of forcing the
    // chip to standby, which also blocks for up to ~25 ms.
    if (std::chrono::steady_clock::now() < mPlaybackEnd) {
        if (ioctl(mFd, kTiktapStopMode) < 0) {
            PLOG(ERROR) << "Failed to stop";
        }
    }
}

void TiktapDevice::streamLoop(std::vector<int8_t> pcm) {
    auto* bufs = reinterpret_cast<volatile mmap_buf_format*>(mBuf);
    const size_t chunks = (pcm.size() + kBufDataSize - 1) / kBufDataSize;

    auto fill = [&](size_t chunk) {
        volatile mmap_buf_format* buf = &bufs[chunk % kBufCount];
        size_t offset = chunk * kBufDataSize;
        size_t len = std::min(kBufDataSize, pcm.size() - offset);

        memcpy(const_cast<int8_t*>(buf->data), pcm.data() + offset, len);
        buf->length = static_cast<int16_t>(len);
    };

    auto publish = [&](size_t chunk) {
        // Data must be visible before the kernel sees the valid status
        std::atomic_thread_fence(std::memory_order_release);
        bufs[chunk % kBufCount].status = kBufDataValid;
    };

    // The kernel stops at the first buffer that isn't valid, so prefill as
    // much of the ring as possible and publish the first buffer last, which
    // is the one the kernel worker waits for before starting playback.
    size_t prefill = std::min(chunks, kBufCount);
    for (size_t i = 0; i < prefill; i++) fill(i);
    for (size_t i = 1; i < prefill; i++) publish(i);
    publish(0);

    for (size_t chunk = prefill; chunk < chunks; chunk++) {
        volatile mmap_buf_format* buf = &bufs[chunk % kBufCount];
        auto start = std::chrono::steady_clock::now();

        while (buf->status != kBufDataFinished) {
            if (mAbort) return;
            if (std::chrono::steady_clock::now() - start > kBufWaitTimeout) {
                LOG(ERROR) << "Timed out waiting for buffer " << chunk % kBufCount
                           << ", status = " << static_cast<int>(buf->status);
                return;
            }
            usleep(kPollIntervalUs);
        }
        std::atomic_thread_fence(std::memory_order_acquire);

        fill(chunk);
        publish(chunk);
    }
}

}  // namespace vibrator
}  // namespace hardware
}  // namespace android
}  // namespace aidl
