/*
 * SPDX-FileCopyrightText: The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "vibrator.moto-tiktap"

#include "Vibrator.h"

#include <android-base/file.h>
#include <android-base/logging.h>
#include <android/binder_enums.h>
#include <tiktap_effect.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>

namespace aidl {
namespace android {
namespace hardware {
namespace vibrator {

namespace {

constexpr char kLedPath[] = "/sys/class/leds/vibrator/";

constexpr uint32_t kPrimitiveIdMask = 0x8000;
constexpr uint8_t kMaxGain = 0x80;

constexpr int32_t kComposeDelayMaxMs = 1000;
constexpr int32_t kComposeSizeMax = 256;

// Only resample when the measured f0 is plausible
constexpr float kMaxF0Deviation = 0.15f;

bool writeNode(const std::string& node, const std::string& value) {
    if (!::android::base::WriteStringToFile(value, kLedPath + node)) {
        PLOG(ERROR) << "Failed to write " << value << " to " << kLedPath << node;
        return false;
    }
    return true;
}

float strengthScale(EffectStrength strength) {
    switch (strength) {
        case EffectStrength::LIGHT:
            return 0.6f;
        case EffectStrength::MEDIUM:
            return 1.0f;
        case EffectStrength::STRONG:
        default:
            return 1.0f;
    }
}

void appendScaled(std::vector<int8_t>* out, const std::vector<int8_t>& wave, float scale) {
    out->reserve(out->size() + wave.size());
    for (int8_t sample : wave) {
        out->push_back(static_cast<int8_t>(std::lround(sample * scale)));
    }
}

// Linear resampling so that a waveform designed for designF0 drives the LRA
// at its measured resonant frequency.
std::vector<int8_t> resample(const effect_stream* stream, float ratio) {
    std::vector<int8_t> out;
    size_t len = std::max<size_t>(1, std::lround(stream->length * ratio));

    out.reserve(len);
    for (size_t i = 0; i < len; i++) {
        float pos = i / ratio;
        size_t idx = static_cast<size_t>(pos);
        if (idx + 1 >= stream->length) {
            out.push_back(stream->data[stream->length - 1]);
            continue;
        }
        float frac = pos - idx;
        float value = stream->data[idx] * (1.0f - frac) + stream->data[idx + 1] * frac;
        out.push_back(static_cast<int8_t>(std::lround(value)));
    }

    return out;
}

}  // namespace

/*
 * LedDevice
 */

LedDevice::LedDevice() {
    mPresent = access((std::string(kLedPath) + "activate").c_str(), W_OK) == 0;
    if (!mPresent) PLOG(ERROR) << "LED vibrator not available";
}

int LedDevice::on(int32_t timeoutMs, uint8_t gain) {
    if (!mPresent) return -ENODEV;

    // duration picks the RAM waveform in the driver, gain must be set
    // before activate since the driver applies it when starting playback.
    if (!writeNode("state", "1") || !writeNode("duration", std::to_string(timeoutMs)) ||
        !writeNode("gain", std::to_string(gain)) || !writeNode("activate", "1")) {
        return -EIO;
    }

    return 0;
}

int LedDevice::off() {
    if (!mPresent) return -ENODEV;
    return writeNode("activate", "0") ? 0 : -EIO;
}

int LedDevice::setGain(uint8_t gain) {
    if (!mPresent) return -ENODEV;
    return writeNode("gain", std::to_string(gain)) ? 0 : -EIO;
}

/*
 * CallbackScheduler
 */

CallbackScheduler::CallbackScheduler() : mThread(&CallbackScheduler::loop, this) {}

CallbackScheduler::~CallbackScheduler() {
    {
        std::lock_guard<std::mutex> lock(mLock);
        mQuit = true;
    }
    mCv.notify_all();
    mThread.join();
}

void CallbackScheduler::schedule(const std::shared_ptr<IVibratorCallback>& callback,
                                 int32_t delayMs) {
    std::shared_ptr<IVibratorCallback> previous;
    {
        std::lock_guard<std::mutex> lock(mLock);
        previous = std::move(mCallback);
        mCallback = callback;
        mDeadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(delayMs);
    }
    mCv.notify_all();

    if (previous) previous->onComplete();
}

void CallbackScheduler::cancel() {
    std::shared_ptr<IVibratorCallback> previous;
    {
        std::lock_guard<std::mutex> lock(mLock);
        previous = std::move(mCallback);
        mCallback = nullptr;
    }
    mCv.notify_all();

    if (previous) previous->onComplete();
}

void CallbackScheduler::loop() {
    std::unique_lock<std::mutex> lock(mLock);

    while (!mQuit) {
        if (!mCallback) {
            mCv.wait(lock);
            continue;
        }

        if (mCv.wait_until(lock, mDeadline) == std::cv_status::timeout && mCallback &&
            std::chrono::steady_clock::now() >= mDeadline) {
            auto callback = std::move(mCallback);
            mCallback = nullptr;

            lock.unlock();
            if (!callback->onComplete().isOk()) LOG(ERROR) << "Failed to call onComplete";
            lock.lock();
        }
    }
}

/*
 * Vibrator
 */

Vibrator::Vibrator() : mGain(kMaxGain) {
    // Effects are advertised from the built-in tables, the TikTap device is
    // opened lazily as the kernel module may not be probed yet.
    loadWaveforms(1.0f);

    std::lock_guard<std::mutex> lock(mLock);
    ensureTiktap();
}

bool Vibrator::ensureTiktap() {
    if (mTiktap.isPresent()) return true;
    if (!mTiktap.open()) return false;

    float designF0 = get_effect_design_f0_x10();
    float realF0 = mTiktap.getF0();

    if (realF0 > 0 && std::fabs(realF0 - designF0) / designF0 <= kMaxF0Deviation) {
        LOG(INFO) << "Resampling waveforms designed for f0 " << designF0 << " to " << realF0;
        loadWaveforms(designF0 / realF0);
    } else {
        LOG(WARNING) << "Ignoring f0 " << realF0 << ", using waveforms as is";
    }

    return true;
}

void Vibrator::loadWaveforms(float ratio) {
    auto load = [&](uint32_t id) {
        const effect_stream* stream = get_effect_stream(id);
        if (!stream || stream->length == 0) return;

        if (stream->play_rate_hz != TiktapDevice::kPlayRateHz) {
            LOG(ERROR) << "Effect " << id << " has unsupported rate " << stream->play_rate_hz;
            return;
        }

        if (ratio == 1.0f) {
            mWaveforms[id].assign(stream->data, stream->data + stream->length);
        } else {
            mWaveforms[id] = resample(stream, ratio);
        }
    };

    for (auto effect : ndk::enum_range<Effect>()) {
        load(static_cast<uint32_t>(effect));
    }
    for (auto primitive : ndk::enum_range<CompositePrimitive>()) {
        if (primitive == CompositePrimitive::NOOP) continue;
        load(kPrimitiveIdMask | static_cast<uint32_t>(primitive));
    }
}

const std::vector<int8_t>* Vibrator::getWaveform(uint32_t id) const {
    auto it = mWaveforms.find(id);
    return it == mWaveforms.end() ? nullptr : &it->second;
}

int32_t Vibrator::durationMs(size_t samples) {
    return static_cast<int32_t>((samples * 1000 + TiktapDevice::kPlayRateHz - 1) /
                                TiktapDevice::kPlayRateHz);
}

void Vibrator::stopAll() {
    mTiktap.stop();
    if (mLedActive) {
        mLed.off();
        mLedActive = false;
    }
}

ndk::ScopedAStatus Vibrator::getCapabilities(int32_t* _aidl_return) {
    std::lock_guard<std::mutex> lock(mLock);

    *_aidl_return = IVibrator::CAP_ON_CALLBACK | IVibrator::CAP_AMPLITUDE_CONTROL;

    if (!mWaveforms.empty()) {
        *_aidl_return |= IVibrator::CAP_PERFORM_CALLBACK | IVibrator::CAP_COMPOSE_EFFECTS;
    }
    if (mTiktap.getF0() > 0) {
        *_aidl_return |= IVibrator::CAP_GET_RESONANT_FREQUENCY;
    }

    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Vibrator::off() {
    std::lock_guard<std::mutex> lock(mLock);

    stopAll();
    mCallbacks.cancel();

    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Vibrator::on(int32_t timeoutMs,
                                const std::shared_ptr<IVibratorCallback>& callback) {
    std::lock_guard<std::mutex> lock(mLock);

    if (timeoutMs <= 0) return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);

    stopAll();

    if (mLed.on(timeoutMs, mGain) != 0) {
        return ndk::ScopedAStatus::fromExceptionCode(EX_SERVICE_SPECIFIC);
    }
    mLedActive = true;

    if (callback) mCallbacks.schedule(callback, timeoutMs);

    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Vibrator::perform(Effect effect, EffectStrength strength,
                                     const std::shared_ptr<IVibratorCallback>& callback,
                                     int32_t* _aidl_return) {
    std::lock_guard<std::mutex> lock(mLock);

    const std::vector<int8_t>* wave = getWaveform(static_cast<uint32_t>(effect));
    if (!wave) return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);

    if (!ensureTiktap()) return ndk::ScopedAStatus::fromExceptionCode(EX_SERVICE_SPECIFIC);
    wave = getWaveform(static_cast<uint32_t>(effect));

    if (strength != EffectStrength::LIGHT && strength != EffectStrength::MEDIUM &&
        strength != EffectStrength::STRONG) {
        return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
    }

    std::vector<int8_t> pcm;
    appendScaled(&pcm, *wave, strengthScale(strength));

    stopAll();

    if (mTiktap.play(std::move(pcm)) != 0) {
        return ndk::ScopedAStatus::fromExceptionCode(EX_SERVICE_SPECIFIC);
    }

    *_aidl_return = TiktapDevice::playbackMs(wave->size());
    if (callback) mCallbacks.schedule(callback, *_aidl_return);

    LOG(DEBUG) << "Perform effect " << toString(effect) << " strength " << toString(strength)
               << " for " << *_aidl_return << " ms";

    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Vibrator::getSupportedEffects(std::vector<Effect>* _aidl_return) {
    std::lock_guard<std::mutex> lock(mLock);

    for (auto effect : ndk::enum_range<Effect>()) {
        if (getWaveform(static_cast<uint32_t>(effect))) _aidl_return->push_back(effect);
    }
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Vibrator::setAmplitude(float amplitude) {
    std::lock_guard<std::mutex> lock(mLock);

    if (amplitude <= 0.0f || amplitude > 1.0f) {
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }

    mGain = static_cast<uint8_t>(std::clamp<long>(std::lround(amplitude * kMaxGain), 1, kMaxGain));

    // Allow changing the amplitude of an ongoing vibration
    if (mLedActive && mLed.setGain(mGain) != 0) {
        return ndk::ScopedAStatus::fromExceptionCode(EX_SERVICE_SPECIFIC);
    }

    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Vibrator::setExternalControl(bool /*enabled*/) {
    return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
}

ndk::ScopedAStatus Vibrator::getCompositionDelayMax(int32_t* maxDelayMs) {
    *maxDelayMs = kComposeDelayMaxMs;
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Vibrator::getCompositionSizeMax(int32_t* maxSize) {
    *maxSize = kComposeSizeMax;
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Vibrator::getSupportedPrimitives(std::vector<CompositePrimitive>* supported) {
    std::lock_guard<std::mutex> lock(mLock);

    if (mWaveforms.empty()) return ndk::ScopedAStatus::ok();

    for (auto primitive : ndk::enum_range<CompositePrimitive>()) {
        if (primitive == CompositePrimitive::NOOP ||
            getWaveform(kPrimitiveIdMask | static_cast<uint32_t>(primitive))) {
            supported->push_back(primitive);
        }
    }
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Vibrator::getPrimitiveDuration(CompositePrimitive primitive,
                                                  int32_t* durationMs) {
    std::lock_guard<std::mutex> lock(mLock);

    if (primitive == CompositePrimitive::NOOP) {
        *durationMs = 0;
        return ndk::ScopedAStatus::ok();
    }

    const std::vector<int8_t>* wave =
            getWaveform(kPrimitiveIdMask | static_cast<uint32_t>(primitive));
    if (!wave) return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);

    *durationMs = Vibrator::durationMs(wave->size());
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Vibrator::compose(const std::vector<CompositeEffect>& composite,
                                     const std::shared_ptr<IVibratorCallback>& callback) {
    std::lock_guard<std::mutex> lock(mLock);

    if (composite.empty() || composite.size() > kComposeSizeMax) {
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }

    if (!ensureTiktap()) return ndk::ScopedAStatus::fromExceptionCode(EX_SERVICE_SPECIFIC);

    std::vector<int8_t> pcm;
    for (const auto& e : composite) {
        if (e.delayMs < 0 || e.delayMs > kComposeDelayMaxMs) {
            return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
        }
        if (e.scale < 0.0f || e.scale > 1.0f) {
            return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
        }

        const std::vector<int8_t>* wave = nullptr;
        if (e.primitive != CompositePrimitive::NOOP) {
            wave = getWaveform(kPrimitiveIdMask | static_cast<uint32_t>(e.primitive));
            if (!wave) return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
        }

        pcm.insert(pcm.end(), static_cast<size_t>(e.delayMs) * TiktapDevice::kPlayRateHz / 1000,
                   0);
        if (wave) appendScaled(&pcm, *wave, e.scale);
    }

    stopAll();

    int32_t duration = TiktapDevice::playbackMs(pcm.size());
    bool silent = std::all_of(pcm.begin(), pcm.end(), [](int8_t s) { return s == 0; });

    // A composition of NOOPs only has to wait for its delays
    if (!silent && mTiktap.play(std::move(pcm)) != 0) {
        return ndk::ScopedAStatus::fromExceptionCode(EX_SERVICE_SPECIFIC);
    }

    if (callback) mCallbacks.schedule(callback, duration);

    LOG(DEBUG) << "Compose " << composite.size() << " primitives for " << duration << " ms";

    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Vibrator::getSupportedAlwaysOnEffects(std::vector<Effect>* /*_aidl_return*/) {
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Vibrator::alwaysOnEnable(int32_t /*id*/, Effect /*effect*/,
                                            EffectStrength /*strength*/) {
    return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
}

ndk::ScopedAStatus Vibrator::alwaysOnDisable(int32_t /*id*/) {
    return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
}

ndk::ScopedAStatus Vibrator::getResonantFrequency(float* resonantFreqHz) {
    std::lock_guard<std::mutex> lock(mLock);

    if (mTiktap.getF0() == 0) {
        return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
    }

    *resonantFreqHz = mTiktap.getF0() / 10.0f;
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Vibrator::getQFactor(float* /*qFactor*/) {
    return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
}

ndk::ScopedAStatus Vibrator::getFrequencyResolution(float* /*freqResolutionHz*/) {
    return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
}

ndk::ScopedAStatus Vibrator::getFrequencyMinimum(float* /*freqMinimumHz*/) {
    return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
}

ndk::ScopedAStatus Vibrator::getBandwidthAmplitudeMap(std::vector<float>* /*_aidl_return*/) {
    return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
}

ndk::ScopedAStatus Vibrator::getPwlePrimitiveDurationMax(int32_t* /*durationMs*/) {
    return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
}

ndk::ScopedAStatus Vibrator::getPwleCompositionSizeMax(int32_t* /*maxSize*/) {
    return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
}

ndk::ScopedAStatus Vibrator::getSupportedBraking(std::vector<Braking>* /*supported*/) {
    return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
}

ndk::ScopedAStatus Vibrator::composePwle(const std::vector<PrimitivePwle>& /*composite*/,
                                         const std::shared_ptr<IVibratorCallback>& /*callback*/) {
    return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
}

}  // namespace vibrator
}  // namespace hardware
}  // namespace android
}  // namespace aidl
