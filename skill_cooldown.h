#ifndef _BALL_GAME_SKILL_COOLDOWN_H_
#define _BALL_GAME_SKILL_COOLDOWN_H_

#include <chrono>

namespace gibbon {

class SkillCooldown {
    using ClockT = std::chrono::steady_clock;
    using TimePointT = std::chrono::time_point<ClockT>;
    using DeltaT = std::chrono::steady_clock::duration;

public:
    SkillCooldown(int64_t cooldown, int initCount = 0, int maxCount = 1)
        : mCooldown(std::chrono::milliseconds(cooldown)),
          mMaxCount(maxCount),
          mCount(initCount),
          mLastTP(ClockT::now()) {
    }
    ~SkillCooldown() = default;

    bool Skill() {
        TimePointT now = ClockT::now();
        DeltaT delta = now - mLastTP;
        while (delta > mCooldown) {
            delta -= mCooldown;
            ++mCount;
        }
        if (mCount >= mMaxCount) {
            mCount = mMaxCount;
            delta = std::chrono::milliseconds(0);
        }
        if (mCount > 0) {
            --mCount;
            mLastTP = now - delta;
            return true;
        } else {
            return false;
        }
    }

private:
    const DeltaT mCooldown;
    const int mMaxCount;
    int mCount;
    TimePointT mLastTP;
};

}  // namespace gibbon

#endif  //_BALL_GAME_SKILL_COOLDOWN_H_