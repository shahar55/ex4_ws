#include "team1.hpp"
#include <argos3/core/utility/math/angles.h>
#include <iostream>
#include <algorithm>
#include <limits>
#include <mutex>

namespace argos {

namespace {

constexpr Real kMaxSpeed = 10.0f;
constexpr Real kAvoidRadius = 0.18f;
constexpr Real kCollisionTrigger = 0.14f;
constexpr Real kCollisionClear = 0.10f;

constexpr Real kFoodMergeTolerance = 0.20f;
constexpr Real kTargetReachedTolerance = 0.10f;
constexpr Real kBaseReachedTolerance = 0.18f;

constexpr uint32_t kTargetLostTimeout = 35;
constexpr uint32_t kMemoryStaleTimeout = 60;
constexpr uint32_t kLockStaleTimeout = 60;

constexpr uint32_t kIdleToBlockTimeout = 140;
constexpr uint32_t kBlockEnemyMaxTimer = 120;
constexpr uint32_t kPostDropTimer = 25;

constexpr std::size_t kTrafficMaxPoints = 6;

struct FoodEntry {
    CVector3 Pos;
    bool Locked;
    const Controller1* Owner;
    uint32_t LastSeen;
    uint32_t LockedAt;
    Real LockDist;
};


static Real Dist(const CVector3& a, const CVector3& b) {
    return (a - b).Length();
}

static bool CloseEnough(const CVector3& a, const CVector3& b, Real tol) {
    return Dist(a, b) <= tol;
}

class SharedMemory {
public:
    void UpdateFoods(const std::vector<CVector3>& seen, uint32_t step) {
        std::lock_guard<std::mutex> lock(Mtx);
        CleanupUnsafe(step);
        for(const auto& p : seen) {
            std::size_t best = Foods.size();
            Real bestD = std::numeric_limits<Real>::max();
            for(std::size_t i = 0; i < Foods.size(); ++i) {
                Real d = Dist(Foods[i].Pos, p);
                if(d < bestD) {
                    bestD = d;
                    best = i;
                }
            }
            if(best < Foods.size() && bestD <= kFoodMergeTolerance) {
                Foods[best].Pos = (Foods[best].Pos + p) * 0.5f;
                Foods[best].LastSeen = step;
            } else {
                Foods.push_back({p, false, nullptr, step, 0, 0.0f});
            }
        }
    }

    bool HasUnlocked(uint32_t step) const {
        std::lock_guard<std::mutex> lock(Mtx);
        const_cast<SharedMemory*>(this)->CleanupUnsafe(step);
        for(const auto& f : Foods) {
            if(!f.Locked) return true;
        }
        return false;
    }

    bool SelectClosestUnlocked(const CVector3& myPos, uint32_t step, CVector3& outPos) const {
        std::lock_guard<std::mutex> lock(Mtx);
        const_cast<SharedMemory*>(this)->CleanupUnsafe(step);
        std::size_t best = Foods.size();
        Real bestD = std::numeric_limits<Real>::max();
        for(std::size_t i = 0; i < Foods.size(); ++i) {
            if(Foods[i].Locked) continue;
            Real d = Dist(Foods[i].Pos, myPos);
            if(d < bestD) {
                bestD = d;
                best = i;
            }
        }
        if(best < Foods.size()) {
            outPos = Foods[best].Pos;
            return true;
        }
        return false;
    }

bool TryLockNear(const Controller1* owner,
                 const CVector3& myPos,
                 const CVector3& desired,
                 uint32_t step,
                 CVector3& lockedPosOut,
                 std::size_t& idxOut)
 {
        std::lock_guard<std::mutex> lock(Mtx);
        CleanupUnsafe(step);

        std::size_t match = Foods.size();
        Real matchD = std::numeric_limits<Real>::max();
        for(std::size_t i = 0; i < Foods.size(); ++i) {
            Real d = Dist(Foods[i].Pos, desired);
            if(d < matchD) {
                matchD = d;
                match = i;
            }
        }
        if(match == Foods.size() || matchD > kFoodMergeTolerance) {
            Foods.push_back({desired, false, nullptr, step, 0});
            match = Foods.size() - 1;
        }

        FoodEntry& f = Foods[match];
        if(f.Locked) {
            Real myDist = Dist(myPos, f.Pos);

            // If I already own it, refresh lock timestamps
            if(f.Owner == owner) {
                f.LockedAt = step;
                f.LastSeen = std::max(f.LastSeen, step);
                f.LockDist = myDist;
                lockedPosOut = f.Pos;
                idxOut = match;
                return true;
            }

            // Allow takeover only if I'm meaningfully closer OR the lock is getting stale
            const bool stale = (step - f.LockedAt) > 40; // recover faster from stuck owners
            const bool muchCloser = (myDist + 0.15f) < f.LockDist;

            if(stale || muchCloser) {
                f.Locked = true;
                f.Owner = owner;
                f.LockedAt = step;
                f.LastSeen = std::max(f.LastSeen, step);
                f.LockDist = myDist;
                lockedPosOut = f.Pos;
                idxOut = match;
                return true;
            }

            return false;
        }



        f.Locked = true;
        f.Owner = owner;
        f.LockedAt = step;
        f.LastSeen = std::max(f.LastSeen, step);
        f.LockDist = Dist(myPos, f.Pos);
        lockedPosOut = f.Pos;
        idxOut = match;
        return true;
    }

    void ReleaseLock(const Controller1* owner, std::size_t idx, bool erase) {
        std::lock_guard<std::mutex> lock(Mtx);
        if(idx >= Foods.size()) return;
        if(Foods[idx].Locked && Foods[idx].Owner == owner) {
            if(erase) {
                Foods.erase(Foods.begin() + idx);
            } else {
                Foods[idx].Locked = false;
                Foods[idx].Owner = nullptr;
            }
        }
    }

private:
    void CleanupUnsafe(uint32_t step) {
        for(auto& f : Foods) {
            if(f.Locked && step - f.LockedAt > kLockStaleTimeout) {
                f.Locked = false;
                f.Owner = nullptr;
            }
        }
        Foods.erase(std::remove_if(Foods.begin(), Foods.end(),
                                  [&](const FoodEntry& f) {
                                      if(!f.Locked && step - f.LastSeen > kMemoryStaleTimeout) return true;
                                      if(f.Locked && step - f.LastSeen > (kMemoryStaleTimeout + kLockStaleTimeout)) return true;
                                      return false;
                                  }),
                   Foods.end());
    }

    mutable std::mutex Mtx;
    std::vector<FoodEntry> Foods;
};

static SharedMemory& SM() {
    static SharedMemory s;
    return s;
}

} // namespace

Controller1::Controller1() :
    m_eState(EState::SEARCH),
    m_ePrevState(EState::SEARCH),
    m_uStepCount(0),
    m_bHasLockedTarget(false),
    m_cLockedTargetPos(0.0f, 0.0f, 0.0f),
    m_optLockedMemoryIndex(std::nullopt),
    m_uLastTargetSeenStep(0),
    m_uTargetLostCounter(0),
    m_bHasCandidateTarget(false),
    m_cCandidateTargetPos(0.0f, 0.0f, 0.0f),
    m_uIdleNoTargetCounter(0),
    m_uBlockEnemyTimer(0),
    m_uPostDropTimer(0),
    m_uSearchTurnTimer(0),
    m_fSearchTurnBias(0.0f) {}

void Controller1::Init(TConfigurationNode& t_tree) {
    ForagingController::Init(t_tree);
    m_eState = EState::SEARCH;
    m_ePrevState = EState::SEARCH;
    m_uStepCount = 0;
    m_bHasLockedTarget = false;
    m_optLockedMemoryIndex = std::nullopt;
    m_uLastTargetSeenStep = 0;
    m_uTargetLostCounter = 0;
    m_bHasCandidateTarget = false;
    m_uIdleNoTargetCounter = 0;
    m_uBlockEnemyTimer = 0;
    m_uPostDropTimer = 0;
    m_uSearchTurnTimer = 0;
    m_fSearchTurnBias = 0.0f;
    m_cLastPos = CVector3(0,0,0);
    m_uStuckCounter = 0;
    m_deqTrafficPoints.clear();
    SetWheelSpeeds(0.0f, 0.0f);
}

Real Controller1::Clamp(Real v, Real lo, Real hi) {
    return (v < lo ? lo : (v > hi ? hi : v));
}

void Controller1::SetWheelSpeeds(Real fLeft, Real fRight) {
    fLeft = Clamp(fLeft, -kMaxSpeed, kMaxSpeed);
    fRight = Clamp(fRight, -kMaxSpeed, kMaxSpeed);
    m_pcWheels->SetLinearVelocity(fLeft, fRight);
}

Controller1::SPerception Controller1::Sense() {
    SPerception s;
    const auto& pr = m_pcPositioning->GetReading();
    s.Position = pr.Position;
    CRadians z, y, x;
    pr.Orientation.ToEulerAngles(z, y, x);
    s.Yaw = z;

    s.AvoidanceVector.Set(0.0f, 0.0f);
    s.ProximityLevel = 0.0f; // MAX proximity (0..1), higher = closer

    m_pcRangefinders->Visit([&](const auto& rr) {
        // Track strongest proximity reading (closest obstacle)
        if(rr.Proximity > s.ProximityLevel) s.ProximityLevel = rr.Proximity;

        // Avoid when proximity is meaningful
        if(rr.Proximity > 0.05f) {
            const CQuaternion& q = std::get<2>(rr.Configuration);
            CRadians sy, sp, sr;
            q.ToEulerAngles(sy, sp, sr);

            // Stronger when closer (bigger proximity)
            Real strength = rr.Proximity;
            CRadians global_ang = s.Yaw + sy + CRadians::PI;
            s.AvoidanceVector += CVector2(strength, global_ang);
        }
    });

    s.FoodWorldPositions.clear();
    s.HasFoodInView = false;
    s.NearestFoodMetric = std::numeric_limits<Real>::max();
    s.NearestFoodWorldPos.Set(0.0f, 0.0f, 0.0f);

    const auto& cam = m_pcCamera->GetReadings();
    s.FoodWorldPositions.reserve(cam.BlobList.size());
    for(const auto& blob : cam.BlobList) {
        if(blob->Color != CColor::GRAY80) continue;
        Real dist = blob->Distance / 100.0f;
        CRadians ang = s.Yaw + blob->Angle;
        CVector3 wpos(s.Position.GetX() + dist * Cos(ang),
                      s.Position.GetY() + dist * Sin(ang),
                      0.0f);
        s.FoodWorldPositions.push_back(wpos);
        if(dist < s.NearestFoodMetric) {
            s.NearestFoodMetric = dist;
            s.NearestFoodWorldPos = wpos;
            s.HasFoodInView = true;
        }
    }

    return s;
}

void Controller1::DriveWithVector(const SPerception& s,
                                  const CVector2& cGlobalDir,
                                  Real fSpeedScale,
                                  Real fTurnGain,
                                  const CVector2& cExtra) {
    CVector2 cFinal = cGlobalDir + cExtra;
    if(cFinal.Length() < 1e-6f) {
        Real bias = Clamp(m_fSearchTurnBias, -1.0f, 1.0f);
        if(Abs(bias) < 0.15f) bias = (bias < 0 ? -0.6f : 0.6f); // enforce non-zero
        Real turn = bias * 0.35f;
        SetWheelSpeeds(-turn * kMaxSpeed, turn * kMaxSpeed);
        return;
    }

    if(cFinal.Length() > 1.0f) cFinal.Normalize();

    Real fLocalX = cFinal.GetX() * Cos(-s.Yaw) - cFinal.GetY() * Sin(-s.Yaw);
    Real fLocalY = cFinal.GetX() * Sin(-s.Yaw) + cFinal.GetY() * Cos(-s.Yaw);

    fLocalX = Clamp(fLocalX, -1.0f, 1.0f);
    fLocalY = Clamp(fLocalY, -1.0f, 1.0f);

    Real fLin = fLocalX * kMaxSpeed * fSpeedScale;
    Real fRot = fLocalY * kMaxSpeed * fTurnGain;

    // Enforce minimum forward speed when we generally want to go forward
    if(fLocalX > 0.05f) {
        Real minLin = 0.25f * kMaxSpeed * fSpeedScale;
        if(fLin < minLin) fLin = minLin;
    }

    SetWheelSpeeds(fLin - fRot, fLin + fRot);
}

CVector3 Controller1::SelectClosestBase(const CVector3& cMyPos) const {
    if(m_basePositions.empty()) return cMyPos;
    Real bestD = std::numeric_limits<Real>::max();
    CVector3 best = m_basePositions[0];
    for(const auto& b : m_basePositions) {
        Real d = (b - cMyPos).Length();
        if(d < bestD) {
            bestD = d;
            best = b;
        }
    }
    return best;
}

bool Controller1::NearBase(const CVector3& cMyPos, const CVector3& cBasePos, Real fTol) const {
    return (cBasePos - cMyPos).Length() <= fTol;
}

void Controller1::DoMemorySync(const SPerception& s) {
    if(!s.FoodWorldPositions.empty()) {
        SM().UpdateFoods(s.FoodWorldPositions, m_uStepCount);
    } else {
        SM().UpdateFoods({}, m_uStepCount);
    }
}

inline void LogWithId(const std::string& id, uint32_t step, const std::string& msg) {
    std::cout << "[R:" << id << "][Step " << step << "] " << msg << std::endl;
}

bool Controller1::TrySelectCandidateFromMemory(const CVector3& cMyPos, CVector3& cOutPos) const {
    return SM().SelectClosestUnlocked(cMyPos, m_uStepCount, cOutPos);
}

bool Controller1::TryLockCandidateTarget(CVector3& cLockedPosOut, std::size_t& unLockedIndexOut) {
    const auto& pr = m_pcPositioning->GetReading();
    return SM().TryLockNear(this, pr.Position, m_cCandidateTargetPos, m_uStepCount, cLockedPosOut, unLockedIndexOut);
}

void Controller1::ReleaseLockedTarget(bool bEraseFromMemory) {
    if(m_optLockedMemoryIndex.has_value()) {
        SM().ReleaseLock(this, m_optLockedMemoryIndex.value(), bEraseFromMemory);
    }
    m_bHasLockedTarget = false;
    m_optLockedMemoryIndex = std::nullopt;
    m_uLastTargetSeenStep = 0;
    m_uTargetLostCounter = 0;
}

void Controller1::UpdateTrafficPoints(const SPerception& s) {
    if(s.ProximityLevel <= kCollisionTrigger) return;
    if(!m_deqTrafficPoints.empty()) {
        if((m_deqTrafficPoints.back() - s.Position).Length() < 0.15f) return;
    }
    m_deqTrafficPoints.push_back(s.Position);
    while(m_deqTrafficPoints.size() > kTrafficMaxPoints) m_deqTrafficPoints.pop_front();
}

void Controller1::ControlStep() {
    const std::string id = GetId();

    ++m_uStepCount;
    SPerception s = Sense();

    if((s.Position - m_cLastPos).Length() < 0.002f) ++m_uStuckCounter;
    else m_uStuckCounter = 0;
    m_cLastPos = s.Position;

    if(m_uStuckCounter > 25 && m_eState != EState::AVOID_COLLISION) {
        m_ePrevState = m_eState;
        m_eState = EState::AVOID_COLLISION;
    }

    if(hasFood()) 
    {
        m_bHasCandidateTarget = false;
        m_uIdleNoTargetCounter = 0;
        m_uBlockEnemyTimer = 0;

        if(m_eState != EState::RETURN_TO_BASE &&
        m_eState != EState::AVOID_COLLISION) {
            ReleaseLockedTarget(false);
            m_eState = EState::RETURN_TO_BASE;
        }
    }

    DoMemorySync(s);

    UpdateTrafficPoints(s);

    if(hasFood() && m_eState != EState::RETURN_TO_BASE && m_eState != EState::AVOID_COLLISION) {
        std::cout << "[Step " << m_uStepCount << "] Action: Picked up food (post-mem sync), switching to RETURN_TO_BASE" << std::endl;
        ReleaseLockedTarget(false);
        m_bHasCandidateTarget = false;
        m_uBlockEnemyTimer = 0;
        m_uIdleNoTargetCounter = 0;
        m_eState = EState::RETURN_TO_BASE;
    }

    if(m_eState != EState::AVOID_COLLISION &&
    m_eState != EState::RETURN_TO_BASE &&
    s.ProximityLevel > kCollisionTrigger)
    {
        std::cout << "[Step " << m_uStepCount << "] Action: Collision detected (prox=" 
                << s.ProximityLevel << "), switching to AVOID_COLLISION" << std::endl;
        m_ePrevState = m_eState;
        m_eState = EState::AVOID_COLLISION;
    }


    if(m_uPostDropTimer > 0) --m_uPostDropTimer;

    CVector2 cNav(0.0f, 0.0f);
    CVector2 cExtra(0.0f, 0.0f);
    Real speedScale = 1.0f;
    Real turnGain = 1.6f;

    switch(m_eState) {
        case EState::AVOID_COLLISION: {
            std::cout << "[Step " << m_uStepCount << "] State: AVOID_COLLISION" << std::endl;
            CVector2 cAvoid = s.AvoidanceVector;
            if(cAvoid.Length() < 1e-6f) {
                SetWheelSpeeds(-0.5f * kMaxSpeed, 0.5f * kMaxSpeed);
                return;
            }
            if(cAvoid.Length() > 1.0f) cAvoid.Normalize();
            Real fSpeed = 0.8f;
            DriveWithVector(s, cAvoid, fSpeed, 2.4f, CVector2(0.0f, 0.0f));

            if(s.ProximityLevel < kCollisionClear) {
                EState back = m_ePrevState;
                if(hasFood()) back = EState::RETURN_TO_BASE;
                std::cout << "[Step " << m_uStepCount << "] Action: Collision cleared (prox="
                        << s.ProximityLevel << "), returning to previous state" << std::endl;
                m_eState = back;
            }
            return;
        }

        case EState::SEARCH: {
            LogWithId(id, m_uStepCount, "State: SEARCH");

            if(hasFood()) {
                LogWithId(id, m_uStepCount, "Action: Picked up food (SEARCH), switching to RETURN_TO_BASE");
                m_eState = EState::RETURN_TO_BASE;
                break;
            }

            // If we see food, move into TARGET_LOCKING (do NOT lock here)

            if(s.HasFoodInView) {
                LogWithId(id, m_uStepCount, "Action: Food in view, switching to TARGET_LOCKING");
                m_bHasCandidateTarget = true;
                m_cCandidateTargetPos = s.NearestFoodWorldPos;
                m_eState = EState::TARGET_LOCKING;
                break;
            }

            // Or use memory candidate
            CVector3 memCandidate;

            if(TrySelectCandidateFromMemory(s.Position, memCandidate)) {
                LogWithId(id, m_uStepCount, "Action: Using memory candidate, switching to TARGET_LOCKING");
                m_bHasCandidateTarget = true;
                m_cCandidateTargetPos = memCandidate;
                m_eState = EState::TARGET_LOCKING;
                break;
            }

            bool anyUnlocked = SM().HasUnlocked(m_uStepCount);

            if(!anyUnlocked && m_uPostDropTimer == 0) {
                ++m_uIdleNoTargetCounter;
                if(m_uIdleNoTargetCounter > kIdleToBlockTimeout) {
                    LogWithId(id, m_uStepCount, "Action: No unlocked food, switching to BLOCK_ENEMY");
                    m_uIdleNoTargetCounter = 0;
                    m_uBlockEnemyTimer = kBlockEnemyMaxTimer;
                    m_eState = EState::BLOCK_ENEMY;
                    break;
                }
            } else {
                m_uIdleNoTargetCounter = 0;
            }

            if(m_uSearchTurnTimer == 0) {
                m_fSearchTurnBias = m_rng->Uniform(CRange<Real>(-1.0f, 1.0f));
                m_uSearchTurnTimer = static_cast<uint32_t>(m_rng->Uniform(CRange<Real>(20.0f, 70.0f)));
            } else {
                --m_uSearchTurnTimer;
            }

            CVector2 forward;
            forward.Set(Cos(s.Yaw), Sin(s.Yaw));
            CVector2 drift;
            CRadians drift_angle = s.Yaw + CRadians(m_fSearchTurnBias * 0.55f);
            drift.Set(Cos(drift_angle), Sin(drift_angle));
            cNav = forward * 0.35f + drift * 0.65f;

            if(!m_basePositions.empty()) {
                CVector3 base = SelectClosestBase(s.Position);
                CVector3 diff3 = s.Position - base;
                Real distB = diff3.Length();
                if(distB < 0.9f) {
                    CVector2 rep(diff3.GetX(), diff3.GetY());
                    if(rep.Length() > 1e-6f) rep.Normalize();
                    Real mag = (0.9f - distB) / 0.9f;
                    if(m_uPostDropTimer > 0) mag *= 1.7f;
                    cExtra += rep * (0.9f * mag);
                }
            }

            cExtra += s.AvoidanceVector * 3.4f;
            speedScale = (m_uPostDropTimer > 0 ? 1.0f : 0.9f);
            turnGain = 1.6f;
            break;
        }

        case EState::TARGET_LOCKING: {
            LogWithId(id, m_uStepCount, "State: TARGET_LOCKING");

                if(hasFood()) {
                    LogWithId(id, m_uStepCount, "Action: Picked up food (TARGET_LOCKING), switching to RETURN_TO_BASE");
                    m_eState = EState::RETURN_TO_BASE;
                    break;
                }


                if(!m_bHasCandidateTarget) {
                    LogWithId(id, m_uStepCount, "Action: No candidate target, switching to SEARCH");
                    m_eState = EState::SEARCH;
                    break;
                }

                // Keep candidate fresh if we still see food (helps lock near true position)
                if(s.HasFoodInView) {
                    m_cCandidateTargetPos = s.NearestFoodWorldPos;
                }

                // Try lock atomically
                CVector3 lockedPos;
                std::size_t idx = 0;

                if(TryLockCandidateTarget(lockedPos, idx)) {
                    LogWithId(id, m_uStepCount, "Action: Locked candidate target, switching to GO_TO_TARGET");
                    m_bHasLockedTarget = true;
                    m_optLockedMemoryIndex = idx;
                    m_cLockedTargetPos = lockedPos;
                    m_uLastTargetSeenStep = m_uStepCount;
                    m_uTargetLostCounter = 0;
                    m_bHasCandidateTarget = false;
                    m_eState = EState::GO_TO_TARGET;
                    break;
                }

                // Lock failed: keep moving toward candidate for a short time, then give up
                CVector3 diff3 = m_cCandidateTargetPos - s.Position;
                cNav.Set(diff3.GetX(), diff3.GetY());
                cExtra = s.AvoidanceVector * 2.8f;
                speedScale = 0.75f;
                turnGain = 1.8f;

                // Deterministic backoff to avoid permanent chasing of reserved targets

                if(m_uStepCount % 20 == 0) {
                    LogWithId(id, m_uStepCount, "Action: Lock failed, backoff to SEARCH");
                    m_bHasCandidateTarget = false;
                    m_eState = EState::SEARCH;
                }

                break;
            }


        case EState::GO_TO_TARGET: {
            LogWithId(id, m_uStepCount, "State: GO_TO_TARGET");

            if(!m_bHasLockedTarget) {
                LogWithId(id, m_uStepCount, "Action: Lost locked target, switching to SEARCH");
                m_eState = EState::SEARCH;
                break;
            }


            if(hasFood()) {
                LogWithId(id, m_uStepCount, "Action: Picked up food (GO_TO_TARGET), switching to RETURN_TO_BASE");
                ReleaseLockedTarget(false);
                m_eState = EState::RETURN_TO_BASE;
                break;
            }

            bool refreshed = false;
            if(s.HasFoodInView) {
                for(const auto& fp : s.FoodWorldPositions) {
                    if(CloseEnough(fp, m_cLockedTargetPos, kFoodMergeTolerance * 1.25f)) {
                        m_cLockedTargetPos = fp;
                        m_uLastTargetSeenStep = m_uStepCount;
                        refreshed = true;
                        break;
                    }
                }
            }

            CVector3 diff3 = m_cLockedTargetPos - s.Position;
            Real distT = diff3.Length();


            if(distT < kTargetReachedTolerance) {
                if(!refreshed && !s.HasFoodInView) {
                    ++m_uTargetLostCounter;
                    if(m_uTargetLostCounter > 8) {
                        LogWithId(id, m_uStepCount, "Action: Target lost at close range, switching to SEARCH");
                        ReleaseLockedTarget(true);
                        m_eState = EState::SEARCH;
                        break;
                    }
                } else {
                    m_uTargetLostCounter = 0;
                }
            }


            if(m_uStepCount - m_uLastTargetSeenStep > kTargetLostTimeout) {
                LogWithId(id, m_uStepCount, "Action: Target lost (timeout), switching to SEARCH");
                ReleaseLockedTarget(false);
                m_eState = EState::SEARCH;
                break;
            }

            cNav.Set(diff3.GetX(), diff3.GetY());
            cExtra = s.AvoidanceVector * 2.2f;
            speedScale = 0.95f;
            turnGain = 1.8f;
            break;
        }

        case EState::RETURN_TO_BASE: {
            LogWithId(id, m_uStepCount, "State: RETURN_TO_BASE");
            CVector3 base = SelectClosestBase(s.Position);
            CVector3 diff3 = base - s.Position;
            Real distB = diff3.Length();


            if(distB < kBaseReachedTolerance) {
                LogWithId(id, m_uStepCount, "Action: Reached base, dropping food and switching to SEARCH");
                if(hasFood()) ClearCarriedFoodId();
                ReleaseLockedTarget(false);
                m_uPostDropTimer = kPostDropTimer;
                m_eState = EState::SEARCH;
                cNav.Set(0.0f, 0.0f);
                m_fSearchTurnBias = -0.6f;
                DriveWithVector(s, cNav, 0.0f, 1.0f, CVector2(0.0f, 0.0f));
                return;
            }

            cNav.Set(diff3.GetX(), diff3.GetY());
            cExtra = s.AvoidanceVector * 2.6f;
            speedScale = 1.0f;
            turnGain = 1.7f;

            if(distB < 0.6f) {
                CVector3 away3 = s.Position - base;
                CVector2 away(away3.GetX(), away3.GetY());
                if(away.Length() > 1e-6f) away.Normalize();
                Real mag = (0.6f - distB) / 0.6f;
                cExtra += away * (0.6f * mag);
            }
            break;
        }

        case EState::BLOCK_ENEMY: {
            LogWithId(id, m_uStepCount, "State: BLOCK_ENEMY");

            if(hasFood()) {
                LogWithId(id, m_uStepCount, "Action: Picked up food (BLOCK_ENEMY), switching to RETURN_TO_BASE");
                m_eState = EState::RETURN_TO_BASE;
                break;
            }

            if(m_uBlockEnemyTimer > 0) --m_uBlockEnemyTimer;


            if(s.HasFoodInView || SM().HasUnlocked(m_uStepCount) || m_uBlockEnemyTimer == 0) {
                LogWithId(id, m_uStepCount, "Action: Ending BLOCK_ENEMY, switching to SEARCH");
                m_eState = EState::SEARCH;
                break;
            }

            CVector3 target = s.Position;
            if(!m_deqTrafficPoints.empty()) {
                CVector3 sum(0.0f, 0.0f, 0.0f);
                for(const auto& p : m_deqTrafficPoints) sum += p;
                target = sum * (1.0f / static_cast<Real>(m_deqTrafficPoints.size()));
            } else if(!m_basePositions.empty()) {
                target = SelectClosestBase(s.Position);
            }

            CVector3 diff3 = target - s.Position;
            Real dist = diff3.Length();

            if(dist < 0.25f) {
                cNav.Set(0.0f, 0.0f);
                m_fSearchTurnBias = 0.8f;
                DriveWithVector(s, cNav, 0.0f, 1.0f, CVector2(0.0f, 0.0f));
                return;
            }

            cNav.Set(diff3.GetX(), diff3.GetY());
            cExtra = s.AvoidanceVector * 2.4f;
            speedScale = 0.55f;
            turnGain = 1.9f;
            break;
        }
    }

    // If avoidance cancels the goal, keep some goal to avoid freezing
    if(cNav.Length() > 1e-6f) {
        CVector2 navN = cNav; navN.Normalize();
        if(cExtra.Length() > 1e-6f) {
            CVector2 extraN = cExtra; extraN.Normalize();
            Real dot = navN.GetX()*extraN.GetX() + navN.GetY()*extraN.GetY();
            // If avoidance is strongly opposing the goal, reduce it
            if(dot < -0.75f) {
                cExtra *= 0.35f;
            }
        }
    }


    DriveWithVector(s, cNav, speedScale, turnGain, cExtra);
}

REGISTER_CONTROLLER(Controller1, "controller1");

} // namespace argos