/**
 * @file team1.cpp
 * @brief Ultimate Swarm Implementation 
 * Features: Anti-Ghosting (Carrier Reports), Tangent Bug, Deadlock Escapes, Owner-Based Locks
 */

#include "team1.hpp"
#include <argos3/core/utility/math/angles.h>
#include <iostream>
#include <algorithm>
#include <limits>
#include <mutex>

namespace argos {

namespace {

constexpr Real kMaxSpeed = 20.0f;
constexpr Real kAvoidRadius = 0.1f;
constexpr Real kCollisionTrigger = 0.2f; 
constexpr Real kCollisionClear = 0.05f;

constexpr Real kFoodMergeTolerance = 0.20f;
constexpr Real kTargetReachedTolerance = 0.10f;
constexpr Real kBaseReachedTolerance = 0.18f;

constexpr uint32_t kTargetLostTimeout = 35;
constexpr uint32_t kMemoryStaleTimeout = 60;
constexpr uint32_t kLockStaleTimeout = 60;

constexpr uint32_t kIdleToBlockTimeout = 140;
constexpr uint32_t kBlockEnemyMaxTimer = 120;
constexpr uint32_t kPostDropTimer = 40;

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

/* =======================================================================
 * SHARED MEMORY (HIVE MIND) - IMMUNE TO GHOSTING & INDEX SHIFTS
 * ======================================================================= */
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
                     std::size_t& idxOut) {
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
            if(f.Owner == owner) {
                f.LockedAt = step;
                f.LastSeen = std::max(f.LastSeen, step);
                f.LockDist = myDist;
                lockedPosOut = f.Pos;
                idxOut = match;
                return true;
            }

            const bool stale = (step - f.LockedAt) > 40; 
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

    /* SAFE RETRIEVAL: Finds target by Owner pointer, immune to index shifts */
    bool GetMyLockedFoodPos(const Controller1* owner, CVector3& outPos) const {
        std::lock_guard<std::mutex> lock(Mtx);
        for(const auto& f : Foods) {
            if(f.Locked && f.Owner == owner) {
                outPos = f.Pos;
                return true;
            }
        }
        return false;
    }

    /* SAFE RELEASE: Erases target by Owner pointer */
    void ReleaseMyLock(const Controller1* owner, bool erase_it) {
        std::lock_guard<std::mutex> lock(Mtx);
        for (auto it = Foods.begin(); it != Foods.end(); ) {
            if (it->Locked && it->Owner == owner) {
                if (erase_it) {
                    it = Foods.erase(it);
                } else {
                    it->Locked = false;
                    it->Owner = nullptr;
                    ++it;
                }
            } else {
                ++it;
            }
        }
    }

    /* --- NEW: ANTI-GHOSTING CARRIER SYSTEM --- */
    void ReportCarrier(const CVector3& pos, uint32_t step) {
        std::lock_guard<std::mutex> lock(Mtx);
        // Remove old reports (>10 ticks)
        Carriers.erase(std::remove_if(Carriers.begin(), Carriers.end(),
                                      [&](const std::pair<CVector3, uint32_t>& c) {
                                          return step - c.second > 10; 
                                      }),
                       Carriers.end());
        Carriers.push_back({pos, step});
    }

    bool IsNearCarrier(const CVector3& pos, uint32_t step) const {
        std::lock_guard<std::mutex> lock(Mtx);
        for(const auto& c : Carriers) {
            // If food is within 25cm of a robot carrying food, it's a ghost!
            if (step - c.second <= 10 && Dist(c.first, pos) < 0.25f) {
                return true;
            }
        }
        return false;
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
    std::vector<std::pair<CVector3, uint32_t>> Carriers; // Stores carrier positions
};

static SharedMemory& SM() {
    static SharedMemory s;
    return s;
}

} // namespace


/* =======================================================================
 * CONTROLLER IMPLEMENTATION
 * ======================================================================= */

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
    m_fSearchTurnBias(0.0f),
    m_uEscapeTimer(0),
    m_fEscapeLeftSpeed(0.0f),
    m_fEscapeRightSpeed(0.0f) {}

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
    m_fSearchTurnBias = m_rng->Uniform(CRange<Real>(-1.0f, 1.0f));
    m_cLastPos = CVector3(0,0,0);
    m_uStuckCounter = 0;
    m_deqTrafficPoints.clear();
    m_uAvoidTimer = 0;
    m_uEscapeTimer = 0;
    m_fEscapeLeftSpeed = 0.0f;
    m_fEscapeRightSpeed = 0.0f;
    
    SetWheelSpeeds(0.0f, 0.0f);
}

Real Controller1::Clamp(Real v, Real lo, Real hi) {
    return (v < lo ? lo : (v > hi ? hi : v));
    LOGERR<<"sdds";
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
    s.ProximityLevel = 0.0f;

    m_pcRangefinders->Visit([&](const auto& rr) {
        if(rr.Proximity > s.ProximityLevel) s.ProximityLevel = rr.Proximity;

        if(rr.Proximity > 0.05f) {
            const CQuaternion& q = std::get<2>(rr.Configuration);
            CRadians sy, sp, sr;
            q.ToEulerAngles(sy, sp, sr);
            Real strength = rr.Proximity;
            CRadians global_ang = s.Yaw + sy + CRadians::PI;
            s.AvoidanceVector += CVector2(strength * Cos(global_ang), strength * Sin(global_ang));
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
                      
        // --- ANTI-GHOSTING FILTER ---
        // If this food is near a carrier, it's not real floor food. Ignore it!
        if (SM().IsNearCarrier(wpos, m_uStepCount)) {
            continue; 
        }

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
        if(Abs(bias) < 0.15f) bias = (bias < 0 ? -0.6f : 0.6f); 
        Real turn = bias * 0.35f;
        SetWheelSpeeds(-turn * kMaxSpeed * 2, turn * kMaxSpeed);
        return;
    }

    if(cFinal.Length() > 1e-6f) cFinal.Normalize();

    Real fLocalX = cFinal.GetX() * Cos(-s.Yaw) - cFinal.GetY() * Sin(-s.Yaw);
    Real fLocalY = cFinal.GetX() * Sin(-s.Yaw) + cFinal.GetY() * Cos(-s.Yaw);

    fLocalX = Clamp(fLocalX, -1.0f, 1.0f);
    fLocalY = Clamp(fLocalY, -1.0f, 1.0f);

    Real fLin = fLocalX * kMaxSpeed * fSpeedScale;
    Real fRot = fLocalY * kMaxSpeed * fTurnGain;

    if(fLocalX > 0.05f) {
        Real minLin = 0.25f * kMaxSpeed * fSpeedScale;
        if(fLin < minLin) fLin = minLin;
    }

    SetWheelSpeeds(fLin - fRot/2, fLin + fRot);
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
    SM().ReleaseMyLock(this, bEraseFromMemory);
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

    // --- CARRIER BROADCAST ---
    // If I have food, tell the Hive Mind my position so others ignore my food!
    if (hasFood()) {
        SM().ReportCarrier(m_pcPositioning->GetReading().Position, m_uStepCount);
    }

    SPerception s = Sense();

    // --- STUCK DETECTION & ESCAPE ---
    if((s.Position - m_cLastPos).Length() < 0.03f) {
        ++m_uStuckCounter;
    } else {
        m_uStuckCounter = 0;
        m_cLastPos = s.Position; 
    }

    // fast unstuck behave
    if(m_uStuckCounter > 10 && m_eState != EState::ESCAPE_STUCK) {
        m_ePrevState = m_eState;
        //m_fSearchTurnBias = (m_rng->Uniform(CRange<Real>(0.0f, 1.0f)) > 0.5f ? 1.5f : -1.5f);
        m_eState = EState::ESCAPE_STUCK;
        
        m_uEscapeTimer = 80;
        m_fSearchTurnBias = m_rng->Uniform(CRange<Real>(-1.0f, 1.0f));
        m_fEscapeLeftSpeed = -kMaxSpeed;
        m_fEscapeRightSpeed = -kMaxSpeed * 0.1f;
        
        if (m_rng->Uniform(CRange<Real>(0.0f, 1.0f)) < 0.5f) {
            std::swap(m_fEscapeLeftSpeed, m_fEscapeRightSpeed);
        }
    }
    
    // --- UNIVERSAL FOOD CHECK ---
    if(hasFood()) {
        m_bHasCandidateTarget = false;
        m_uIdleNoTargetCounter = 0;
        m_uBlockEnemyTimer = 0;

        if(m_eState != EState::RETURN_TO_BASE && m_eState != EState::ESCAPE_STUCK) {
            ReleaseLockedTarget(true); // Destroy target from memory, we have it!
            m_eState = EState::RETURN_TO_BASE;
        }
    }

    DoMemorySync(s);
    UpdateTrafficPoints(s);

    // Collision Check
    if(m_eState != EState::AVOID_COLLISION &&
       m_eState != EState::RETURN_TO_BASE &&
       m_eState != EState::ESCAPE_STUCK &&
       m_uPostDropTimer == 0 && 
       s.ProximityLevel > kCollisionTrigger)
    {
        m_ePrevState = m_eState;
        m_eState = EState::AVOID_COLLISION;
    }

    if(m_uPostDropTimer > 0) --m_uPostDropTimer;

    CVector2 cNav(0.0f, 0.0f);
    CVector2 cExtra(0.0f, 0.0f);
    Real speedScale = 1.0f;
    Real turnGain = 1.6f;
    if (m_eState != EState::AVOID_COLLISION) {
        m_uAvoidTimer = 0;
    }
    switch(m_eState) {
        
        case EState::ESCAPE_STUCK: {
            SetWheelSpeeds(m_fEscapeLeftSpeed, m_fEscapeRightSpeed);
            if (m_uEscapeTimer > 0) {
                --m_uEscapeTimer;
            } else {
                m_uStuckCounter = 0; 
                
                if (m_ePrevState == EState::GO_TO_TARGET || m_ePrevState == EState::TARGET_LOCKING) {
                    ReleaseLockedTarget(false); 
                }
                
                m_eState = EState::SEARCH;
                m_uSearchTurnTimer = 0;
            }
            return; 
        }
        case EState::AVOID_COLLISION: {
            CVector2 cAvoid = s.AvoidanceVector;
            if(cAvoid.Length() < 1e-6f) {
                SetWheelSpeeds(-kMaxSpeed * 0.5f, kMaxSpeed * 0.5f); 
                return;
            }
            cAvoid.Normalize();

            CVector2 cTangent(-cAvoid.GetY(), cAvoid.GetX());
            
            CVector2 cNewDir = (cAvoid * 0.6f) + (cTangent * 0.8f);
            
            DriveWithVector(s, cNewDir, 0.7f, 2.5f, CVector2(0,0));

            if(s.ProximityLevel < kCollisionClear) {
                m_eState = hasFood() ? EState::RETURN_TO_BASE : m_ePrevState;
                m_uStuckCounter = 0;
            }
            return;
        }
        case EState::SEARCH: {
            if(hasFood()) {
                m_eState = EState::RETURN_TO_BASE;
                break;
            }

            if (m_uPostDropTimer == 0) {
                if(s.HasFoodInView) {
                    m_bHasCandidateTarget = true;
                    m_cCandidateTargetPos = s.NearestFoodWorldPos;
                    m_eState = EState::TARGET_LOCKING;
                    break;
                }

                CVector3 memCandidate;
                if(TrySelectCandidateFromMemory(s.Position, memCandidate)) {
                    m_bHasCandidateTarget = true;
                    m_cCandidateTargetPos = memCandidate;
                    m_eState = EState::TARGET_LOCKING;
                    break;
                }
            }

            bool anyUnlocked = SM().HasUnlocked(m_uStepCount);

            if(!anyUnlocked && m_uPostDropTimer == 0) {
                ++m_uIdleNoTargetCounter;
                if(m_uIdleNoTargetCounter > kIdleToBlockTimeout) {
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

            CVector2 forward(Cos(s.Yaw), Sin(s.Yaw));
            CRadians drift_angle = s.Yaw + CRadians(m_fSearchTurnBias * 0.55f);
            CVector2 drift(Cos(drift_angle), Sin(drift_angle));
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
            if(hasFood()) {
                m_eState = EState::RETURN_TO_BASE;
                break;
            }

            if(!m_bHasCandidateTarget) {
                m_eState = EState::SEARCH;
                break;
            }

            if(s.HasFoodInView) {
                m_cCandidateTargetPos = s.NearestFoodWorldPos;
            }

            CVector3 lockedPos;
            std::size_t idx = 0;

            if(TryLockCandidateTarget(lockedPos, idx)) {
                m_bHasLockedTarget = true;
                m_optLockedMemoryIndex = idx;
                m_cLockedTargetPos = lockedPos;
                m_uLastTargetSeenStep = m_uStepCount;
                m_uTargetLostCounter = 0;
                m_bHasCandidateTarget = false;
                m_eState = EState::GO_TO_TARGET;
                break;
            }

            CVector3 diff3 = m_cCandidateTargetPos - s.Position;
            cNav.Set(diff3.GetX(), diff3.GetY());
            cExtra = s.AvoidanceVector * 2.8f;
            speedScale = 0.75f;
            turnGain = 1.8f;

            if(m_uStepCount % 20 == 0) {
                m_bHasCandidateTarget = false;
                m_eState = EState::SEARCH;
            }
            break;
        }

        case EState::GO_TO_TARGET: {
            if(!m_bHasLockedTarget) {
                m_eState = EState::SEARCH;
                break;
            }

            if(hasFood()) {
                m_eState = EState::RETURN_TO_BASE;
                break;
            }

            CVector3 livePos;
            if (SM().GetMyLockedFoodPos(this, livePos)) {
                m_cLockedTargetPos = livePos; 
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
                    if(m_uTargetLostCounter > 40) {
                        ReleaseLockedTarget(true);
                        m_eState = EState::SEARCH;
                        break;
                    }
                } else {
                    m_uTargetLostCounter = 0;
                }
            }

            if(m_uStepCount - m_uLastTargetSeenStep > kTargetLostTimeout) {
                ReleaseLockedTarget(false);
                m_eState = EState::SEARCH;
                break;
            }

            cNav.Set(diff3.GetX(), diff3.GetY());
            if(cNav.Length() > 0.0f) {
                cNav.Normalize();
            }
            cExtra = s.AvoidanceVector * 2.2f;
            speedScale = 0.95f;
            turnGain = 1.8f;
            break;
        }

        case EState::RETURN_TO_BASE: {
            if (!hasFood()) {
                ReleaseLockedTarget(true); 
                m_uPostDropTimer = kPostDropTimer;
                m_eState = EState::SEARCH;
                break; 
            }

            CVector3 base = SelectClosestBase(s.Position);
            CVector3 diff3 = base - s.Position;
            Real distToBase = diff3.Length();
            
            cNav.Set(diff3.GetX(), diff3.GetY());
            if(cNav.Length() > 0.0f) cNav.Normalize(); 
            
            // Base Push: Ignore others when very close to base to prevent Yo-Yo
            if (distToBase < 0.35f) {
                cExtra = s.AvoidanceVector * 0.2f; 
            } else {
                cExtra = s.AvoidanceVector * 2.1f; 
            }
            
            speedScale = 1.0f;
            turnGain = 1.7f;
            break;
        }

        case EState::BLOCK_ENEMY: {
            if(hasFood()) {
                m_eState = EState::RETURN_TO_BASE;
                break;
            }

            if(m_uBlockEnemyTimer > 0) --m_uBlockEnemyTimer;

            if(s.HasFoodInView || SM().HasUnlocked(m_uStepCount) || m_uBlockEnemyTimer == 0) {
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
            CVector3 diff3 = s.Position - target; // change direction
            Real dist = diff3.Length();

            if(dist < 0.25f) {
                cNav.Set(0.0f, 0.0f);
                m_fSearchTurnBias = 1.0f; 
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

    // Tangent Bug: Smooth wall sliding
    if(cNav.Length() > 1e-6f) {
        CVector2 navN = cNav; navN.Normalize();
        if(cExtra.Length() > 1e-6f) {
            CVector2 extraN = cExtra; extraN.Normalize();
            Real dot = navN.GetX()*extraN.GetX() + navN.GetY()*extraN.GetY();
            
            if(dot < -0.6f) {
                cExtra*= 0.35f;//.Set(-cExtra.GetY(), cExtra.GetX()); 
            }
        }
    }
    // smooth forces
    if (s.ProximityLevel > 0.05f) {
        Real damping = Clamp(1.0f - (s.ProximityLevel * 2.0f), 0.1f, 1.0f);
        cNav *= damping;
    }
    
    DriveWithVector(s, cNav, speedScale, turnGain, cExtra);
}

REGISTER_CONTROLLER(Controller1, "controller1");

} // namespace argos