#include "team1.hpp"

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
    m_uStuckCounter(0),
    m_uAvoidTimer(0),
    m_uAvoidPhase(0),
    m_fAvoidTurnSign(1.0f),
    m_bAvoidHard(false),
    m_eAvoidResumeState(EState::SEARCH),
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
    m_uAvoidPhase = 0;
    m_fAvoidTurnSign = 1.0f;
    m_bAvoidHard = false;
    m_eAvoidResumeState = EState::SEARCH;

    m_uEscapeTimer = 0;
    m_fEscapeLeftSpeed = 0.0f;
    m_fEscapeRightSpeed = 0.0f;

    m_fLastLeftCmd = 0.0f;
    m_fLastRightCmd = 0.0f;
    m_fPrevGoalDist = std::numeric_limits<Real>::max();
    m_fBestGoalDist = std::numeric_limits<Real>::max();
    m_uNoProgressCounter = 0;
    m_uTeamDeadlockCounter = 0;
    m_uTeamDeadlockRetries = 0;
    m_bTeamRecoveryActive = false;
    m_uTeamRecoveryTimer = 0;
    m_uTeamRecoveryPhase = 0;
    m_eResumeState = EState::SEARCH;
    m_uLastSummaryStep = 0;

    m_cTeammateAvoidLPF = CVector2();
    m_uTeammateAvoidHold = 0u;

    SetWheelSpeeds(0.0f, 0.0f);
}

Real Controller1::Clamp(Real v, Real lo, Real hi) {
    return (v < lo ? lo : (v > hi ? hi : v));
}

void Controller1::SetWheelSpeeds(Real fLeft, Real fRight) {
    fLeft = Clamp(fLeft, -kMaxSpeed, kMaxSpeed);
    fRight = Clamp(fRight, -kMaxSpeed, kMaxSpeed);
    m_fLastLeftCmd = fLeft;
    m_fLastRightCmd = fRight;
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
    s.HasRobotInView = false;
    s.NearestRobotDist = std::numeric_limits<Real>::max();
    s.NearestRobotBearing = CRadians::ZERO;
    s.NearestFoodMetric = std::numeric_limits<Real>::max();
    s.NearestFoodWorldPos.Set(0.0f, 0.0f, 0.0f);

    const auto& cam = m_pcCamera->GetReadings();
    s.FoodWorldPositions.reserve(cam.BlobList.size());
    for(const auto& blob : cam.BlobList) {
        // Food is reported as GRAY80; non-gray blobs are robots (team LEDs).
        if(blob->Color != CColor::GRAY80) {
            Real rdist = blob->Distance / 100.0f;
            if(rdist < s.NearestRobotDist) {
                s.NearestRobotDist = rdist;
                s.NearestRobotBearing = blob->Angle;
                s.HasRobotInView = true;
            }
            continue;
        }
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

inline int ParseNumericId(const std::string& id) {
    // Expected ids like "puck1-0" / "pipuck1-0" / "robot0". Fallback to 0.
    int num = 0;
    for(int i = static_cast<int>(id.size()) - 1; i >= 0; --i) {
        if(id[i] >= '0' && id[i] <= '9') {
            int j = i;
            while(j >= 0 && id[j] >= '0' && id[j] <= '9') --j;
            try {
                num = std::stoi(id.substr(j + 1, i - j));
            } catch(...) { num = 0; }
            break;
        }
    }
    return num;
}

inline const char* StateToStr(Controller1::EState st) {
    switch(st) {
        case Controller1::EState::SEARCH: return "SEARCH";
        case Controller1::EState::TARGET_LOCKING: return "TARGET_LOCKING";
        case Controller1::EState::GO_TO_TARGET: return "GO_TO_TARGET";
        case Controller1::EState::RETURN_TO_BASE: return "RETURN_TO_BASE";
        case Controller1::EState::AVOID_COLLISION: return "AVOID_COLLISION";
        case Controller1::EState::BLOCK_ENEMY: return "BLOCK_ENEMY";
        case Controller1::EState::ESCAPE_STUCK: return "ESCAPE_STUCK";
        default: return "UNKNOWN";
    }
}

inline void LogKV(uint32_t step,
                  const std::string& rid,
                  Controller1::EState st,
                  bool has_food,
                  const std::optional<std::size_t>& target_id,
                  const CVector3& target_pos,
                  const std::string& action,
                  const std::string& kv) {
    const Real t = static_cast<Real>(step) / 10.0f; // ticks_per_second=10
    std::cout << "[t=" << t << "][R:" << rid << "][Step:" << step << "][State:" << StateToStr(st) << "] "
              << "has_food=" << (has_food ? 1 : 0) << " "
              << "target_id=";
    if(target_id.has_value()) std::cout << *target_id;
    else std::cout << "none";
    std::cout << " "
              << "target_pos=(" << target_pos.GetX() << "," << target_pos.GetY() << ") "
              << "action=" << action;
    if(!kv.empty()) std::cout << " " << kv;
    std::cout << std::endl;
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
    const EState state_before = m_eState;

    // --- CARRIER BROADCAST ---
    if (hasFood()) {
        SM().ReportCarrier(m_pcPositioning->GetReading().Position, m_uStepCount);
    }

    SPerception s = Sense();

    // --- STUCK / TEAM-DEADLOCK DETECTION & BOUNDED RECOVERY (deterministic) ---
    const int my_num_id = ParseNumericId(id);

    // Track basic motion
    const Real pos_delta = (s.Position - m_cLastPos).Length();
    if(pos_delta < 0.01f) ++m_uStuckCounter;
    else m_uStuckCounter = 0;
    m_cLastPos = s.Position;

    // Compute current goal distance (for progress tracking)
    Real goal_dist = std::numeric_limits<Real>::max();
    if(m_eState == EState::GO_TO_TARGET && m_bHasLockedTarget) {
        goal_dist = (m_cLockedTargetPos - s.Position).Length();
    } else if(m_eState == EState::RETURN_TO_BASE) {
        goal_dist = (SelectClosestBase(s.Position) - s.Position).Length();
    }

    if(goal_dist < m_fBestGoalDist - 0.02f) {
        m_fBestGoalDist = goal_dist;
        m_uNoProgressCounter = 0;
    } else {
        if(goal_dist < std::numeric_limits<Real>::max()) ++m_uNoProgressCounter;
    }
    const Real dd_goal = (m_fPrevGoalDist < std::numeric_limits<Real>::max() && goal_dist < std::numeric_limits<Real>::max())
                         ? (goal_dist - m_fPrevGoalDist)
                         : 0.0f;
    m_fPrevGoalDist = goal_dist;

    const bool wheels_push = (Abs(m_fLastLeftCmd) + Abs(m_fLastRightCmd)) > (1.0f * kMaxSpeed);
    const bool robot_front_block = s.HasRobotInView && (s.NearestRobotDist < 0.18f) && (Abs(s.NearestRobotBearing.GetValue()) < 0.45f);

    // Specialized trigger: likely head-on / bumper lock
    if(!m_bTeamRecoveryActive &&
       m_eState != EState::ESCAPE_STUCK &&
       m_eState != EState::AVOID_COLLISION &&
       robot_front_block &&
       wheels_push &&
       pos_delta < 0.01f &&
       m_uNoProgressCounter > 8) {
        ++m_uTeamDeadlockCounter;
        if(m_uTeamDeadlockCounter == 1 || m_uTeamDeadlockCounter == 10) {
            LogKV(m_uStepCount, id, m_eState, hasFood(), m_optLockedMemoryIndex,
                  (m_bHasLockedTarget ? m_cLockedTargetPos : SelectClosestBase(s.Position)),
                  "TEAM_DEADLOCK_DETECTED",
                  "pos_delta=" + std::to_string(pos_delta) +
                  " front_robot_dist=" + std::to_string(s.NearestRobotDist) +
                  " bearing=" + std::to_string(s.NearestRobotBearing.GetValue()) +
                  " dd_goal=" + std::to_string(dd_goal) +
                  " retries=" + std::to_string(m_uTeamDeadlockRetries));
        }
    } else {
        m_uTeamDeadlockCounter = 0;
    }

    const uint32_t kDeadlockTriggerSteps = 14;
    if(!m_bTeamRecoveryActive &&
       m_uTeamDeadlockCounter >= kDeadlockTriggerSteps &&
       m_eState != EState::ESCAPE_STUCK) {

        // Too many retries => release lock ONLY for food target (not base)
        if(m_uTeamDeadlockRetries >= 3 && m_eState == EState::GO_TO_TARGET) {
            LogKV(m_uStepCount, id, m_eState, hasFood(), m_optLockedMemoryIndex,
                  m_cLockedTargetPos,
                  "LOCK_RELEASED",
                  "reason=TEAM_DEADLOCK_RETRIES");
            ReleaseLockedTarget(false);
            m_eState = EState::SEARCH;
            m_uTeamDeadlockRetries = 0;
            m_uTeamDeadlockCounter = 0;
        } else {
            m_ePrevState = m_eState;
            m_eResumeState = m_eState;
            m_eState = EState::ESCAPE_STUCK;
            m_bTeamRecoveryActive = true;
            m_uTeamRecoveryPhase = 0;
            m_uTeamRecoveryTimer = 0;
            ++m_uTeamDeadlockRetries;

            LogKV(m_uStepCount, id, m_ePrevState, hasFood(), m_optLockedMemoryIndex,
                  (m_bHasLockedTarget ? m_cLockedTargetPos : SelectClosestBase(s.Position)),
                  "TEAM_DEADLOCK_RECOVERY",
                  "phase=reverse resume_state=" + std::string(StateToStr(m_eResumeState)));
        }
    }

    // Generic stuck fallback
    if(!m_bTeamRecoveryActive &&
       m_uStuckCounter > 18 &&
       m_eState != EState::ESCAPE_STUCK &&
       m_eState != EState::AVOID_COLLISION) {
        m_ePrevState = m_eState;
        m_eResumeState = m_eState;
        m_eState = EState::ESCAPE_STUCK;
        m_bTeamRecoveryActive = true;
        m_uTeamRecoveryPhase = 0;
        m_uTeamRecoveryTimer = 0;
        m_uTeamDeadlockRetries = 0;

        LogKV(m_uStepCount, id, m_ePrevState, hasFood(), m_optLockedMemoryIndex,
              (m_bHasLockedTarget ? m_cLockedTargetPos : SelectClosestBase(s.Position)),
              "TEAM_DEADLOCK_RECOVERY",
              "phase=reverse reason=GENERIC_STUCK resume_state=" + std::string(StateToStr(m_eResumeState)));
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

    // === Collision Check (ALL states except ESCAPE_STUCK; includes RETURN_TO_BASE) ===
    if(m_eState != EState::AVOID_COLLISION &&
       m_eState != EState::ESCAPE_STUCK &&
       m_uPostDropTimer == 0 &&
       s.ProximityLevel > kCollisionTrigger) {

        m_ePrevState = m_eState;
        m_eAvoidResumeState = m_eState;
        m_eState = EState::AVOID_COLLISION;

        // Reset routine
        m_uAvoidTimer = 0;
        m_uAvoidPhase = 0;

        // Deterministic turn side: break symmetry by id + robot bearing (if exists)
        const Real bear = s.HasRobotInView ? s.NearestRobotBearing.GetValue() : 0.0f;
        const int id_sign = (my_num_id % 2 == 0) ? 1 : -1;
        const int bear_sign = (bear >= 0.0f) ? 1 : -1;
        // turn away from bearing; id breaks ties
        m_fAvoidTurnSign = static_cast<Real>(-id_sign * bear_sign);

        // Classify encounter:
        // - Near base tends to be teammate traffic => SOFT (minimal direction change)
        // - Very close / head-on / strong contact => HARD
        const Real base_dist = (SelectClosestBase(s.Position) - s.Position).Length();
        const bool near_base = base_dist < 0.65f;

        const bool very_close = s.HasRobotInView && std::isfinite(s.NearestRobotDist) && (s.NearestRobotDist < 0.16f);
        const bool head_on = s.HasRobotInView && std::isfinite(s.NearestRobotDist) &&
                             (s.NearestRobotDist < 0.18f) && (Abs(s.NearestRobotBearing.GetValue()) < 0.30f);
        const bool strong_contact = (s.ProximityLevel < 0.040f); // close wall/robot

        m_bAvoidHard = (!near_base) && (very_close || head_on || strong_contact);

        LogKV(m_uStepCount, id, m_ePrevState, hasFood(), m_optLockedMemoryIndex,
              (m_bHasLockedTarget ? m_cLockedTargetPos : SelectClosestBase(s.Position)),
              "AVOID_COLLISION_ENTER",
              "prox=" + std::to_string(s.ProximityLevel) +
              (s.HasRobotInView ? (" robot_dist=" + std::to_string(s.NearestRobotDist) +
                                   " robot_bearing=" + std::to_string(s.NearestRobotBearing.GetValue())) : "") +
              " mode=" + std::string(m_bAvoidHard ? "HARD" : "SOFT") +
              " turn_sign=" + std::to_string(m_fAvoidTurnSign) +
              " resume_state=" + std::string(StateToStr(m_eAvoidResumeState)));
    }

    if(m_uPostDropTimer > 0) --m_uPostDropTimer;

    CVector2 cNav(0.0f, 0.0f);
    CVector2 cExtra(0.0f, 0.0f);
    Real speedScale = 1.0f;
    Real turnGain = 1.6f;

    switch(m_eState) {

        case EState::ESCAPE_STUCK: {
            // Deterministic bounded recovery (used for teammate-deadlock + generic stuck)
            if(!m_bTeamRecoveryActive) {
                m_eState = EState::SEARCH;
                m_uStuckCounter = 0;
                return;
            }

            const Real bear = s.HasRobotInView ? s.NearestRobotBearing.GetValue() : 0.0f;
            const int id_sign = (my_num_id % 2 == 0) ? 1 : -1;
            const int bear_sign = (bear >= 0.0f) ? 1 : -1;
            const int turn_sign = -id_sign * bear_sign; // rotate away, break symmetry by id

            constexpr uint32_t T_REV = 10;
            constexpr uint32_t T_ROT = 10;
            constexpr uint32_t T_FWD = 14;

            if(m_uTeamRecoveryPhase == 0) {
                SetWheelSpeeds(-0.8f * kMaxSpeed, -0.8f * kMaxSpeed);
                ++m_uTeamRecoveryTimer;
                if(m_uTeamRecoveryTimer >= T_REV) {
                    m_uTeamRecoveryPhase = 1;
                    m_uTeamRecoveryTimer = 0;
                    LogKV(m_uStepCount, id, m_ePrevState, hasFood(), m_optLockedMemoryIndex,
                          (m_bHasLockedTarget ? m_cLockedTargetPos : SelectClosestBase(s.Position)),
                          "TEAM_DEADLOCK_RECOVERY",
                          "phase=rotate resume_state=" + std::string(StateToStr(m_eResumeState)));
                }
                return;
            }

            if(m_uTeamRecoveryPhase == 1) {
                SetWheelSpeeds(-turn_sign * 0.6f * kMaxSpeed, turn_sign * 0.6f * kMaxSpeed);
                ++m_uTeamRecoveryTimer;
                if(m_uTeamRecoveryTimer >= T_ROT) {
                    m_uTeamRecoveryPhase = 2;
                    m_uTeamRecoveryTimer = 0;
                    LogKV(m_uStepCount, id, m_ePrevState, hasFood(), m_optLockedMemoryIndex,
                          (m_bHasLockedTarget ? m_cLockedTargetPos : SelectClosestBase(s.Position)),
                          "TEAM_DEADLOCK_RECOVERY",
                          "phase=forward resume_state=" + std::string(StateToStr(m_eResumeState)));
                }
                return;
            }

            SetWheelSpeeds(0.9f * kMaxSpeed, 0.9f * kMaxSpeed);
            ++m_uTeamRecoveryTimer;
            if(m_uTeamRecoveryTimer >= T_FWD) {
                m_bTeamRecoveryActive = false;
                m_uTeamRecoveryTimer = 0;
                m_uTeamRecoveryPhase = 0;
                m_uTeamDeadlockCounter = 0;
                m_uStuckCounter = 0;
                m_uNoProgressCounter = 0;
                m_fBestGoalDist = std::numeric_limits<Real>::max();

                const EState resume = hasFood() ? EState::RETURN_TO_BASE : m_eResumeState;
                m_eState = resume;

                LogKV(m_uStepCount, id, m_eState, hasFood(), m_optLockedMemoryIndex,
                      (m_bHasLockedTarget ? m_cLockedTargetPos : SelectClosestBase(s.Position)),
                      "TEAM_DEADLOCK_RECOVERY",
                      "phase=done resume_state=" + std::string(StateToStr(resume)));
            }
            return;
        }

        case EState::AVOID_COLLISION: {
            // NEW: break direction briefly then resume objective (does not wait for "clear" forever)
            // Soft (teammates): minimal heading change (shorter turn + more forward)
            // Hard (enemy/head-on): stronger back+turn to guarantee separation

            const uint32_t T_BACK = m_bAvoidHard ? 9u : 5u;
            const uint32_t T_TURN = m_bAvoidHard ? 14u : 7u;
            const uint32_t T_FWD  = m_bAvoidHard ? 10u : 7u;

            // If we were pushed into a wall, back up even if sensors flicker
            if(m_uAvoidPhase == 0) {
                const Real back = m_bAvoidHard ? 0.85f : 0.60f;
                SetWheelSpeeds(-back * kMaxSpeed, -back * kMaxSpeed);
                ++m_uAvoidTimer;
                if(m_uAvoidTimer >= T_BACK) {
                    m_uAvoidPhase = 1;
                    m_uAvoidTimer = 0;
                    LogKV(m_uStepCount, id, m_eState, hasFood(), m_optLockedMemoryIndex,
                          (m_bHasLockedTarget ? m_cLockedTargetPos : SelectClosestBase(s.Position)),
                          "AVOID_COLLISION_PHASE",
                          "phase=TURN mode=" + std::string(m_bAvoidHard ? "HARD" : "SOFT") +
                          " turn_sign=" + std::to_string(m_fAvoidTurnSign));
                }
                return;
            }

            if(m_uAvoidPhase == 1) {
                const Real turn = m_bAvoidHard ? 0.75f : 0.45f;
                // arc turn: one wheel forward, one backward (quick reorientation)
                SetWheelSpeeds(-m_fAvoidTurnSign * turn * kMaxSpeed,
                                m_fAvoidTurnSign * turn * kMaxSpeed);
                ++m_uAvoidTimer;
                if(m_uAvoidTimer >= T_TURN) {
                    m_uAvoidPhase = 2;
                    m_uAvoidTimer = 0;
                    LogKV(m_uStepCount, id, m_eState, hasFood(), m_optLockedMemoryIndex,
                          (m_bHasLockedTarget ? m_cLockedTargetPos : SelectClosestBase(s.Position)),
                          "AVOID_COLLISION_PHASE",
                          "phase=FWD mode=" + std::string(m_bAvoidHard ? "HARD" : "SOFT"));
                }
                return;
            }

            // phase 2 forward (re-engage the lane)
            const Real fwd = m_bAvoidHard ? 0.95f : 0.80f;
            SetWheelSpeeds(fwd * kMaxSpeed, fwd * kMaxSpeed);
            ++m_uAvoidTimer;

            // Early exit if sensors show clear
            const bool clear_now = (s.ProximityLevel < kCollisionClear);
            if(clear_now || m_uAvoidTimer >= T_FWD) {
                const EState resume = hasFood() ? EState::RETURN_TO_BASE : m_eAvoidResumeState;
                LogKV(m_uStepCount, id, m_eState, hasFood(), m_optLockedMemoryIndex,
                      (m_bHasLockedTarget ? m_cLockedTargetPos : SelectClosestBase(s.Position)),
                      "AVOID_COLLISION_EXIT",
                      "resume_state=" + std::string(StateToStr(resume)) +
                      " clear=" + std::to_string(clear_now ? 1 : 0));
                m_eState = resume;
                m_uAvoidTimer = 0;
                m_uAvoidPhase = 0;
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

            const bool obstacle_close = (s.ProximityLevel < 0.065f);
            if (distToBase < 0.35f) {
                cExtra = obstacle_close ? (s.AvoidanceVector * 0.7f) : CVector2();
            } else {
                cExtra = obstacle_close ? (s.AvoidanceVector * 2.0f) : CVector2();
            }

            speedScale = (distToBase < 0.65f ? 0.70f : 1.0f);
            turnGain = 2.1f;
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
                cExtra *= 0.35f;
            }
        }
    }

    // Smooth forces: apply only when close to obstacles.
    if (s.ProximityLevel < 0.065f) {
        Real damping = Clamp(s.ProximityLevel / 0.065f, 0.15f, 1.0f);
        cNav *= damping;
    }

    // --- Layer A: robot-robot prevention: repulse + tangential slide + deterministic yield ---
    if(m_eState != EState::AVOID_COLLISION &&
       m_eState != EState::ESCAPE_STUCK &&
       s.HasRobotInView) {

        const Real personal_space = 0.22f;
        const Real exit_space     = 0.26f;

        if(!std::isfinite(s.NearestRobotDist) || s.NearestRobotDist > exit_space) {
            m_cTeammateAvoidLPF = CVector2();
            m_uTeammateAvoidHold = 0u;
        }

        if(s.NearestRobotDist < personal_space && std::isfinite(s.NearestRobotDist)) {
            const Real w = Clamp((personal_space - s.NearestRobotDist) / personal_space, 0.0f, 1.0f);

            CVector2 repulse; repulse.FromPolarCoordinates(s.NearestRobotDist, s.NearestRobotBearing);
            if(repulse.Length() > 1e-6f) repulse.Normalize();
            repulse *= -1.0f;

            CVector2 tangent(-repulse.GetY(), repulse.GetX());
            const Real side = (my_num_id % 2 == 0) ? 1.0f : -1.0f;
            tangent *= side;

            const Real base_dist = (SelectClosestBase(s.Position) - s.Position).Length();
            const bool near_base = base_dist < 0.65f;
            const Real base_boost = near_base ? 1.8f : 1.0f;

            CVector2 avoidRaw = (repulse * (3.2f * w * base_boost)) + (tangent * (2.2f * w * base_boost));

            if(cNav.Length() > 0.01f) {
                CVector2 goal = cNav;
                goal.Normalize();
                avoidRaw += goal * (1.2f * w);
            }

            const Real alpha = 0.75f;
            if(m_uTeammateAvoidHold == 0u) {
                m_cTeammateAvoidLPF = avoidRaw;
            } else {
                m_cTeammateAvoidLPF = (m_cTeammateAvoidLPF * alpha) + (avoidRaw * (1.0f - alpha));
            }
            m_uTeammateAvoidHold = 5u;

            const Real maxMag = 4.0f * base_boost;
            if(m_cTeammateAvoidLPF.Length() > maxMag) {
                m_cTeammateAvoidLPF.Normalize();
                m_cTeammateAvoidLPF *= maxMag;
            }

            cExtra += m_cTeammateAvoidLPF;

            const bool head_on = (s.NearestRobotDist < 0.14f) && (Abs(s.NearestRobotBearing.GetValue()) < 0.25f);
            if(head_on) {
                const bool i_yield = (my_num_id % 2) == 0;
                if(i_yield) speedScale = std::min<argos::Real>(speedScale, argos::Real(0.45));
                else        speedScale = std::max<argos::Real>(speedScale, argos::Real(0.65));
            } else {
                speedScale = std::max<argos::Real>(speedScale, argos::Real(0.65));
            }
        }

        const Real hold_space = 0.22f;
        if (m_uTeammateAvoidHold > 0u && std::isfinite(s.NearestRobotDist) && s.NearestRobotDist < hold_space) {
            cExtra += m_cTeammateAvoidLPF * 0.60f;
            speedScale = std::max<argos::Real>(speedScale, argos::Real(0.75));
            --m_uTeammateAvoidHold;
        }
    }

    // Periodic summary (every 20 steps)
    if(m_uStepCount % 20 == 0) {
        CVector3 tpos(0.0f, 0.0f, 0.0f);
        if(m_bHasLockedTarget) tpos = m_cLockedTargetPos;
        else if(hasFood()) tpos = SelectClosestBase(s.Position);
        LogKV(m_uStepCount, id, m_eState, hasFood(), m_optLockedMemoryIndex,
              tpos,
              "SUMMARY",
              "pos=(" + std::to_string(s.Position.GetX()) + "," + std::to_string(s.Position.GetY()) + ")" +
              " prox=" + std::to_string(s.ProximityLevel) +
              (s.HasRobotInView ? (" robot_dist=" + std::to_string(s.NearestRobotDist) +
                                   " robot_bearing=" + std::to_string(s.NearestRobotBearing.GetValue())) : ""));
    }

    // State change logging
    if(m_eState != state_before) {
        LogKV(m_uStepCount, id, m_eState, hasFood(), m_optLockedMemoryIndex,
              (m_bHasLockedTarget ? m_cLockedTargetPos : (hasFood() ? SelectClosestBase(s.Position) : CVector3(0,0,0))),
              "STATE_CHANGE",
              "from=" + std::string(StateToStr(state_before)) + " to=" + std::string(StateToStr(m_eState)));
    }

    DriveWithVector(s, cNav, speedScale, turnGain, cExtra);
}

} // namespace argos

REGISTER_CONTROLLER(Controller1, "controller1");
