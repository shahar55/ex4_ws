#ifndef TEAM1_HPP
#define TEAM1_HPP

#include "foraging.hpp"
#include <argos3/core/utility/math/vector2.h>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <vector>
#include <argos3/core/utility/math/angles.h>
#include <iostream>
#include <algorithm>
#include <limits>
#include <mutex>
#include <cmath>
#include <iomanip>

namespace argos {

class Controller1 : public ForagingController {
public:
    Controller1();
    virtual ~Controller1() {}

    void Init(TConfigurationNode& t_tree) override;
    void ControlStep() override;
    uint8_t getTeamId() const override { return 1; }

public:
    enum class EState : uint8_t {
        SEARCH = 0,
        TARGET_LOCKING,
        GO_TO_TARGET,
        RETURN_TO_BASE,
        AVOID_COLLISION,
        BLOCK_ENEMY,
        ESCAPE_STUCK,
    };

private:

    struct SPerception {
        CVector3 Position;
        CRadians Yaw;
        CVector2 AvoidanceVector;
        Real ProximityLevel;
        // Nearest robot blob (any team). Used for teammate deadlock prevention.
        bool HasRobotInView;
        Real NearestRobotDist;
        CRadians NearestRobotBearing;
        std::vector<CVector3> FoodWorldPositions;
        bool HasFoodInView;
        CVector3 NearestFoodWorldPos;
        Real NearestFoodMetric;
    };

    EState m_eState;
    EState m_ePrevState;

    uint32_t m_uStepCount;

    bool m_bHasLockedTarget;
    CVector3 m_cLockedTargetPos;
    CVector3 m_cOriginalTargetPos;
    std::optional<std::size_t> m_optLockedMemoryIndex;
    uint32_t m_uLastTargetSeenStep;
    uint32_t m_uTargetLostCounter;
    Real m_fBestDistToTarget;
    bool m_bHasCandidateTarget;
    CVector3 m_cCandidateTargetPos;

    uint32_t m_uIdleNoTargetCounter;
    uint32_t m_uBlockEnemyTimer;
    std::deque<CVector3> m_deqTrafficPoints;

    uint32_t m_uPostDropTimer;

    uint32_t m_uSearchTurnTimer;
    Real m_fSearchTurnBias;

    CVector3 m_cLastPos;
    uint32_t m_uStuckCounter;

    // --- Collision "break + resume" routine (works for enemy and teammate) ---
    uint32_t m_uAvoidTimer;
    uint8_t  m_uAvoidPhase;      // 0=backup, 1=turn, 2=forward
    Real     m_fAvoidTurnSign;   // -1 or +1 deterministic
    bool     m_bAvoidHard;       // hard break (enemy-like) vs soft (teammate-like)
    EState   m_eAvoidResumeState;

    // ---- Teammate deadlock (same-team collision) prevention & recovery ----
    // Last wheel commands (used to detect "commanding forward but not moving")
    Real m_fLastLeftCmd;
    Real m_fLastRightCmd;

    // Goal-progress tracking
    Real m_fPrevGoalDist;
    Real m_fBestGoalDist;
    uint32_t m_uNoProgressCounter;

    // Team-deadlock detection + bounded recovery routine
    uint32_t m_uTeamDeadlockCounter;
    uint32_t m_uTeamDeadlockRetries;
    bool m_bTeamRecoveryActive;
    uint32_t m_uTeamRecoveryTimer;
    uint8_t m_uTeamRecoveryPhase; // 0=reverse,1=rotate,2=forward
    EState m_eResumeState;

    // Teammate avoidance hysteresis (prevents left/right oscillation)
    CVector2 m_cTeammateAvoidLPF;
    UInt32   m_uTeammateAvoidHold;

    // Periodic summary
    uint32_t m_uLastSummaryStep;

    // --- Variables for the new Escape logic ---
    uint32_t m_uEscapeTimer;
    Real m_fEscapeLeftSpeed;
    Real m_fEscapeRightSpeed;
    uint32_t m_uStartChaseStep;

    SPerception Sense();

    void SetWheelSpeeds(Real fLeft, Real fRight);
    void DriveWithVector(const SPerception& s,
                         const CVector2& cGlobalDir,
                         Real fSpeedScale,
                         Real fTurnGain,
                         const CVector2& cExtra);

    CVector3 SelectClosestBase(const CVector3& cMyPos) const;

    void DoMemorySync(const SPerception& s);
    bool TrySelectCandidateFromMemory(const CVector3& cMyPos, CVector3& cOutPos) const;

    bool TryLockCandidateTarget(CVector3& cLockedPosOut, std::size_t& unLockedIndexOut);
    void ReleaseLockedTarget(bool bEraseFromMemory);

    bool NearBase(const CVector3& cMyPos, const CVector3& cBasePos, Real fTol) const;
    void UpdateTrafficPoints(const SPerception& s);

    static Real Clamp(Real v, Real lo, Real hi);
};

} // namespace argos

#endif
