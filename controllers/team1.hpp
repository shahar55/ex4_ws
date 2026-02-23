#ifndef TEAM1_HPP
#define TEAM1_HPP

#include "foraging.hpp"
#include <argos3/core/utility/math/vector2.h>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <vector>

namespace argos {

class Controller1 : public ForagingController {
public:
    Controller1();
    virtual ~Controller1() {}

    void Init(TConfigurationNode& t_tree) override;
    void ControlStep() override;
    uint8_t getTeamId() const override { return 1; }

private:
    enum class EState : uint8_t {
        SEARCH = 0,
        TARGET_LOCKING,
        GO_TO_TARGET,
        RETURN_TO_BASE,
        AVOID_COLLISION,
        BLOCK_ENEMY,
        ESCAPE_STUCK,
    };

    struct SPerception {
        CVector3 Position;
        CRadians Yaw;
        CVector2 AvoidanceVector;
        Real ProximityLevel;
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
    Real m_fBestDistToTarget; // המשתנה החדש שלך
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
    uint32_t m_uAvoidTimer;
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