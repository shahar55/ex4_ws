#ifndef TEAM1_HPP
#define TEAM1_HPP

#include "foraging.hpp"
#include <vector>

namespace argos {

    struct SharedTarget {
        CVector3 WorldPosition;
        bool IsLocked;
    };

    class Controller1 : public ForagingController {

    public:
        Controller1();
        virtual ~Controller1() {}

        void Init(TConfigurationNode& t_tree) override;
        void ControlStep() override;
        uint8_t getTeamId() const override { return 1; }

    private:
        enum EState {
            STATE_SEARCH = 0,
            STATE_GO_TO_TARGET,
            STATE_RETURN_TO_BASE
        };

        EState m_eState;

        // --- Navigation Variables ---
        CVector3 m_cCurrentTargetPos;
        bool m_bHasLockedTarget;
        
        // --- Unstuck Mechanism Variables ---
        CVector3 m_cLastPosition;      // Where were we 10 steps ago?
        uint32_t m_uStuckCounter;      // How long have we been in the same place?
        bool m_bIsTakingEvasiveAction; // Are we currently spinning to get free?
        uint32_t m_uEvasionTimer;      // How long to keep spinning

        // --- Constants ---
        const Real MAX_SPEED = 0.15f; 
        const Real COLLISION_THRESHOLD = 0.12f; // Distance to start avoiding
        const Real TARGET_TOLERANCE = 0.10f;
        const Real MOVEMENT_TOLERANCE = 0.01f; // If moved less than 1cm, we are stuck
        const uint32_t STUCK_TIMEOUT = 20;     // 2 seconds (assuming 10 ticks/sec)

        // --- Helpers ---
        CVector3 GetMyPosition();
        CRadians GetMyYaw();
        
        // Logic
        void UpdateSharedMemory();
        bool AcquireTarget();
        void ReleaseTarget();
        bool IsStuck(); // Check if we haven't moved

        // Movement
        CVector2 CalculateAvoidanceVector(); // Soft avoidance
        void SetWheelSpeeds(Real fLeft, Real fRight);

        static std::vector<SharedTarget> m_vecSharedTargets;
    };
}

#endif