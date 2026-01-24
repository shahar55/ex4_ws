#include "team1.hpp"
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/logging/argos_log.h>

namespace argos {

    std::vector<SharedTarget> Controller1::m_vecSharedTargets;

    Controller1::Controller1() : 
        m_eState(STATE_SEARCH),
        m_bHasLockedTarget(false),
        m_uStuckCounter(0),
        m_bIsTakingEvasiveAction(false),
        m_uEvasionTimer(0) {}

    void Controller1::Init(TConfigurationNode& t_tree) {
        ForagingController::Init(t_tree);
        m_eState = STATE_SEARCH;
        m_bHasLockedTarget = false;
        m_uStuckCounter = 0;
        m_bIsTakingEvasiveAction = false;
        m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
        m_cLastPosition = GetMyPosition();
    }

    void Controller1::ControlStep() {
        // 1. Update Perception
        UpdateSharedMemory();
        
        // 2. State Maintenance (Food Logic)
        if (hasFood()) {
            if (m_eState != STATE_RETURN_TO_BASE) {
                ReleaseTarget(); 
                m_eState = STATE_RETURN_TO_BASE;
            }
        } else if (m_eState == STATE_RETURN_TO_BASE && !hasFood()) {
            m_eState = STATE_SEARCH;
        }

        // --- 3. UNSTUCK LOGIC (Highest Priority) ---
        // If we are currently executing an evasive maneuver (spinning), continue doing it.
        if (m_bIsTakingEvasiveAction) {
            m_uEvasionTimer--;
            // Spin hard
            SetWheelSpeeds(MAX_SPEED, -MAX_SPEED);
            
            if (m_uEvasionTimer == 0) {
                m_bIsTakingEvasiveAction = false; // Maneuver done
                m_uStuckCounter = 0; // Reset counter
            }
            return; // Exit here
        }

        // Check if we are physically stuck (haven't moved despite trying)
        if (IsStuck()) {
            // Trigger Evasion
            m_bIsTakingEvasiveAction = true;
            m_uEvasionTimer = 15; // Spin for 1.5 seconds
            RLOG << "STUCK DETECTED! Initiating Evasion." << std::endl;
            return;
        }


        // --- 4. NAVIGATION LOGIC (Vectors) ---
        
        CVector2 cNavVector(0.0f, 0.0f);

        switch (m_eState) {
            case STATE_SEARCH: {
                if (AcquireTarget()) {
                    m_eState = STATE_GO_TO_TARGET;
                } else {
                    // Search Pattern: Move Forward + Slight Random Turn
                    cNavVector.Set(1.0f, 0.0f); // Forward vector
                    
                    // Add small random noise to Y (turning) to cover area
                    Real fNoise = m_rng->Uniform(CRange<Real>(-0.2f, 0.2f));
                    cNavVector += CVector2(0.0f, fNoise);
                }
                break;
            }

            case STATE_GO_TO_TARGET: {
                // Vector to Target
                CVector3 cMyPos = GetMyPosition();
                CVector3 cTargetDiff = m_cCurrentTargetPos - cMyPos;
                
                // Check arrival
                if (cTargetDiff.Length() < TARGET_TOLERANCE && !hasFood()) {
                    ReleaseTarget();
                    m_eState = STATE_SEARCH;
                    break;
                }

                // Global vector to target
                cNavVector.Set(cTargetDiff.GetX(), cTargetDiff.GetY());
                if (cNavVector.Length() > 0) cNavVector.Normalize();
                break;
            }

            case STATE_RETURN_TO_BASE: {
                // Logic to find base and set cNavVector
                if (!m_basePositions.empty()) {
                    CVector3 cMyPos = GetMyPosition();
                    CVector3 cBestBase = m_basePositions[0];
                    Real fMinDist = 10000.0f;
                    for (const auto& base : m_basePositions) {
                        Real d = (base - cMyPos).Length();
                        if (d < fMinDist) { fMinDist = d; cBestBase = base; }
                    }
                    CVector3 cBaseDiff = cBestBase - cMyPos;
                    cNavVector.Set(cBaseDiff.GetX(), cBaseDiff.GetY());
                    if (cNavVector.Length() > 0) cNavVector.Normalize();
                }
                break;
            }
        }

        // --- 5. AVOIDANCE FUSION ---
        // Calculate soft avoidance (push away from walls)
        CVector2 cAvoid = CalculateAvoidanceVector();
        
        // If we are in Search, Avoidance is strong. If Going to Target, it's medium.
        Real fAvoidWeight = (m_eState == STATE_SEARCH) ? 4.0f : 2.5f;
        
        // Combine Vectors: Target + (Weight * Repulsion)
        CVector2 cFinal = cNavVector + (cAvoid * fAvoidWeight);

        // --- 6. MOTOR CONTROL ---
        // Convert Global Vector to Local Robot Frame
        CRadians cMyYaw = GetMyYaw();
        Real fLocalX = cFinal.GetX() * Cos(-cMyYaw) - cFinal.GetY() * Sin(-cMyYaw);
        Real fLocalY = cFinal.GetX() * Sin(-cMyYaw) + cFinal.GetY() * Cos(-cMyYaw);

        // Drive
        Real fLinSpeed = fLocalX * MAX_SPEED;
        Real fRotSpeed = fLocalY * 3.0f; // Turn gain

        // Always maintain some forward speed if no major obstacle
        if (fLinSpeed < 0.02f && fLocalX > 0) fLinSpeed = 0.05f;

        SetWheelSpeeds(fLinSpeed - fRotSpeed, fLinSpeed + fRotSpeed);
    }

    // --- HELPER IMPLEMENTATIONS ---

    bool Controller1::IsStuck() {
        // Every 10 steps (1 second), check displacement
        if (m_uStuckCounter % 10 == 0) {
            CVector3 cCurrPos = GetMyPosition();
            Real fDist = (cCurrPos - m_cLastPosition).Length();
            m_cLastPosition = cCurrPos; // Update check point

            if (fDist < MOVEMENT_TOLERANCE) {
                // Check if we accumulated enough "non-movement" time
                // We use a separate internal counter or just relying on the loop
                // Simple implementation: If logic called > STUCK_TIMEOUT times without reset
                // Here we simplify: Just check raw counter in ControlStep
            }
        }
        
        m_uStuckCounter++;
        if (m_uStuckCounter > STUCK_TIMEOUT) {
            CVector3 cCurrPos = GetMyPosition();
            Real fTotalDist = (cCurrPos - m_cLastPosition).Length(); // Dist since reset
            
            // If after 2 seconds we haven't moved 2cm -> STUCK.
            if (fTotalDist < 0.02f) {
                return true;
            } else {
                // We moved, reset logic
                m_uStuckCounter = 0;
                m_cLastPosition = cCurrPos;
                return false;
            }
        }
        return false;
    }

    CVector2 Controller1::CalculateAvoidanceVector() {
        CVector2 cAccum(0.0f, 0.0f);
        m_pcRangefinders->Visit([&](const auto &s_sensor) {
            // Only care about really close things
            if (s_sensor.Proximity < COLLISION_THRESHOLD) {
                // Extract Angle
                const CQuaternion& cOrient = std::get<2>(s_sensor.Configuration);
                CRadians cYaw, cP, cR;
                cOrient.ToEulerAngles(cYaw, cP, cR);

                Real fStr = (COLLISION_THRESHOLD - s_sensor.Proximity) / COLLISION_THRESHOLD;
                cAccum += CVector2(fStr, cYaw + CRadians::PI);
            }
        });
        return cAccum;
    }

    void Controller1::SetWheelSpeeds(Real fLeft, Real fRight) {
        if (fLeft > MAX_SPEED) fLeft = MAX_SPEED;
        if (fLeft < -MAX_SPEED) fLeft = -MAX_SPEED;
        if (fRight > MAX_SPEED) fRight = MAX_SPEED;
        if (fRight < -MAX_SPEED) fRight = -MAX_SPEED;
        m_pcWheels->SetLinearVelocity(fLeft, fRight);
    }

    CVector3 Controller1::GetMyPosition() {
        return m_pcPositioning->GetReading().Position;
    }

    CRadians Controller1::GetMyYaw() {
        CRadians cZ, cY, cX;
        m_pcPositioning->GetReading().Orientation.ToEulerAngles(cZ, cY, cX);
        return cZ;
    }

    void Controller1::UpdateSharedMemory() {
        const auto& sReadings = m_pcCamera->GetReadings();
        CVector3 cMyPos = GetMyPosition();
        CRadians cMyYaw = GetMyYaw();
        for (const auto& blob : sReadings.BlobList) {
            if (blob->Color == CColor::GRAY80) {
                Real fDist = blob->Distance / 100.0f;
                CRadians cAngle = cMyYaw + blob->Angle;
                CVector3 cFood(cMyPos.GetX()+fDist*Cos(cAngle), cMyPos.GetY()+fDist*Sin(cAngle), 0);
                bool bExists = false;
                for (auto& t : m_vecSharedTargets) {
                    if ((t.WorldPosition - cFood).Length() < 0.20f) { bExists=true; break; }
                }
                if (!bExists) m_vecSharedTargets.push_back({cFood, false});
            }
        }
    }

    bool Controller1::AcquireTarget() {
        CVector3 cMyPos = GetMyPosition();
        int iBest = -1; 
        Real fMin = 10000.0f;
        for (size_t i=0; i<m_vecSharedTargets.size(); ++i) {
            if (!m_vecSharedTargets[i].IsLocked) {
                Real d = (m_vecSharedTargets[i].WorldPosition - cMyPos).Length();
                if (d < fMin) { fMin=d; iBest=i; }
            }
        }
        if (iBest != -1) {
            m_vecSharedTargets[iBest].IsLocked = true;
            m_cCurrentTargetPos = m_vecSharedTargets[iBest].WorldPosition;
            m_bHasLockedTarget = true;
            return true;
        }
        return false;
    }

    void Controller1::ReleaseTarget() {
        if (!m_bHasLockedTarget) return;
        for (auto it = m_vecSharedTargets.begin(); it != m_vecSharedTargets.end(); ) {
            if ((it->WorldPosition - m_cCurrentTargetPos).Length() < 0.20f) {
                it = m_vecSharedTargets.erase(it);
            } else ++it;
        }
        m_bHasLockedTarget = false;
    }

    REGISTER_CONTROLLER(Controller1, "controller1");
}