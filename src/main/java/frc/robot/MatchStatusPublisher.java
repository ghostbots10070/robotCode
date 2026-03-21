package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Publishes simplified match timing, hub state, and game state for Elastic/SmartDashboard.
 */
public final class MatchStatusPublisher {

    private MatchStatusPublisher() {
    }

    public static void publish() {
        double periodTime = Math.max(0.0, DriverStation.getMatchTime());
        
        double totalMatchTimeRemaining = 150.0;
        double timeUntilNextShift = 0.0;
        double shiftMaxTime = 1.0; // Prevents divide-by-zero when disabled
        String gameState = "Disabled";
        boolean isHubActive = false;

        // Calculate continuous match time (150 down to 0)
        if (DriverStation.isAutonomous()) {
            totalMatchTimeRemaining = periodTime + 135.0;
        } else if (DriverStation.isTeleop()) {
            totalMatchTimeRemaining = periodTime;
        }

        if (DriverStation.isEnabled()) {
            if (DriverStation.isAutonomousEnabled()) {
                gameState = "Auto";
                isHubActive = true;
                timeUntilNextShift = periodTime; 
                shiftMaxTime = 15.0; // Auto is 15s long
            } else if (DriverStation.isTeleopEnabled()) {
                boolean shift1Active = isShift1ActiveForUs();

                if (periodTime > 130) {
                    gameState = "Transition";
                    isHubActive = true;
                    timeUntilNextShift = periodTime - 130;
                    shiftMaxTime = 5.0; // Transition is 5s long
                } else if (periodTime > 105) {
                    gameState = "Shift 1";
                    isHubActive = shift1Active;
                    timeUntilNextShift = periodTime - 105;
                    shiftMaxTime = 25.0; // Shifts are 25s long
                } else if (periodTime > 80) {
                    gameState = "Shift 2";
                    isHubActive = !shift1Active;
                    timeUntilNextShift = periodTime - 80;
                    shiftMaxTime = 25.0;
                } else if (periodTime > 55) {
                    gameState = "Shift 3";
                    isHubActive = shift1Active;
                    timeUntilNextShift = periodTime - 55;
                    shiftMaxTime = 25.0;
                } else if (periodTime > 30) {
                    gameState = "Shift 4";
                    isHubActive = !shift1Active;
                    timeUntilNextShift = periodTime - 30;
                    shiftMaxTime = 25.0;
                } else {
                    gameState = "Endgame";
                    isHubActive = true;
                    timeUntilNextShift = periodTime;
                    shiftMaxTime = 30.0; // Endgame is 30s long
                }
            }
        }

        SmartDashboard.putBoolean("Match/IsHubActive", isHubActive);
        SmartDashboard.putString("Match/GameState", gameState);
        
        SmartDashboard.putNumber("Match/TimeUntilNextShiftText", timeUntilNextShift);
        SmartDashboard.putNumber("Match/MatchTimeText", totalMatchTimeRemaining);

        // 2. PROGRESS BAR: Normalized between 0.0 and 1.0
        double shiftProgress = timeUntilNextShift / shiftMaxTime;
        SmartDashboard.putNumber("Match/ShiftProgress", shiftProgress);
    }

    private static boolean isShift1ActiveForUs() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) return true; 

        String gameData = DriverStation.getGameSpecificMessage();
        if (gameData == null || gameData.isEmpty()) return true;

        boolean redInactiveFirst = gameData.charAt(0) == 'R';
        return switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };
    }
}