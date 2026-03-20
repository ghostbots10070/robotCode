package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Publishes match timing and phase-shift state for Elastic/SmartDashboard.
 */
// https://docs.wpilib.org/en/stable/docs/yearly-overview/2026-game-data.html
public final class MatchStatusPublisher {
    private MatchStatusPublisher() {
    }

    public static void publish() {
        double matchTime = Math.max(0.0, DriverStation.getMatchTime());

        boolean isAuto = DriverStation.isAutonomousEnabled();
        boolean isTeleop = DriverStation.isTeleopEnabled();
        boolean isEnabled = DriverStation.isEnabled();

        String phase = isAuto ? "AUTO" : (isTeleop ? "TELEOP" : (isEnabled ? "ENABLED" : "DISABLED"));
        double autoTimeLeft = isAuto ? matchTime : 0.0;
        double teleopTimeLeft = isTeleop ? matchTime : 0.0;

        boolean blueWonAuto = blueWonAutoFromGameData();
        Optional<DriverStation.Alliance> allianceOpt = DriverStation.getAlliance();
        boolean isBlueAlliance = allianceOpt.isPresent() && allianceOpt.get() == DriverStation.Alliance.Blue;

        boolean autoWonByUs = isBlueAlliance ? blueWonAuto : !blueWonAuto;
        boolean ourHubActive = !autoWonByUs;
        boolean opponentHubActive = autoWonByUs;

        boolean currentShiftBlue = isCurrentShiftBlue(matchTime, blueWonAuto);
        boolean currentShiftRed = !currentShiftBlue;
        boolean currentShiftIsYours = isBlueAlliance ? currentShiftBlue : currentShiftRed;
        int shiftTimeLeft = timeLeftInShiftSeconds(matchTime);

        SmartDashboard.putString("Match/Phase", phase);
        SmartDashboard.putNumber("Match/TimeLeft", matchTime);
        SmartDashboard.putNumber("Match/AutoTimeLeft", autoTimeLeft);
        SmartDashboard.putNumber("Match/TeleopTimeLeft", teleopTimeLeft);
        SmartDashboard.putBoolean("Match/IsAuto", isAuto);
        SmartDashboard.putBoolean("Match/IsTeleop", isTeleop);
        SmartDashboard.putBoolean("Match/IsEnabled", isEnabled);

        SmartDashboard.putString("Match/Alliance", isBlueAlliance ? "Blue" : "Red");
        SmartDashboard.putBoolean("Match/BlueWonAuto", blueWonAuto);
        SmartDashboard.putBoolean("Match/AutoWonByUs", autoWonByUs);

        SmartDashboard.putBoolean("Match/OurHubActive", ourHubActive);
        SmartDashboard.putBoolean("Match/OpponentHubActive", opponentHubActive);

        SmartDashboard.putBoolean("Match/CurrentShiftBlue", currentShiftBlue);
        SmartDashboard.putBoolean("Match/CurrentShiftRed", currentShiftRed);
        SmartDashboard.putBoolean("Match/CurrentShiftIsYours", currentShiftIsYours);
        SmartDashboard.putNumber("Match/ShiftTimeLeftSeconds", shiftTimeLeft);
    }

    public static boolean blueWonAutoFromGameData() {
        String matchInfo = DriverStation.getGameSpecificMessage();
        return matchInfo != null && !matchInfo.isEmpty() && matchInfo.charAt(0) == 'B';
    }

    public static int timeLeftInShiftSeconds(double currentMatchTime) {
        if (currentMatchTime >= 130) {
            return (int) (currentMatchTime - 130);
        } else if (currentMatchTime >= 105) {
            return (int) (currentMatchTime - 105);
        } else if (currentMatchTime >= 80) {
            return (int) (currentMatchTime - 80);
        } else if (currentMatchTime >= 55) {
            return (int) (currentMatchTime - 55);
        } else if (currentMatchTime >= 30) {
            return (int) (currentMatchTime - 30);
        } else {
            return (int) currentMatchTime;
        }
    }

    public static boolean isCurrentShiftBlue(double currentMatchTime, boolean blueWonAuto) {
        if (currentMatchTime >= 105 && currentMatchTime < 130) {
            return !blueWonAuto;
        } else if (currentMatchTime >= 80 && currentMatchTime < 105) {
            return blueWonAuto;
        } else if (currentMatchTime >= 55 && currentMatchTime < 80) {
            return !blueWonAuto;
        } else if (currentMatchTime >= 30 && currentMatchTime < 55) {
            return blueWonAuto;
        } else {
            return true;
        }
    }
}