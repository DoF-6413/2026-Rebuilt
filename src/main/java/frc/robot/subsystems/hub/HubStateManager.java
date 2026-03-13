// package frc.robot.subsystems.hub;

// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import java.util.Optional;
// import org.littletonrobotics.junction.Logger;

// public class HubStateManager extends SubsystemBase {

//   public enum HubIndicator {
//     GREEN, // 0 - Safe to shoot
//     YELLOW, // 1 - Opponent window ending (<2s)
//     RED // 2 - Inactive (Opponent scoring)
//   }

//   private HubIndicator currentState = HubIndicator.GREEN;
//   private double secondsUntilNextState = 0.0;

//   public HubStateManager() {
//     // This "seeds" the table so the values appear in the sidebar for adding to
//     // the UI.  Using setDefault... is better than put... here because it won't
//     // overwrite a value if you've already changed it on the dashboard while the
//     // robot is restarting.
//     SmartDashboard.setDefaultBoolean("HubTestMode", false);
//     SmartDashboard.setDefaultString("TestHubGameData", "R");
//   }

//   public HubIndicator getState() {
//     return currentState;
//   }

//   @Override
//   public void periodic() {
//     String gameData;
//     double teleopElapsed;

//     // 1. Failsafe Logic: Always disable test mode if the FMS is connected
//     boolean hubTestMode = SmartDashboard.getBoolean("HubTestMode", false);

//     if (DriverStation.isFMSAttached() && hubTestMode) {
//       hubTestMode = false;
//       SmartDashboard.putBoolean("HubTestMode", false); // Force dashboard to match reality
//     }

//     Optional<Alliance> alliance = DriverStation.getAlliance();

//     // 2. Determine Timing and Data Source
//     // MatchTime counts down from 140.0 to 0.0 during Teleop
//     teleopElapsed = 140.0 - DriverStation.getMatchTime();

//     if (hubTestMode) {
//       gameData = SmartDashboard.getString("TestHubGameData", "R");
//     } else {
//       gameData = DriverStation.getGameSpecificMessage();
//     }

//     // 3. Disabled/Safety Check
//     // Allows testing on the bench while disabled if hubTestMode is active
//     if (!hubTestMode && (alliance.isEmpty() || !DriverStation.isTeleopEnabled())) {
//       currentState = HubIndicator.GREEN;
//       secondsUntilNextState = 0.0;
//       publish();
//       return;
//     }

//     // 4. Phase: TRANSITION SHIFT (0-10s)
//     if (teleopElapsed < 10.0) {
//       currentState = HubIndicator.GREEN;
//       secondsUntilNextState = 10.0 - teleopElapsed;
//     }
//     // 5. Phase: END GAME (110-140s)
//     else if (teleopElapsed >= 110.0) {
//       currentState = HubIndicator.GREEN;
//       secondsUntilNextState = 140.0 - teleopElapsed;
//     }
//     // 6. Phase: ALLIANCE SHIFTS (10-110s)
//     else {
//       if (gameData.isEmpty()) {
//         // Keep safe until data arrives
//         currentState = HubIndicator.GREEN;
//         secondsUntilNextState = 0.0;
//       } else {
//         calculateShiftState(teleopElapsed, gameData, alliance.orElse(Alliance.Red));
//       }
//     }

//     publish();
//   }

//   /** Logic for the four 25-second alternating shifts. */
//   private void calculateShiftState(double teleopElapsed, String gameData, Alliance ourAlliance) {
//     // 'R' = Red is inactive in Shift 1. 'B' = Blue is inactive in Shift 1.
//     boolean redInactiveFirst = (gameData.charAt(0) == 'R');

//     boolean ourHubStartsActive =
//         switch (ourAlliance) {
//           case Red -> !redInactiveFirst;
//           case Blue -> redInactiveFirst;
//         };

//     // Determine shift index (0, 1, 2, or 3)
//     int shiftIndex = (int) ((teleopElapsed - 10.0) / 25.0);

//     // Status flips every shift
//     boolean ourHubActive = (shiftIndex % 2 == 0) ? ourHubStartsActive : !ourHubStartsActive;

//     double shiftEnd = 10.0 + ((shiftIndex + 1) * 25.0);
//     secondsUntilNextState = shiftEnd - teleopElapsed;

//     if (ourHubActive) {
//       currentState = HubIndicator.GREEN;
//     } else if (secondsUntilNextState < 2.0) {
//       currentState = HubIndicator.YELLOW;
//     } else {
//       currentState = HubIndicator.RED;
//     }
//   }

//   private void publish() {
//     // 1. Send the raw data for logging and coach's text view
//     SmartDashboard.putNumber("Hub/IndicatorRaw", currentState.ordinal());
//     SmartDashboard.putNumber("Hub/SecondsRemaining", secondsUntilNextState);

//     // 2. Boolean indicators for the "Color Block" widgets on the Dashboard
//     SmartDashboard.putBoolean("Hub/IsGreen", currentState == HubIndicator.GREEN);
//     SmartDashboard.putBoolean("Hub/IsYellow", currentState == HubIndicator.YELLOW);
//     SmartDashboard.putBoolean("Hub/IsRed", currentState == HubIndicator.RED);

//     // AdvantageKit Logging
//     Logger.recordOutput("Hub/State", currentState);
//     Logger.recordOutput("Hub/SecondsUntilNext", secondsUntilNextState);
//   }
// }
