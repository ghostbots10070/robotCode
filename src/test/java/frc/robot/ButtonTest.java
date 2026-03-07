// package frc.robot;

// import org.junit.jupiter.api.Test;

// import static org.junit.jupiter.api.Assertions.assertTrue;
// import static org.junit.jupiter.api.Assertions.assertEquals;

// import frc.robot.commands.ClimbDown;
// import frc.robot.commands.ClimbUp;
// import frc.robot.subsystems.ClimberSubsystem;

// class ButtonTest {
//     @Test
//     void testClimbDownCommand() {
//         ClimberSubsystem climber = new ClimberSubsystem();
//         ClimbDown command = new ClimbDown(climber);

//         command.initialize();
//         assertEquals(frc.robot.Constants.ClimbConstants.CLIMBER_MOTOR_DOWN_PERCENT, climber.getLastSetPower(), "ClimbDown should set correct percent");

//         command.end(false);
//         assertTrue(climber.wasStopCalled(), "ClimbDown should stop climber on end");
//     }
// }