

package frc.robot.subsystems;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;


import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DriveConstants;




public class DriveSubsystem extends SubsystemBase {
   //gyro
   private final AHRS m_gyro;
  
   //track robot field location for dashboard
   private boolean gyroZeroPending = true;
   private Field2d m_field = new Field2d();

   private boolean halfSpeed = false;
  
   //motors
   private final SparkMax ejectMotor;
  
   private final SparkMax leftLeader;
   private final SparkMax leftFollower;
   private final SparkMax rightLeader;
   private final SparkMax rightFollower;


   //motor configs
   private final SparkMaxConfig leftLeaderConfig = new SparkMaxConfig();
   private final SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
   private final SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();
   private final SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();
   // private final SparkMaxConfig driveConfig = new SparkMaxConfig();
   private final SparkMaxConfig ejectConfig = new SparkMaxConfig();


   //encoders
   private final RelativeEncoder m_encoderleftLeader;
   private final RelativeEncoder m_encoderleftFollower;
   private final RelativeEncoder m_encoderrightLeader;
   private final RelativeEncoder m_encoderrightFollower;


   //motor pid controllers
   private final SparkClosedLoopController m_leftLeaderPIDController;
   private final SparkClosedLoopController m_rightLeaderPIDController;


   //main drive function
   private final DifferentialDrive myDrive;


   //odometry class for tracking robot on field
   private DifferentialDrivePoseEstimator m_driveOdometry;


   //motor feedforward
   SimpleMotorFeedforward m_driveFeedForward =
     new SimpleMotorFeedforward(
         DriveConstants.ksDriveVolts,
         DriveConstants.kvDriveVoltSecondsPerMeter,
         DriveConstants.kaDriveVoltSecondsSquaredPerMeter);




   public DriveSubsystem() {


   //init gyro
   m_gyro = new AHRS(NavXComType.kMXP_SPI);


   //init motor
   ejectMotor = new SparkMax(5, MotorType.kBrushed);


   leftLeader = new SparkMax(1, MotorType.kBrushed);
   leftFollower = new SparkMax(2, MotorType.kBrushed);
   rightLeader = new SparkMax(3, MotorType.kBrushed);
   rightFollower = new SparkMax(4, MotorType.kBrushed);

   rightLeaderConfig.inverted(true);
   rightFollowerConfig.inverted(true);


   //init drive function
   myDrive = new DifferentialDrive(leftLeader, rightLeader);
  
   //init encoders
   m_encoderleftLeader = leftLeader.getEncoder();
   m_encoderleftFollower = leftFollower.getEncoder();
   m_encoderrightLeader = rightLeader.getEncoder();
   m_encoderrightFollower = rightFollower.getEncoder();

   // invert motors

   // init PID Controllers
   m_leftLeaderPIDController = leftLeader.getClosedLoopController();
   m_rightLeaderPIDController = rightLeader.getClosedLoopController();
  


   // configure encoders
   // RPM TO m/s
   leftLeaderConfig.encoder.velocityConversionFactor(DriveConstants.VELOCITY_CONVERSION_RATIO);
   rightLeaderConfig.encoder.velocityConversionFactor(DriveConstants.VELOCITY_CONVERSION_RATIO);
   leftFollowerConfig.encoder.velocityConversionFactor(DriveConstants.VELOCITY_CONVERSION_RATIO);
   rightFollowerConfig.encoder.velocityConversionFactor(DriveConstants.VELOCITY_CONVERSION_RATIO);
   // rotations to meters
   leftLeaderConfig.encoder.positionConversionFactor(DriveConstants.POSITION_CONVERSION_RATIO);
   rightLeaderConfig.encoder.positionConversionFactor(DriveConstants.POSITION_CONVERSION_RATIO);
   leftFollowerConfig.encoder.positionConversionFactor(DriveConstants.POSITION_CONVERSION_RATIO);
   rightFollowerConfig.encoder.positionConversionFactor(DriveConstants.POSITION_CONVERSION_RATIO);
   resetEncoders();
  
   // setup PID controllers
   configureMotorPIDControllers();


   // setup main and secondary motors
   leftFollowerConfig.follow(leftLeader); // set front left to follow back left
   rightFollowerConfig.follow(rightLeader); // set front right to follow back right
  
  
  
   //burn config to motor
   // driveConfig.smartCurrentLimit(60);
   // driveConfig.voltageCompensation(12);


   // driveConfig.follow(leftLeader);
   // leftFollower.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


   // driveConfig.follow(rightLeader);
   // rightFollower.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


   // driveConfig.disableFollowerMode();
   // rightLeader.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


   // driveConfig.inverted(true);
   // leftLeader.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


   ejectConfig.smartCurrentLimit(60);
   ejectConfig.voltageCompensation(10);
   ejectMotor.configure(ejectConfig, ResetMode.kResetSafeParameters,
   PersistMode.kPersistParameters);






   //init odometry
   m_driveOdometry =
       new DifferentialDrivePoseEstimator(
           DriveConstants.kDriveKinematics,
           getRotation2d(),
           getPositionLeft(),
           getPositionRight(),
           new Pose2d());




  
     // All other subsystem initialization
     // ...
      // Load the RobotConfig from the GUI settings. You should probably
     // store this in your Constants file


     // Configure AutoBuilder last
     AutoBuilder.configure(
             this::getPose, // Robot pose supplier
             this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
             this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
             this::setSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
             new PPLTVController(0.02), // PPLTVController is the built in path following controller for differential drive trains
             DriveConstants.autoConfig, // The robot configuration
             () -> {
               // Boolean supplier that controls when the path will be mirrored for the red alliance
               // This will flip the path being followed to the red side of the field.
               // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                var alliance = DriverStation.getAlliance();
               if (alliance.isPresent()) {
                 return alliance.get() == DriverStation.Alliance.Red;
               }
               return false;
             },
             this
        // Reference to this subsystem to set requirements
     );


     PathPlannerLogging.setLogActivePathCallback((poses) -> m_field.getObject("path").setPoses(poses));


     SmartDashboard.putData("Field", m_field);


   }










   public Pose2d getPose() {
       return m_driveOdometry.getEstimatedPosition();
     }


//
     public Rotation2d getRotation2d() {
       return m_gyro.getRotation2d();
     }
  
     // for balance correction
     public double getPitch() {
       return m_gyro.getPitch(); // get pitch in degrees
     }
  
     // for PID control (turn by degrees)
     public double getAccumYaw() {
       return m_gyro.getAngle(); // get angle in degrees
     }
  
     public double getYaw() {
       return m_gyro.getYaw();
     }
  
     public void resetGyro() {
       m_gyro.reset();
     }




     //
     public double getVelocityLeft() {
       return m_encoderleftLeader.getVelocity();
     }
  
     public double getVelocityRight() {
       return m_encoderrightLeader.getVelocity();
     }
  
     public double getPositionLeft() {
       return m_encoderleftLeader.getPosition();
     }
  
     public double getPositionRight() {
       return m_encoderrightLeader.getPosition();
     }


     public void resetPose(Pose2d pose) {
       resetEncoders();
       m_driveOdometry.resetPosition(getRotation2d(), getPositionLeft(), getPositionRight(), pose);
     }


    public void arcadeDrive(double leftSpeed, double rightSpeed) {
      myDrive.arcadeDrive(leftSpeed, rightSpeed);
    }

    public void eject(boolean light, boolean hard){
      if(hard){
        ejectMotor.set(0.60);
      }else if(light){
        ejectMotor.set(0.35);
      }else{
        ejectMotor.set(0);
      }
    }

    public void drive(boolean half, boolean light, boolean hard, double leftSpeed, double rightSpeed){
      int divider;
      if (half){
        halfSpeed = !halfSpeed;
      }
      if (halfSpeed){
        divider = 2;
      }else{
        divider = 1;
      }
      arcadeDrive(leftSpeed / divider, rightSpeed / divider);
      eject(light, hard);
    }



     public void resetEncoders() {
       m_encoderleftLeader.setPosition(0);
       m_encoderleftFollower.setPosition(0);
       m_encoderrightLeader.setPosition(0);
       m_encoderrightFollower.setPosition(0);
     }




     public DifferentialDriveWheelSpeeds getWheelSpeeds() {
       return new DifferentialDriveWheelSpeeds(getVelocityLeft(), getVelocityRight());
     }


     public ChassisSpeeds getSpeeds() {
       return DriveConstants.kDriveKinematics.toChassisSpeeds(getWheelSpeeds());
     }


     public void setSpeeds(ChassisSpeeds speeds) {
       setWheelVelocities(DriveConstants.kDriveKinematics.toWheelSpeeds(speeds));
     }


       public void setWheelVelocities(DifferentialDriveWheelSpeeds speeds) {
           // get left and right speeds in m/s, and run through feedforward to get feedforward voltage
           // offset
           double leftSpeed = speeds.leftMetersPerSecond;
           double rightSpeed = speeds.rightMetersPerSecond;
           // set to position of motors
           m_leftLeaderPIDController.setReference(
               leftSpeed,
               SparkBase.ControlType.kVelocity,
               DriveConstants.kDrivetrainVelocityPIDSlot,
               m_driveFeedForward.calculate(leftSpeed));
               m_rightLeaderPIDController.setReference(
               rightSpeed,
               SparkBase.ControlType.kVelocity,
               DriveConstants.kDrivetrainVelocityPIDSlot,
               m_driveFeedForward.calculate(rightSpeed));
   }


   private void configureMotorPIDControllers() {
       // setup velocity PID controllers (used by auto)
       // PID
       leftLeaderConfig.closedLoop.pid(
           DriveConstants.kPDriveVel,
           DriveConstants.kIDriveVel,
           DriveConstants.kDDriveVel,
           DriveConstants.kDrivetrainVelocityPIDSlot);
           rightLeaderConfig.closedLoop.pid(
           DriveConstants.kPDriveVel,
           DriveConstants.kIDriveVel,
           DriveConstants.kDDriveVel,
           DriveConstants.kDrivetrainVelocityPIDSlot);
       // Set Izone (Integral Zone)
       leftLeaderConfig.closedLoop.iZone(
           DriveConstants.kIzDriveVel, DriveConstants.kDrivetrainVelocityPIDSlot);
           rightLeaderConfig.closedLoop.iZone(
           DriveConstants.kIzDriveVel, DriveConstants.kDrivetrainVelocityPIDSlot);
       // set output range
       leftLeaderConfig.closedLoop.outputRange(
           DriveConstants.kMinOutputDrive,
           DriveConstants.kMaxOutputDrive,
           DriveConstants.kDrivetrainVelocityPIDSlot);
           rightLeaderConfig.closedLoop.outputRange(
           DriveConstants.kMinOutputDrive,
           DriveConstants.kMaxOutputDrive,
           DriveConstants.kDrivetrainVelocityPIDSlot);
  
       // setup position PID controllers (used when we manually path find)
       // PID
       leftLeaderConfig.closedLoop.pid(
           DriveConstants.kPDrivePos,
           DriveConstants.kIDrivePos,
           DriveConstants.kDDrivePos,
           DriveConstants.kDrivetrainPositionPIDSlot);
           rightLeaderConfig.closedLoop.pid(
           DriveConstants.kPDrivePos,
           DriveConstants.kIDrivePos,
           DriveConstants.kDDrivePos,
           DriveConstants.kDrivetrainPositionPIDSlot);
       // Integral Zone
  
       leftLeaderConfig.closedLoop.iZone(
           DriveConstants.kIzDrivePos, DriveConstants.kDrivetrainPositionPIDSlot);
           rightLeaderConfig.closedLoop.iZone(
           DriveConstants.kIzDrivePos, DriveConstants.kDrivetrainPositionPIDSlot);
       // Output Range
       leftLeaderConfig.closedLoop.outputRange(
           DriveConstants.kMinOutputDrive,
           DriveConstants.kMaxOutputDrive,
           DriveConstants.kDrivetrainPositionPIDSlot);
           rightLeaderConfig.closedLoop.outputRange(
           DriveConstants.kMinOutputDrive,
           DriveConstants.kMaxOutputDrive,
           DriveConstants.kDrivetrainPositionPIDSlot);
     }

     @Override
     public void periodic() {
      if (gyroZeroPending && !m_gyro.isCalibrating()) {
        resetGyro();
        gyroZeroPending = false;
      }
      // Update the odometry in the periodic block
      m_driveOdometry.update(getRotation2d(), getPositionLeft(), getPositionRight());
      m_field.setRobotPose(getPose());
    }
 }
