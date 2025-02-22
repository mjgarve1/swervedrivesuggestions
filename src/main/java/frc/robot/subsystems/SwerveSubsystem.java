// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */

   private final SwerveModule frontLeft = new SwerveModule(
        DriveConstants.kFrontLeftDriveMotorPort, 
        DriveConstants.kFrontLeftTurningMotorPort, 
        DriveConstants.kFrontLeftDriveEncoderReversed, 
        DriveConstants.kFrontLeftTurningEncoderReversed, 
        DriveConstants.kFrontLeftDriveAbsoluteEncoderPort, 
        DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad, 
        DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed
        );
    private final SwerveModule frontRight = new SwerveModule(
        DriveConstants.kFrontRightDriveMotorPort, 
        DriveConstants.kFrontRightTurningMotorPort, 
        DriveConstants.kFrontRightDriveEncoderReversed, 
        DriveConstants.kFrontRightTurningEncoderReversed, 
        DriveConstants.kFrontRightDriveAbsoluteEncoderPort, 
        DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad, 
        DriveConstants.kFrontRightDriveAbsoluteEncoderReversed
        );
    private final SwerveModule backLeft = new SwerveModule(
        DriveConstants.kBackLeftDriveMotorPort, 
        DriveConstants.kBackLeftTurningMotorPort, 
        DriveConstants.kBackLeftDriveEncoderReversed, 
        DriveConstants.kBackLeftTurningEncoderReversed, 
        DriveConstants.kBackLeftDriveAbsoluteEncoderPort, 
        DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad, 
        DriveConstants.kBackLeftDriveAbsoluteEncoderReversed
        );
    private final SwerveModule backRight = new SwerveModule(
        DriveConstants.kBackRightDriveMotorPort, 
        DriveConstants.kBackRightTurningMotorPort, 
        DriveConstants.kBackRightDriveEncoderReversed, 
        DriveConstants.kBackRightTurningEncoderReversed, 
        DriveConstants.kBackRightDriveAbsoluteEncoderPort, 
        DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad, 
        DriveConstants.kBackRightDriveAbsoluteEncoderReversed
        );

         //creates a navX gyro to use in da calcs
    private final AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI, AHRS.NavXUpdateRate.k50Hz);
    
    // private ADXRS450_Gyro gyro = new ADXRS450_Gyro();

    //throughout, poseEstimator is used for odometry but the odometer is used to get the initial pose I guess, if not I have to use vectors and that confuses me.
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,new Rotation2d(0),
        new SwerveModulePosition[]{frontLeft.getPosition(),frontRight.getPosition(),backLeft.getPosition(),backRight.getPosition()});

    private final SwerveDrivePoseEstimator m_poseEstimator =
      new SwerveDrivePoseEstimator(
          DriveConstants.kDriveKinematics,
          gyro.getRotation2d(),
          new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
          }, odometer.getPoseMeters());

    private ChassisSpeeds chassisSpeeds;
    private RobotConfig config;

  public SwerveSubsystem() {
    new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();


        //The following is the code to configure the pathplanner auto builder
        try{
            config = RobotConfig.fromGUISettings();
        } catch(Exception e){
            e.printStackTrace();
        }


        AutoBuilder.configure(
            this.getPose(), //gets a supplier of pose2d
            this.resetOdometry(getPose()), //used if the auto needs to reset the pose
            this.getRobotRelativeSpeeds(), //uses the chassisSpeeds relative to the robot
            (speeds, feedforwards) -> driveRobotRelative(speeds), //used to command the robot chassis speeds using robot relative speeds
            new PPHolonomicDriveController(
                new PIDConstants(AutoConstants.kAutoTranslationP, 0.0, 0.0), 
                new PIDConstants(AutoConstants.kAutoRotationP, 0.0, 0.0)
            ),
            config, 
            () -> {
                var alliance = DriverStation.getAlliance();
                if(alliance.isPresent()){
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
        );
    }

    public void zeroHeading() {
        gyro.reset();
    }
    //gets the heading returned as the gyro reading remainder after being divided by 360
    //that way it always reads from 0 to 360
    //or 0 to -360
    public double getHeading() {
        //this being negative screws with the gyro. - J
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }
    //returns as radians?
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }
    //returns Pose with x,y, and theta coordinates of robot
    // changed to a supplier for the sake of the autoBuilder and pathPlanner - J
    public Supplier<Pose2d> getPose(){
        return () -> m_poseEstimator.getEstimatedPosition();
    }

    //We moved the use of Chassis Speeds from our Swerve Joystick Command to our Swerve Subsytem
    //This will be seen in setModuleStates and in driveRobotRelative (Which is just used for auto currently)
    public void setChassisSpeed( ChassisSpeeds speed){
        chassisSpeeds = speed;
    }

    //gets the chassis speeds relative the robot - J
    public Supplier<ChassisSpeeds> getRobotRelativeSpeeds(){
        return ()-> ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, getRotation2d());
    }
    //reset the odometer current theta, module positions
    public Consumer<Pose2d> resetOdometry(Supplier<Pose2d> poseFunction){
        Pose2d pose = poseFunction.get();
        m_poseEstimator.resetPosition(getRotation2d(), 
        new SwerveModulePosition[]{frontLeft.getPosition(),frontRight.getPosition(),backLeft.getPosition(),backRight.getPosition()},
         pose);
        
        return null;
    }

      public void updateOdometry() {
    m_poseEstimator.update(
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
        });


    boolean doRejectUpdate = false;
    if(LimelightConstants.useMegaTag2 == false)
    {
      LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
      
      if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
      {
        if(mt1.rawFiducials[0].ambiguity > .7)
        {
          doRejectUpdate = true;
        }
        if(mt1.rawFiducials[0].distToCamera > 3)
        {
          doRejectUpdate = true;
        }
      }
      if(mt1.tagCount == 0)
      {
        doRejectUpdate = true;
      }

      if(!doRejectUpdate)
      {
        m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
        m_poseEstimator.addVisionMeasurement(
            mt1.pose,
            mt1.timestampSeconds);
      }
    }
    else if (LimelightConstants.useMegaTag2 == true)
    {
      LimelightHelpers.SetRobotOrientation("limelight", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      if(Math.abs(gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
      {
        doRejectUpdate = true;
      }
      if(mt2.tagCount == 0)
      {
        doRejectUpdate = true;
      }
      if(!doRejectUpdate)
      {
        m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        m_poseEstimator.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds);
      }
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

     //updates odometer based on new positions which take into account turn encoder and drive encoder along with position on robot
     //m_poseEstimator.update(getRotation2d(),new SwerveModulePosition[]{frontLeft.getPosition(),frontRight.getPosition(),backLeft.getPosition(),backRight.getPosition()} );
     updateOdometry();
     SmartDashboard.putString("frontLeft", frontLeft.getPosition().toString());
     SmartDashboard.putString("frontRight", frontRight.getPosition().toString());
     SmartDashboard.putString("backLeft", backLeft.getPosition().toString());
     SmartDashboard.putString("backRight", backRight.getPosition().toString());
     SmartDashboard.putString("Robot Heading", getRotation2d().toString());

     SmartDashboard.putString("Robot Location", getPose().get().getTranslation().toString());
    
  }

  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
}

public void driveRobotRelative(ChassisSpeeds robotRelativeSpeed){
    SwerveModuleState[] desiredStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeed, getRotation2d()));
  SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

  //frontLeft.setDesiredState(desiredStates[0]);
  //frontRight.setDesiredState(desiredStates[1]);
  //backLeft.setDesiredState(desiredStates[2]);
  //backRight.setDesiredState(desiredStates[3]);
  frontLeft.setDesiredState(desiredStates[3]);
  frontRight.setDesiredState(desiredStates[1]);
  backLeft.setDesiredState(desiredStates[2]);
  backRight.setDesiredState(desiredStates[0]);
}

public void setModuleStates() {

    SwerveModuleState[] desiredStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
  SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

  //frontLeft.setDesiredState(desiredStates[0]);
  //frontRight.setDesiredState(desiredStates[1]);
  //backLeft.setDesiredState(desiredStates[2]);
  //backRight.setDesiredState(desiredStates[3]);
  frontLeft.setDesiredState(desiredStates[3]);
  frontRight.setDesiredState(desiredStates[1]);
  backLeft.setDesiredState(desiredStates[2]);
  backRight.setDesiredState(desiredStates[0]);

}

}
