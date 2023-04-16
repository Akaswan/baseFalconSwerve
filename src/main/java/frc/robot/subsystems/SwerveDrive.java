// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SPI;
import frc.robot.NavX.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

import frc.robot.utilities.GeometryUtils;
import frc.robot.utilities.OdometryUtility;
import frc.robot.utilities.SwerveModuleConstants;

public class SwerveDrive extends SubsystemBase {
  private boolean isFieldRelative = true;
  public static final Translation2d[] kModuleTranslations = {
      new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
      new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
      new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
      new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)
  };

  public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(kModuleTranslations);

  //https://github.com/Thunderstamps/navx2workaround.git
    //link to import navX vendordeps
  private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte)50);

  private SwerveDriveOdometry m_odometry;

  private OdometryUtility m_aprilCamera;

  private static final double kMaxRotationRadiansPerSecond = Math.PI * 2.0; // Last year 11.5?
  private static final boolean invertGyro = false;


  private SwerveModuleState[] moduleStates;

  private SwerveModule[] mSwerveMods;


  /**
   * <h3>SwerveDrive</h3>
   * 
   * Builds the swerve drivebase from the swerve module class
   * 
   * @param frontLeftModuleConstants
   * @param frontRightModuleConstants
   * @param backLeftModuleConstants
   * @param backRightModuleConstants
   */
  public SwerveDrive(SwerveModuleConstants frontLeftModuleConstants, SwerveModuleConstants frontRightModuleConstants,
      SwerveModuleConstants backLeftModuleConstants, SwerveModuleConstants backRightModuleConstants) {

    mSwerveMods = new SwerveModule[] {
        new SwerveModule(0, frontLeftModuleConstants),
        new SwerveModule(1, frontRightModuleConstants),
        new SwerveModule(2, backLeftModuleConstants),
        new SwerveModule(3, backRightModuleConstants)
    };

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
        * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
        */
        Timer.delay(1.0);
        resetAngleToAbsolute();
        
        m_odometry = new SwerveDriveOdometry(
                kDriveKinematics,
                getHeadingRotation2d(),
                getModulePositions(),
                new Pose2d());
    
        m_navx.zeroYaw();
        
        m_aprilCamera = new OdometryUtility(kDriveKinematics, getHeadingRotation2d(), getModulePositions(), getPoseMeters());
  }

  public Pose2d getPose() {
    return m_aprilCamera.getPose();
  }

  public void zeroGyroscope() {
    m_navx.zeroYaw();
  }

  public void drive(
      double throttle,
      double strafe,
      double rotation,
      boolean isOpenLoop) {

    throttle = throttle * MAX_METERS_PER_SECOND;
    strafe = strafe * MAX_METERS_PER_SECOND;
    rotation = rotation * kMaxRotationRadiansPerSecond;
    
    ChassisSpeeds chassisSpeeds = isFieldRelative
      ? ChassisSpeeds.fromFieldRelativeSpeeds(throttle, strafe, rotation, getHeadingRotation2d())
      : new ChassisSpeeds(throttle, strafe, rotation);

    moduleStates = kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    setSwerveModuleStates(moduleStates, isOpenLoop);
    chassisSpeeds = correctForDynamics(chassisSpeeds);
  }
  
  public void drive(ChassisSpeeds speeds){
    moduleStates = kDriveKinematics.toSwerveModuleStates(speeds);
    setSwerveModuleStates(moduleStates, true);
    speeds = correctForDynamics(speeds);
  }
  public void setSwerveModuleStates(SwerveModuleState[] states, boolean isOpenLoop) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_METERS_PER_SECOND);

    for (int i = 0; i < mSwerveMods.length; i++) {
      SwerveModule module = mSwerveMods[i];
      states[i] = new SwerveModuleState(states[i].speedMetersPerSecond, states[i].angle);
      module.setDesiredState(states[i], isOpenLoop);
    }
  } 

  public Rotation2d getHeadingRotation2d() {
    return getYaw();
  }

  public Pose2d getPoseMeters() {
    return m_odometry.getPoseMeters();
  }

  public SwerveModule getSwerveModule(int moduleNumber) {
    return mSwerveMods[moduleNumber];
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
        mSwerveMods[0].getState(),
        mSwerveMods[1].getState(),
        mSwerveMods[2].getState(),
        mSwerveMods[3].getState()
    };
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        mSwerveMods[0].getPosition(),
        mSwerveMods[1].getPosition(),
        mSwerveMods[2].getPosition(),
        mSwerveMods[3].getPosition()
    };
  }

  public void updateOdometry() {
    m_odometry.update(getHeadingRotation2d(), getModulePositions());
  }

  public void updateOdometryWithVision(Pose2d estimatedPose) {
    if(estimatedPose.equals(new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(0.0)))){
      m_odometry.resetPosition(getHeadingRotation2d(), getModulePositions(), estimatedPose);
    }
    else{
      
    }
	}

  public static SwerveDriveKinematics getSwerveKinematics() {
    return kDriveKinematics;
  }

  public void resetOdometry(Pose2d initialPose) {
    // resets the position of swerveEstimatorPoses
    m_aprilCamera.resetPosition(getYaw(), getModulePositions(), initialPose);
    m_odometry.resetPosition(getYaw(), getModulePositions(), initialPose);
  }

   /**
   * Correction for swerve second order dynamics issue. Borrowed from 254:
   * https://github.com/Team254/FRC-2022-Public/blob/main/src/main/java/com/team254/frc2022/subsystems/Drive.java#L325
   * Discussion:
   * https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964
   */
  private static ChassisSpeeds correctForDynamics(ChassisSpeeds originalSpeeds) {
    final double LOOP_TIME_S = 0.02;
    Pose2d futureRobotPose =
        new Pose2d(
            originalSpeeds.vxMetersPerSecond * LOOP_TIME_S,
            originalSpeeds.vyMetersPerSecond * LOOP_TIME_S,
            Rotation2d.fromRadians(originalSpeeds.omegaRadiansPerSecond * LOOP_TIME_S));
    Twist2d twistForPose = GeometryUtils.log(futureRobotPose);
    ChassisSpeeds updatedSpeeds =
        new ChassisSpeeds(
            twistForPose.dx / LOOP_TIME_S,
            twistForPose.dy / LOOP_TIME_S,
            twistForPose.dtheta / LOOP_TIME_S);
    return updatedSpeeds;
  }

  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(-(m_navx.getYaw() + 180));
  }

  public float getYawDegrees(){
    return m_navx.getYaw();
  }

  public Rotation2d getPitch() {
    double[] ypr = new double[3];
    ypr[0] = m_navx.getYaw();
    ypr[1] = m_navx.getRoll();
    ypr[2] = m_navx.getPitch();
    return (invertGyro) ? Rotation2d.fromDegrees(360 - ypr[1]) : Rotation2d.fromDegrees(ypr[1]);
  }

  public void setSwerveModuleStates(SwerveModuleState[] states) {
    setSwerveModuleStates(states, false);
  }

  /**
   * Resets each SwerveModule to the absolute position.
   */
  public void resetAngleToAbsolute() {
    for (SwerveModule mod : mSwerveMods) {
      mod.resetAngleToAbsolute();
    }
  }

  /**
   * Sets the origin for April Tags to be used by robot not using FMS ie practice or simulation
   */
  public void setOriginBasedOnAlliance() {
    m_aprilCamera.setOriginBasedOnAlliance();
  }

  @Override
  public void periodic() {
    updateOdometry();

    SmartDashboard.putNumber("gyro rotation degrees", m_navx.getYaw());
    m_aprilCamera.updateCameraPos(getHeadingRotation2d(), getModulePositions(), getPoseMeters());
  }
}
