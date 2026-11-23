// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


// *** Drive to an arbitrary tag
// ** Here we use 3D targeting to calculate the camera to tag transformation.

// *** We use the best target here.
// *** Once the best tag is detected, its ID is stored and we filter by ID after

// *** No need to zero the robot orientation to  field zero
// *** The robot must be able to see the tag at the start of the command.  

// *** The goal position is continuously updated.
// *** If the robot loses view of the tag it will contoniue to the last known goal position.


// IMPORTANT TO UNDERSTAND - In 3D mode photonvision can calcuate a relative camera to tag transformation
// with no input other than the robot to camera transformation 
// (MUST BE ACCURATE - results are very sensitive to the pitch value in the robot to camera transformation).

package frc.robot.Commands;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Camera.targetType;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class driveToFixedTag3D extends Command {
    private final CommandSwerveDrivetrain sd;
    private final Camera camera;
    // state 0 - no goal pose calculated yet, state 1 = goal pose has been calculated
    int state = 0;
    int tagID = 0;
    PhotonTrackedTarget target;
    double targetAngle;
    Pose2d robotPose2d;
    Pose2d goalPose= new Pose2d();
    double finalRawRotation;
    double deltaRot;

    //  PID Controller Constants
    private double followPose_Trans_kP=3;
    private double followPose_Trans_kD=0;
    private double followPose_Rot_kP=4;
    private double followPose_Rot_kD=.5;
    private double followPose_Trans_maxV=2;
    private double followPose_Trans_maxA=4;
    private double followPose_Rot_maxV=4;
    private double followPose_Rot_maxA=3;

    ProfiledPIDController pidx, pidy, pidr;
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public driveToFixedTag3D(CommandSwerveDrivetrain m_sd, Camera m_cam) { 
        SmartDashboard.putNumber("driveTag Trans kP",followPose_Trans_kP);
        SmartDashboard.putNumber("driveTag Trans kD",followPose_Trans_kD);
        SmartDashboard.putNumber("driveTag Rot kP",followPose_Rot_kP);
        SmartDashboard.putNumber("driveTag Trans Max V",followPose_Trans_maxV);
        SmartDashboard.putNumber("driveTag Trans Max A",followPose_Trans_maxA);
        SmartDashboard.putNumber("driveTag Rot Max V",followPose_Rot_maxV);
        SmartDashboard.putNumber("driveTag Rot Max A",followPose_Rot_maxA);        

        sd = m_sd;
        camera = m_cam;
        addRequirements(sd,camera);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state=0;
    robotPose2d=sd.getState().Pose;

    
    updateControllers();
    pidr.reset(robotPose2d.getRotation().getRadians());
    pidx.reset(robotPose2d.getX());
    pidy.reset(robotPose2d.getY());
    ///t
    pidr.setTolerance(0.017);   // 1 degree
    pidx.setTolerance(0.0254);  // 1 inch
    pidy.setTolerance(0.0254);  // 1 inch



  }

  // Called every time the scheduler runs while the command is scheduled.

@Override
public void execute() {
    robotPose2d = sd.getState().Pose;

    // Pick the largest visible tag, then keep tracking that ID.
    target = state == 0
        ? camera.getTarget(targetType.AREA)
        : camera.getTarget(targetType.ID, tagID);

    if (target != null) {
        Transform3d cameraToTag = target.getBestCameraToTarget();
        var layoutTag = Camera.kTagLayout.getTagPose(target.getFiducialId());

        if (cameraToTag.getX() > 0.0 && layoutTag.isPresent()) {
            Pose3d estimatedTag = new Pose3d(robotPose2d)
                .transformBy(camera.ROBOT_TO_CAMERA)
                .transformBy(cameraToTag);

            Rotation2d tagOutward = layoutTag.get().toPose2d().getRotation();
            Translation2d goal = estimatedTag.getTranslation().toTranslation2d()
                .plus(new Translation2d(1, tagOutward));

            pidx.setGoal(goal.getX());
            pidy.setGoal(goal.getY());
            pidr.setGoal(tagOutward.plus(Rotation2d.fromDegrees(180)).getRadians());

            tagID = target.getFiducialId();
            state = 1;
        }
    }

    // Hold the last accepted goal if the tag goes out of view.
    if (state == 1) {
        sd.setControl(drive
            .withVelocityX(-pidx.calculate(robotPose2d.getX()))
            .withVelocityY(-pidy.calculate(robotPose2d.getY()))
            .withRotationalRate(pidr.calculate(robotPose2d.getRotation().getRadians())));
    }
}

  @Override
  public boolean isFinished() {
//    return (pidx.atSetpoint() && pidy.atSetpoint() && pidr.atSetpoint());
      return false;
  }

  @Override 
  public void end(boolean interrupted) {
 //   sd.Stop();
  }

  public void updateControllers(){

    readConstants();

      pidx = new ProfiledPIDController(
             followPose_Trans_kP*.5,
             0,
             followPose_Trans_kD, 
             new TrapezoidProfile.Constraints(
            followPose_Trans_maxV,
            followPose_Trans_maxA)  );
      
      pidy = new ProfiledPIDController(
             followPose_Trans_kP,
             0,
             followPose_Trans_kD, 
             new TrapezoidProfile.Constraints(
            followPose_Trans_maxV,
            followPose_Trans_maxA));

      pidr = new ProfiledPIDController(
             followPose_Rot_kP,
             0,
             followPose_Rot_kD, 
             new TrapezoidProfile.Constraints(
            followPose_Rot_maxV,
            followPose_Rot_maxA));
      
       pidr.enableContinuousInput(-Math.PI, Math.PI);

  }

  private void readConstants(){

    followPose_Trans_kP = SmartDashboard.getNumber("driveTag Trans kP",followPose_Trans_kP);
    followPose_Trans_kD  =  SmartDashboard.getNumber("driveTag Trans kD",followPose_Trans_kD);
    followPose_Rot_kP = SmartDashboard.getNumber("driveTag Rot kP",followPose_Rot_kP);
    followPose_Trans_maxV = SmartDashboard.getNumber("driveTag Trans Max V",followPose_Trans_maxV);
    followPose_Trans_maxA = SmartDashboard.getNumber("driveTag Trans Max A",followPose_Trans_maxA);
    followPose_Rot_maxV = SmartDashboard.getNumber("driveTag Rot Max V",followPose_Rot_maxV);
    followPose_Rot_maxA = SmartDashboard.getNumber("driveTag Rot Max A",followPose_Rot_maxA);        

  }

}
