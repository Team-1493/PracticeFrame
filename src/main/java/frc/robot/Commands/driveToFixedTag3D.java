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
      robotPose2d=sd.getState().Pose;

      if (state ==0)
        target = camera.getTarget(targetType.AREA);
      else
        target = camera.getTarget(targetType.ID,tagID);

      if (target != null){
        Transform3d camToTatrget3D = target.bestCameraToTarget;
        Transform2d camToTarget2D = new Transform2d(camToTatrget3D.getTranslation().toTranslation2d(),
          camToTatrget3D.getRotation().toRotation2d());
        tagID = target.fiducialId;

        Translation2d goal =  robotPose2d.getTranslation().plus(camToTarget2D.getTranslation());        

        // move goal position 1 meter away from tag, in direction normal to tag face
        double tagAngle =camera.getTagAngle(tagID); 
        Translation2d offset = new Translation2d(1,0).rotateBy(new Rotation2d(tagAngle));
       // goal = goal.plus(offset);

        pidx.setGoal(goal.getX());
        pidy.setGoal(goal.getY());
        pidr.setGoal(targetAngle);

        state = 1;
        System.out.println(tagID+"  "+goal.getX()+"  "+goal.getY());
      }

      if (state ==1){
          double dx = -pidx.calculate(robotPose2d.getX());
          double dy = -pidy.calculate(robotPose2d.getY());
          double dr = pidr.calculate(robotPose2d.getRotation().getRadians());
//        System.out.println("dx,dy,dz "+dx+"  "+dy+"  "+dr);


          sd.setControl(drive.withVelocityX(dx)
              .withVelocityY(dy) 
              .withRotationalRate(dr)); 
      
      }

      
//      if (Math.abs(pidx.getPositionError()) < 0.0254) dx=0;    
//      if (Math.abs(pidy.getPositionError()) < 0.0254) dy=0;    
//      if (Math.abs(pidr.getPositionError()) < 0.0254) dr=0;    




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

}
