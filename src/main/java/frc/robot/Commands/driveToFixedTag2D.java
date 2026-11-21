// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


// *** Drive to a tag of known height off the ground and known orientation.
// ** Here we specify a tag number and use the field tag layout to determine the tag position
// *** Only the tag height and rotation are used from the layout!  
// *** We use the tag height, rotation, and  pitch and yaw from the camnera
// *** to calculate the relative camera to tag transformation

// *** The robot orientation MUST be zerod to the same field zero that the tag orientation is referenced to,
// *** The robot must be able to see the tag at the start of the command.  

// *** The goal position is continuously updated.
// *** If the robot loses view of the tag it will contoniue to the last known goal position.
// *** This command uses pitch and yaw from 2D targeting, camera can run in 2D or 3D mode. 

// IMPORTANT TO UNDERSTAND - This works in 2D because the tag height and tag orientation are known|,
// This lets us calculate the camera to tag transformation from only pitch and yaw
// If the tag height and rotation are not known, 3D targeting is needed to get the camera to tag transformation!!!

package frc.robot.Commands;import frc.robot.subsystems.Camera;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Camera.targetType;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class driveToFixedTag2D extends Command {
    private final CommandSwerveDrivetrain sd;
    private final Camera camera;
    // state 0 - no goal pose calculated yet, state 1 = goal pose has been calculated
    int state = 0;
    int tagID = 25;
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

  public driveToFixedTag2D(CommandSwerveDrivetrain m_sd, Camera m_cam) { 
      sd = m_sd;
      camera = m_cam;
      SmartDashboard.putNumber("DriveToTag ID", tagID);
      addRequirements(sd,camera);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state=0;
    robotPose2d=sd.getState().Pose;
    tagID = (int)SmartDashboard.getNumber("DriveToTag ID", 26);

    
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

      
      target = camera.getTarget(targetType.ID, tagID);
      robotPose2d=sd.getState().Pose;
      if (target != null){
        double tagAngle =camera.getTagAngle(tagID); 
        // tag angle points out from tag face, robot should end up facing 180 degrees from tag angle
        targetAngle = tagAngle+Math.PI;
        double tagx = camera.getTagHeight(26)/Math.tan(Math.PI*target.pitch/180.);
        double tagy = -tagx*Math.tan(target.yaw*Math.PI/180.);
        Translation2d goal =  robotPose2d.getTranslation().plus(
        new Translation2d(tagx,tagy).rotateBy(robotPose2d.getRotation()));

        // move goal position 1 meter away from tag, in direction normal to tag face
        Translation2d offset = new Translation2d(1,0).rotateBy(new Rotation2d(tagAngle));
        goal = goal.plus(offset);

        pidx.setGoal(goal.getX());
        pidy.setGoal(goal.getY());
        pidr.setGoal(targetAngle);

      state = 1;
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
