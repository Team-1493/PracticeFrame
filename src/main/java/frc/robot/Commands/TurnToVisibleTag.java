// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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

public class TurnToVisibleTag extends Command {
    private final CommandSwerveDrivetrain sd;
    private final Camera camera;
    int state = 0;
    PhotonTrackedTarget target;
    double targetAngle;
    Pose2d robotPose2d;
    Pose2d goalPose= new Pose2d();
    double finalRawRotation;
    double deltaRot;

    private double followPose_Trans_kP=1;
    private double followPose_Trans_kD=.1;
    private double followPose_Rot_kP=4;
    private double followPose_Rot_kD=.5;
    private double followPose_Trans_maxV=2;
    private double followPose_Trans_maxA=4;
    private double followPose_Rot_maxV=4;
    private double followPose_Rot_maxA=3;

    ProfiledPIDController pidx, pidy, pidr;
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public TurnToVisibleTag(CommandSwerveDrivetrain m_sd, Camera m_cam) { 
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

    pidr.setTolerance(0.017);   // 1 degree
    pidx.setTolerance(0.0254);  // 1 inch
    pidy.setTolerance(0.0254);  // 1 inch


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      target = camera.getTarget(targetType.ID, 26);
      robotPose2d=sd.getState().Pose;
      if (target != null){
        targetAngle = camera.getTagAngle(26)+Math.PI;
        double targetX =robotPose2d.getX()+1/Math.tan(Math.PI*target.pitch/180.)-1/Math.tan(Math.PI*30/180);
        double targetY = robotPose2d.getY()-1/Math.sin(Math.PI*target.yaw/180.);
        pidx.setGoal(targetX);
        pidy.setGoal(targetY);
        pidr.setGoal(targetAngle);

        System.out.println("targets "+targetX+"  "+targetY+"  "+targetAngle);
        state = 1;
      }

      if (state ==1){
          double dx = -pidx.calculate(robotPose2d.getX());
          double dy = -pidy.calculate(robotPose2d.getY());
          double dr = pidr.calculate(robotPose2d.getRotation().getRadians());
          System.out.println("dx,dy,dz "+dx+"  "+dy+"  "+dr);


          sd.setControl(drive.withVelocityX(0)
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
             followPose_Trans_kP*.8,
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
