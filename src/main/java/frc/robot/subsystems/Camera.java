package frc.robot.subsystems;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Camera extends SubsystemBase {
    PhotonCamera cam;
    PhotonCameraSim cameraSim;
    double timestamp=0;
    double numberOfTargets=0;
    public static final AprilTagFieldLayout kTagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    // A vision system sim labelled as "main" in NetworkTables
    VisionSystemSim visionSim = new VisionSystemSim("main");                
    TargetModel targetModel = TargetModel.kAprilTag36h11;
    SimCameraProperties cameraProp = new SimCameraProperties();
    CommandSwerveDrivetrain drivetrain;

    
    public enum targetType {
        BEST, HIGHEST, LOWEST, AMBIGUITY, ID, AREA
    }

    public Camera(CommandSwerveDrivetrain m_drivetrain) {
        drivetrain = m_drivetrain;
        cam = new PhotonCamera("cam0");
        cameraSim = new PhotonCameraSim(cam, cameraProp);
        visionSim.addCamera(cameraSim, new Transform3d(0,0, 0, new Rotation3d(0,0,0)));
        visionSim.addAprilTags(kTagLayout);
        visionSim.getDebugField();
        cameraSim.enableRawStream(true);
        cameraSim.enableProcessedStream(true);

    }



    // use to filter by target attributes, if you want to filter by ID, use getTarget2d(targetType.ID, id)
    public PhotonTrackedTarget getTarget(targetType type) {
        return getTarget(type, -1);
    }


    public PhotonTrackedTarget getTarget(targetType type,int id) {
        PhotonPipelineResult result = cam.getLatestResult();
        SmartDashboard.putNumberArray("cam0/Detected IDs",
            result.getTargets().stream()
            .mapToDouble(PhotonTrackedTarget::getFiducialId)
            .toArray());
        PhotonTrackedTarget returnTarget = null;
    
        // check if there are any targets detected, return null if not
        if (!result.hasTargets()) {
            numberOfTargets = 0;
            timestamp=0;
            printTarget(returnTarget);
            return null;
        }
        
        timestamp = result.getTimestampSeconds();
        numberOfTargets = result.getTargets().size();

        List<PhotonTrackedTarget> targets = result.getTargets();
        
        // default return target is bestTarget, unless we are looking for a specific ID
        if (type != targetType.ID )
            returnTarget = result.getBestTarget(); 
        
        // Loop through all targets and find the one that matches the criteria
        // If we are looking for a specific ID, we will return the first target that matches that ID
        // If we are looking fore the bestr target, don't do anything, we already have the best target    
        for (PhotonTrackedTarget target : targets) {
            switch (type) {
                case BEST:
                    break;
                case ID:
                    if (target.getFiducialId() == id) {
                        printTarget(target);
                        return target;
                    }
                    break;
                case HIGHEST:
                    if (target.pitch > returnTarget.pitch) {
                        returnTarget = target;
                    }
                    break;
                case LOWEST:
                    if (target.pitch < returnTarget.pitch) {
                        returnTarget = target;
                    }
                    break;
                case AMBIGUITY:
                    if (target.getPoseAmbiguity() < returnTarget.getPoseAmbiguity()) {
                        returnTarget = target;
                    }
                    break;
                case AREA:
                    if (target.getArea() > returnTarget.getArea()) {
                        returnTarget = target;
                    }
                    break;
            }
        }
        return returnTarget;
    }

    public void  printTarget(PhotonTrackedTarget target) {
        if (target != null) {
            SmartDashboard.putNumber("Target ID", target.getFiducialId());
            SmartDashboard.putNumber("Target Pitch", target.pitch);
            SmartDashboard.putNumber("Target Yaw", target.yaw);
            SmartDashboard.putNumber("Target Area", target.getArea());  
            SmartDashboard.putNumber("Target Pose Ambiguity", target.getPoseAmbiguity());
        } else {
            SmartDashboard.putNumber("Target ID", -1);
            SmartDashboard.putNumber("Target Pitch", 999);
            SmartDashboard.putNumber("Target Yaw", 999);
            SmartDashboard.putNumber("Target Area", 999);
            SmartDashboard.putNumber("Target Pose Ambiguity", 999);  
        }

    }


    
    public double getTagAngle(int id){
        double angle= kTagLayout.getTags().get(id-1).pose.getRotation().getZ();
        SmartDashboard.putNumber("Target Rot", angle);
        return angle;
    }

    public double getTagHeight(int id){
        double height= kTagLayout.getTags().get(id-1).pose.getZ();
        SmartDashboard.putNumber("Target Height", height);
        return height;
    }

    public double getTagX(int id){
        double height= kTagLayout.getTags().get(id-1).pose.getX();
        SmartDashboard.putNumber("Target Height", height);
        return height;
    }


    public double getTagY(int id){
        double height= kTagLayout.getTags().get(id-1).pose.getY();
        SmartDashboard.putNumber("Target Height", height);
        return height;
    }    


    @Override
    public void simulationPeriodic() {
        // Update the camera simulation
        visionSim.update(drivetrain.getState().Pose);
        PhotonTrackedTarget target = getTarget(targetType.ID,26);
        if (target != null){
            printTarget(target);
            getTagAngle(target.fiducialId);
        }
    }




}