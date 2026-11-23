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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Camera extends SubsystemBase {
    PhotonCamera cam;
    PhotonCameraSim cameraSim;
    double timestamp=0;
    double numberOfTargets=0;
    // An april tag layout from the 2026 game, includes ID's and field poses for each tag
    public static final AprilTagFieldLayout kTagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    // A vision system sim labelled as "main" in NetworkTables
    VisionSystemSim visionSim = new VisionSystemSim("main");                
    TargetModel targetModel = TargetModel.kAprilTag36h11;
    CommandSwerveDrivetrain drivetrain;
    
    // This creates a default set of simulation camera properties
    SimCameraProperties cameraProp = new SimCameraProperties();
    
    // This is the transform that describes the location of the camera relative to the center of the robot
    // This is important for accurate camera results
    // and will be different for each camera -
    //  should be passed in as a parameter when creating camera instances if there are multiple cameras
    public Transform3d ROBOT_TO_CAMERA = new Transform3d(new Translation3d(0,0,0),new Rotation3d(0,0,0));

    
    public enum targetType {
        BEST, HIGHEST, LOWEST, AMBIGUITY, ID, AREA
    }

    public Camera(CommandSwerveDrivetrain m_drivetrain) {
        drivetrain = m_drivetrain;
        cam = new PhotonCamera("cam0");

        // Sample simulation camera poperties. Match to the real cam when possible
        // Resolution and Field of View
        cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
        // Approximate detection noise with average and standard deviation error in pixels.
        // Use this to simulate real world camera errors
        cameraProp.setCalibError(0.25, 0.08);
        // Set the camera image capture framerate (Note: this is limited by robot loop rate).
        cameraProp.setFPS(60);
        // The average and standard deviation in milliseconds of image data latency.
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);

        // Create the simulated camera. You pass in the actual camera
        // The simulation feeds simulated cam results to the real camera in your code
        cameraSim = new PhotonCameraSim(cam, cameraProp);        
        visionSim.addCamera(cameraSim, new Transform3d(0,0, 0, new Rotation3d(0,0,0)));
       
       // You need to add april tags to the vision system
       // Here we are adding the tags from the 2026 game
        visionSim.addAprilTags(kTagLayout);


        // Get the sim field for viewing.
        // You only need to save as a variable if using the sippet below
        // other you could just have visionSim.getDebugField(); 
        var debugField = visionSim.getDebugField();  
        
        // This part is optional. You can hover over the tags on the simulated vision field
        // and it will show the details of the tag, but the identifying numbers do not
        // match the tag fiducial ID's ! The hover info shows some random array index
        // that is different from the ID. 
        // This snippet creates another set of tag markers that have the correct ID's showing
        // Try it if you think it's worth being able to hover and see the tag ID
        // 
        // The downside is that it will create a separate entry in the object menu
        // for each new tag. If you want to turn on/off or edit color and size
        // of the tags you need to edit each one separately. The default tags
        // are grouped so you can edit them as a batch! 

        /*  
        for (var tag : kTagLayout.getTags()) {
            kTagLayout.getTagPose(tag.ID).ifPresent(pose ->
            debugField.getObject("Tag ID " + tag.ID).setPose(pose.toPose2d()));
        }
        */
        
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
    }




}