package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;


public class Robot extends TimedRobot {

    // ============================================================
    // FIELD LAYOUT CONFIGURATION
    // ============================================================

    private static final String FIELD_2024 =
            "2024 - Crescendo";

    private static final String FIELD_2025_WELDED =
            "2025 - Reefscape Welded";

    private static final String FIELD_2025_ANDYMARK =
            "2025 - Reefscape AndyMark";

    private static final String FIELD_2026_WELDED =
            "2026 - Rebuilt Welded";

    private static final String FIELD_2026_ANDYMARK =
            "2026 - Rebuilt AndyMark";


    // ============================================================
    // DEFAULT FIELD
    // ============================================================

    private String DEFAULT_FIELD =
            FIELD_2025_WELDED;


    // ============================================================
    // CONTROL
    // ============================================================

    private Joystick m_joystick;

    private static final int BUTTON_SIDES = 2;
    private static final int BUTTON_X = 3;
    private static final int BUTTON_V = 4;
    private static final int BUTTON_Z = 1; // Button 1 on WPILib virtual keyboard (Z key)
    private static final int BUTTON_C = 10;


    // ============================================================
    // SHUFFLEBOARD
    // ============================================================

    private ShuffleboardTab m_tab;

    private GenericEntry m_fieldEntry;

    private GenericEntry m_capturedTagsEntry;
    private GenericEntry m_polygonSidesEntry;
    private GenericEntry m_totalTagsRadarEntry;
    private GenericEntry m_referenceTagEntry;
    private GenericEntry m_alignmentModeEntry;

    private GenericEntry m_closestTagEntry;
    private GenericEntry m_closestTagDistanceEntry;

    private GenericEntry m_closestTagYawEntry;
    private GenericEntry m_closestTagPitchEntry;

    private GenericEntry m_fieldTagRollEntry;
    private GenericEntry m_fieldTagPitchEntry;
    private GenericEntry m_fieldTagYawEntry;

    private GenericEntry m_targetTagEntry;
    private GenericEntry m_lockedTargetEntry;
    private GenericEntry m_lostFramesEntry;

    private GenericEntry m_cameraConnectedEntry;
    private GenericEntry m_scanningEntry;

    private GenericEntry m_tangentLinesCountEntry;

    private GenericEntry m_targetSidesEntry;


    // ============================================================
    // NETWORKTABLES
    // ============================================================

    private StructPublisher<Pose2d> m_robotPosePublisher;

    private StructArrayPublisher<Pose2d>
            m_polygonVisualPublisher;


    // NEW:
    // Real vertices calculated by tangent lines intersection.
    private StructArrayPublisher<Pose2d>
            m_polygonEdgesPublisher;


    // ============================================================
    // TAG ORIENTATION LINE
    // ============================================================

    private StructPublisher<Pose2d>
            m_tagOrientationArrowPublisher;

    private StructArrayPublisher<Pose2d>
            m_tagOrientationLinePublisher;


    // ============================================================
    // TAG TANGENT LINE
    // ============================================================

    private StructArrayPublisher<Pose2d>
            m_tagTangentLinePublisher;


    // ============================================================
    // PHOTONVISION
    // ============================================================

    private PhotonCamera m_camera;


    // ============================================================
    // SIMULATION
    // ============================================================

    private VisionSystemSim m_visionSim;

    private PhotonCameraSim m_cameraSim;

    private AprilTagFieldLayout m_fieldLayout;


    // ============================================================
    // CURRENT FIELD
    // ============================================================

    private String m_loadedFieldName = null;


    // ============================================================
    // OBSTACLE COLLISION SYSTEM
    // ============================================================

    private static final String OBSTACLES_FILE_PATH =
            "C:\\Downloads\\map_obstacles.txt";

    private static final double ROBOT_LENGTH = 0.85;
    private static final double ROBOT_WIDTH = 0.85;
    private static final double COLLISION_EPSILON = 1e-9;

    // Each obstacle is stored as [p1, p2], where each point is [x, y].
    private final List<double[][]> m_obstacles =
            new ArrayList<>();

    private final Translation2d m_frontLeft =
            new Translation2d(ROBOT_LENGTH / 2.0, ROBOT_WIDTH / 2.0);

    private final Translation2d m_frontRight =
            new Translation2d(ROBOT_LENGTH / 2.0, -ROBOT_WIDTH / 2.0);

    private final Translation2d m_backRight =
            new Translation2d(-ROBOT_LENGTH / 2.0, -ROBOT_WIDTH / 2.0);

    private final Translation2d m_backLeft =
            new Translation2d(-ROBOT_LENGTH / 2.0, ROBOT_WIDTH / 2.0);

    private boolean m_collisionBlockedLastCycle = false;



    // ============================================================
    // ROBOT POSE
    // ============================================================

    private Pose2d m_robotPose =
            new Pose2d(
                    2.53,
                    4.02,
                    new Rotation2d(0));

    // Relative Forward direction for Field-Centric (Reset with Z key)
    private Rotation2d m_forwardHeading = new Rotation2d(0.0);


    // ============================================================
    // SCANNING
    // ============================================================

    private boolean m_isScanning = false;

    private boolean m_lastActivationButtonState = false;


    // ============================================================
    // BUTTONS
    // ============================================================

    private boolean m_lastSidesButtonState = false;


    // ============================================================
    // SIDES
    // ============================================================

    private int m_targetSides = 1;

    private static final int MIN_TARGET_SIDES = 1;

    private static final int MAX_TARGET_SIDES = 7;

    private int m_lastWrittenSides = 1;


    // ============================================================
    // DETECTED TAGS
    // ============================================================

    private final List<Integer>
            m_detectedUniqueTagIDs =
            new ArrayList<>();


    // ============================================================
    // POLYGON ANGLES
    // ============================================================

    private final List<Double>
            m_scannedUniqueAngles =
            new ArrayList<>();

    private final double
            ANGULAR_TOLERANCE_RAD =
            Math.toRadians(15.0);


    // ============================================================
    // POLYGON CENTER
    // ============================================================

    private final double SCANNER_CENTER_X = 4.23;

    private final double SCANNER_CENTER_Y = 4.02;


    // ============================================================
    // ORBIT
    // ============================================================

    private final double AUTONOMOUS_ORBIT_RADIUS = 1.70;

    private final double ORBIT_SPEED = 1.2;


    // ============================================================
    // POSITION IN FRONT OF TAG
    // ============================================================

    private final double DISTANCE_CLOSE_TO_TAG = 0.15;


    // ============================================================
    // LINES
    // ============================================================

    private static final double FIELD_LINE_MARGIN_METERS = 1.0;

    private static final double VERTICAL_ANGLE_EPSILON = 1e-9;


    // ============================================================
    // V ROTATION
    // ============================================================

    private boolean m_rotatingWithV = false;

    private double m_rotationTargetV = 0.0;

    private final double TURN_TOLERANCE_V =
            Math.toRadians(1.0);


    // ============================================================
    // ORBIT TRACKING
    // ============================================================

    private double m_lastRobotAngleToCenter = 0.0;

    private double m_totalAccumulatedAngle = 0.0;


    // ============================================================
    // DIAGNOSTICS
    // ============================================================

    private int m_lastClosestTagID = -1;

    private double m_lastClosestTagDistance =
            Double.NaN;

    private double m_lastClosestTagYaw =
            Double.NaN;

    private double m_lastClosestTagPitch =
            Double.NaN;


    // ============================================================
    // ORIENTATION
    // ============================================================

    private double m_lastTagRollField =
            Double.NaN;

    private double m_lastTagPitchField =
            Double.NaN;

    private double m_lastTagYawField =
            Double.NaN;


    // ============================================================
    // ROBUST SELECTOR
    // ============================================================

    private int m_lockedTargetID = -1;

    private double m_lockedTargetDistance =
            Double.NaN;

    private double m_lockedTargetYaw =
            Double.NaN;

    private double m_lockedTargetPitch =
            Double.NaN;

    private double m_lockedTargetLastSeenTimestamp =
            -1.0;

    private int m_lockedTargetLostFrames = 0;


    private static final double TARGET_MEMORY_SECONDS = 1.00;

    private static final double TARGET_SWITCH_MARGIN_METERS = 0.25;

    private static final double TARGET_SWITCH_RATIO = 0.70;


    // ============================================================
    // TANGENT LINES
    // ============================================================

    private final List<TangentLine>
            m_tangentLines =
            new ArrayList<>();

    private int m_tangentLinesCount = 0;


    // ============================================================
    // POLYGON VERTICES
    // ============================================================

    private final List<PolygonPoint>
            m_polygonVertices =
            new ArrayList<>();


    // ============================================================
    // LINE STRUCTURE
    // ============================================================

    private static class TangentLine {

        int tagID;

        double x;
        double y;

        double angle;

        boolean isVertical;

        double slope;
        double intercept;


        TangentLine(
                int tagID,
                double x,
                double y,
                double angle) {

            this.tagID = tagID;

            this.x = x;

            this.y = y;

            this.angle = angle;


            double cosine =
                    Math.cos(angle);


            if (Math.abs(cosine)
                    < VERTICAL_ANGLE_EPSILON) {

                isVertical = true;

                slope = Double.NaN;

                intercept = x;

            } else {

                isVertical = false;

                slope =
                        Math.tan(angle);

                intercept =
                        y - slope * x;
            }
        }
    }


    // ============================================================
    // POINT STRUCTURE
    // ============================================================

    private static class PolygonPoint {

        double x;
        double y;

        PolygonPoint(
                double x,
                double y) {

            this.x = x;
            this.y = y;
        }
    }


    // ============================================================
    // ROBOT INIT
    // ============================================================

    @Override
    public void robotInit() {

        configureLogging();

        // ========================================================
        // INITIAL WRITE TEST (ON ROBOT INITIALIZATION)
        // ========================================================
        writeRandomTextOnStartup();


        // ========================================================
        // JOYSTICK
        // ========================================================

        m_joystick =
                new Joystick(0);


        // ========================================================
        // SHUFFLEBOARD
        // ========================================================

        m_tab =
                Shuffleboard.getTab("Vision");


        // ========================================================
        // FIELD
        // ========================================================

        m_fieldEntry =
                m_tab.add(
                        "Field Layout",
                        DEFAULT_FIELD)
                        .withPosition(0, 0)
                        .withSize(4, 1)
                        .getEntry();


        // ========================================================
        // SHUFFLEBOARD REMAINDER
        // ========================================================

        m_capturedTagsEntry =
                m_tab.add(
                        "Tags In View",
                        0)
                        .getEntry();


        m_polygonSidesEntry =
                m_tab.add(
                        "Polygon Sides",
                        0)
                        .getEntry();


        m_targetSidesEntry =
                m_tab.add(
                        "Target Sides",
                        m_targetSides)
                        .getEntry();


        m_totalTagsRadarEntry =
                m_tab.add(
                        "Unique Tags",
                        0)
                        .getEntry();


        m_referenceTagEntry =
                m_tab.add(
                        "Reference Tag",
                        -1)
                        .getEntry();


        m_alignmentModeEntry =
                m_tab.add(
                        "X Mode",
                        false)
                        .getEntry();


        m_closestTagEntry =
                m_tab.add(
                        "Closest Tag",
                        -1)
                        .getEntry();


        m_closestTagDistanceEntry =
                m_tab.add(
                        "Closest Tag Distance",
                        -1.0)
                        .getEntry();


        m_closestTagYawEntry =
                m_tab.add(
                        "Camera Tag Yaw",
                        0.0)
                        .getEntry();


        m_closestTagPitchEntry =
                m_tab.add(
                        "Camera Tag Pitch",
                        0.0)
                        .getEntry();


        m_fieldTagRollEntry =
                m_tab.add(
                        "FIELD Tag Roll",
                        0.0)
                        .getEntry();


        m_fieldTagPitchEntry =
                m_tab.add(
                        "FIELD Tag Pitch",
                        0.0)
                        .getEntry();


        m_fieldTagYawEntry =
                m_tab.add(
                        "FIELD Tag Yaw",
                        0.0)
                        .getEntry();


        m_targetTagEntry =
                m_tab.add(
                        "Locked Target Tag",
                        -1)
                        .getEntry();


        m_lockedTargetEntry =
                m_tab.add(
                        "Target Locked",
                        false)
                        .getEntry();


        m_lostFramesEntry =
                m_tab.add(
                        "Lost Frames",
                        0)
                        .getEntry();


        m_cameraConnectedEntry =
                m_tab.add(
                        "Camera Connected",
                        false)
                        .getEntry();


        m_scanningEntry =
                m_tab.add(
                        "Scanning",
                        false)
                        .getEntry();


        m_tangentLinesCountEntry =
                m_tab.add(
                        "Tangent Lines Count",
                        0)
                        .getEntry();


        // ========================================================
        // NETWORKTABLES
        // ========================================================

        NetworkTableInstance nt =
                NetworkTableInstance.getDefault();


        m_robotPosePublisher =
                nt.getStructTopic(
                        "SmartDashboard/RobotPose",
                        Pose2d.struct)
                        .publish();


        m_polygonVisualPublisher =
                nt.getStructArrayTopic(
                        "SmartDashboard/ScannedPolygonPerimeter",
                        Pose2d.struct)
                        .publish();


        // NEW
        m_polygonEdgesPublisher =
                nt.getStructArrayTopic(
                        "SmartDashboard/ScannedPolygonEdges",
                        Pose2d.struct)
                        .publish();


        m_tagOrientationArrowPublisher =
                nt.getStructTopic(
                        "SmartDashboard/TagOrientationArrow",
                        Pose2d.struct)
                        .publish();


        m_tagOrientationLinePublisher =
                nt.getStructArrayTopic(
                        "SmartDashboard/TagOrientationLine",
                        Pose2d.struct)
                        .publish();


        m_tagTangentLinePublisher =
                nt.getStructArrayTopic(
                        "SmartDashboard/TagTangentLine",
                        Pose2d.struct)
                        .publish();


        // ========================================================
        // PHOTONVISION
        // ========================================================

        m_camera =
                new PhotonCamera(
                        "CamaraPhoton"); // Camera name kept as defined in setup


        // ========================================================
        // LOAD FIELD
        // ========================================================

        loadField();
        loadObstaclesFromFile(m_loadedFieldName);


        // ========================================================
        // SIMULATION
        // ========================================================

        configureVisionSimulation();
    }


    // ============================================================
    // AUTOMATIC INITIAL WRITE (BOOT)
    // ============================================================

    private void writeRandomTextOnStartup() {
        String filePath = "C:\\Downloads\\map_obstacles.txt";
        try {
            File file = new File(filePath);
            File parentDir = file.getParentFile();

            if (parentDir != null && !parentDir.exists()) {
                parentDir.mkdirs(); // Creates C:\Downloads if it doesn't exist
            }

            // 'true' enables append (addition without deleting previous content)
            try (FileWriter writer = new FileWriter(file, true)) {
                writer.write("/* RANDOM_STARTUP_TEST */\n");
                writer.write("[\n");
                writer.write("  [1.234567, 9.876543, [0.000000, 5.000000], [1.000000, 6.000000]]\n");
                writer.write("]\n");
                writer.write("/* END_RANDOM_TEST */\n\n");

                System.out.println("[INIT] File initialized and random text written automatically to: " + filePath);
            }
        } catch (IOException e) {
            System.out.println("[INIT] Error writing the initial file: " + e.getMessage());
        }
    }


    // ============================================================
    // LOAD FIELD
    // ============================================================

    private void loadField() {

        String selectedField =
                m_fieldEntry.getString(
                        DEFAULT_FIELD);


        if (selectedField == null
                || selectedField.isBlank()) {

            selectedField =
                    DEFAULT_FIELD;
        }


        try {

            switch (selectedField) {

                case FIELD_2024:

                    m_fieldLayout =
                            AprilTagFieldLayout.loadField(
                                    AprilTagFields.k2024Crescendo);

                    break;


                case FIELD_2025_WELDED:

                    m_fieldLayout =
                            AprilTagFieldLayout.loadField(
                                    AprilTagFields.k2025ReefscapeWelded);

                    break;


                case FIELD_2025_ANDYMARK:

                    m_fieldLayout =
                            AprilTagFieldLayout.loadField(
                                    AprilTagFields.k2025ReefscapeAndyMark);

                    break;


                case FIELD_2026_WELDED:

                    m_fieldLayout =
                            AprilTagFieldLayout.loadField(
                                    AprilTagFields.k2026RebuiltWelded);

                    break;


                case FIELD_2026_ANDYMARK:

                    m_fieldLayout =
                            AprilTagFieldLayout.loadField(
                                    AprilTagFields.k2026RebuiltAndymark);

                    break;


                default:

                    System.out.println(
                            "[FIELD] Unrecognized field: "
                            + selectedField);

                    m_fieldLayout = null;

                    m_loadedFieldName = null;

                    return;
            }


            if (m_fieldLayout != null) {

                m_loadedFieldName =
                        selectedField;


                System.out.println("");

                System.out.println(
                        "==========================================");

                System.out.println(
                        "[FIELD] MAP LOADED");

                System.out.println(
                        "[FIELD] "
                        + selectedField);

                System.out.println(
                        "[FIELD] Length = "
                        + String.format(
                                "%.3f",
                                m_fieldLayout.getFieldLength())
                        + " m");

                System.out.println(
                        "[FIELD] Width = "
                        + String.format(
                                "%.3f",
                                m_fieldLayout.getFieldWidth())
                        + " m");

                System.out.println(
                        "[FIELD] AprilTags = "
                        + m_fieldLayout.getTags().size());

                System.out.println(
                        "==========================================");

                System.out.println("");
            }

        } catch (Exception e) {

            System.out.println(
                    "[FIELD] ERROR loading layout: "
                    + e.getMessage());

            m_fieldLayout = null;

            m_loadedFieldName = null;
        }
    }


    // ============================================================
    // LIVE MAP CHANGE
    // ============================================================

    private void checkMapChange() {

        if (m_fieldEntry == null) {
            return;
        }


        String selectedField =
                m_fieldEntry.getString(
                        DEFAULT_FIELD);


        if (selectedField == null
                || selectedField.isBlank()) {

            return;
        }


        if (m_loadedFieldName == null) {

            loadField();
            loadObstaclesFromFile(m_loadedFieldName);

            updateSimulationWithNewField();

            return;
        }


        if (selectedField.equals(
                m_loadedFieldName)) {

            return;
        }


        System.out.println("");

        System.out.println(
                "==========================================");

        System.out.println(
                "[FIELD] MAP CHANGE DETECTED");

        System.out.println(
                "[FIELD] Previous = "
                + m_loadedFieldName);

        System.out.println(
                "[FIELD] New = "
                + selectedField);

        System.out.println(
                "==========================================");


        m_isScanning = false;

        m_rotatingWithV = false;


        clearTarget();

        clearTangentLines();

        clearPolygonVertices();


        m_detectedUniqueTagIDs.clear();

        m_scannedUniqueAngles.clear();

        m_totalAccumulatedAngle = 0.0;


        m_polygonSidesEntry.setInteger(0);

        m_totalTagsRadarEntry.setInteger(0);

        m_referenceTagEntry.setInteger(-1);


        m_polygonVisualPublisher.set(
                new Pose2d[0]);


        loadField();
        loadObstaclesFromFile(m_loadedFieldName);


        updateSimulationWithNewField();


        System.out.println(
                "[FIELD] Change completed.");

        System.out.println("");
    }


    // ============================================================
    // CHECK WRITTEN SIDES
    // ============================================================

    private void checkWrittenSides() {

        if (m_targetSidesEntry == null) {
            return;
        }


        int writtenSides =
                (int) Math.round(
                        m_targetSidesEntry.getDouble(
                                m_targetSides));


        if (writtenSides < MIN_TARGET_SIDES) {

            writtenSides =
                    MIN_TARGET_SIDES;
        }


        if (writtenSides > MAX_TARGET_SIDES) {

            writtenSides =
                    MAX_TARGET_SIDES;
        }


        if (writtenSides ==
                m_lastWrittenSides) {

            return;
        }


        m_lastWrittenSides =
                writtenSides;

        m_targetSides =
                writtenSides;


        m_targetSidesEntry.setDouble(
                m_targetSides);


        clearTangentLines();

        clearPolygonVertices();

        m_scannedUniqueAngles.clear();

        m_detectedUniqueTagIDs.clear();

        m_totalAccumulatedAngle = 0.0;


        m_polygonSidesEntry.setInteger(0);

        m_totalTagsRadarEntry.setInteger(0);

        m_referenceTagEntry.setInteger(-1);


        m_polygonVisualPublisher.set(
                new Pose2d[0]);


        System.out.println(
                "[POLYGON] Value received = "
                + m_targetSides);

        System.out.println(
                "[POLYGON] Verification + 1 = "
                + (m_targetSides + 1));
    }


    // ============================================================
    // UPDATE SIMULATION
    // ============================================================

    private void updateSimulationWithNewField() {

        if (!Robot.isSimulation()) {
            return;
        }


        if (m_visionSim == null) {
            return;
        }


        if (m_fieldLayout == null) {
            return;
        }


        m_visionSim.clearAprilTags();


        m_visionSim.addAprilTags(
                m_fieldLayout);


        System.out.println(
                "[SIM] AprilTags updated.");

        System.out.println(
                "[SIM] Tags = "
                + m_fieldLayout.getTags().size());
    }


    // ============================================================
    // LOGGING
    // ============================================================

    private void configureLogging() {

        Logger rootLogger =
                Logger.getLogger("");

        rootLogger.setLevel(
                Level.SEVERE);

        for (var handler :
                rootLogger.getHandlers()) {

            handler.setLevel(
                    Level.SEVERE);
        }
    }


    // ============================================================
    // VISION SIMULATION
    // ============================================================

    private void configureVisionSimulation() {

        if (!Robot.isSimulation()) {
            return;
        }


        if (m_fieldLayout == null) {
            return;
        }


        m_visionSim =
                new VisionSystemSim("main");


        m_visionSim.addAprilTags(
                m_fieldLayout);


        SimCameraProperties cameraProp =
                new SimCameraProperties();


        cameraProp.setCalibration(
                1280,
                720,
                Rotation2d.fromDegrees(75));


        cameraProp.setCalibError(
                0.35,
                0.10);


        cameraProp.setFPS(30);

        cameraProp.setAvgLatencyMs(35);

        cameraProp.setLatencyStdDevMs(8);


        m_cameraSim =
                new PhotonCameraSim(
                        m_camera,
                        cameraProp);


        Transform3d robotToCamera =
                new Transform3d(
                        new Translation3d(
                                0.30,
                                0.0,
                                0.75),

                        new Rotation3d(
                                0.0,
                                Math.toRadians(-5.0),
                                0.0));


        m_visionSim.addCamera(
                m_cameraSim,
                robotToCamera);


        m_cameraSim.enableRawStream(true);

        m_cameraSim.enableProcessedStream(true);

        m_cameraSim.enableDrawWireframe(true);
    }


    // ============================================================
    // ROBOT PERIODIC
    // ============================================================

    @Override
    @SuppressWarnings("removal")
    public void robotPeriodic() {

        checkMapChange();

        checkWrittenSides();


        m_robotPosePublisher.set(
                m_robotPose);


        m_cameraConnectedEntry.setBoolean(
                m_camera.isConnected());


        m_scanningEntry.setBoolean(
                m_isScanning);


        PhotonPipelineResult result =
                m_camera.getLatestResult();


        if (result.hasTargets()) {

            m_capturedTagsEntry.setInteger(
                    result.getTargets().size());

        } else {

            m_capturedTagsEntry.setInteger(
                    0);
        }


        processVision(result);

        processSidesButton();

        processButtonC();


        if (m_joystick.getRawButtonPressed(
                BUTTON_X)) {

            m_isScanning = false;

            m_rotatingWithV = false;


            moveDirectlyToClosestTag();


            m_alignmentModeEntry.setBoolean(
                    false);
        }


        if (m_joystick.getRawButtonPressed(
                BUTTON_V)) {

            m_isScanning = false;

            m_rotatingWithV = true;


            double currentRotation =
                    m_robotPose
                            .getRotation()
                            .getRadians();


            m_rotationTargetV =
                    normalizeAngle(
                            currentRotation
                            + Math.toRadians(30.0)); // 30 degrees rotation
        }


        // ========================================================
        // ODOMETRY / FIELD RESET (Z KEY ON VIRTUAL KEYBOARD = BUTTON 1)
        // ========================================================
        if (m_joystick.getRawButtonPressed(
                BUTTON_Z)) {

            m_forwardHeading =
                    m_robotPose.getRotation();

            System.out.println(
                    "[ODOMETRY] Field Reset (Forward Heading) set to: "
                    + String.format(
                            Locale.US,
                            "%.2f",
                            m_forwardHeading.getDegrees())
                    + " deg");
        }
    }


    // ============================================================
    // SIDES BUTTON
    // ============================================================

    private void processSidesButton() {

        boolean currentState =
                m_joystick.getRawButton(
                        BUTTON_SIDES);


        if (currentState
                && !m_lastSidesButtonState) {

            m_targetSides++;


            if (m_targetSides
                    > MAX_TARGET_SIDES) {

                m_targetSides =
                        MIN_TARGET_SIDES;
            }


            m_lastWrittenSides =
                    m_targetSides;


            m_targetSidesEntry.setDouble(
                    m_targetSides);


            clearTangentLines();

            clearPolygonVertices();

            m_scannedUniqueAngles.clear();

            m_detectedUniqueTagIDs.clear();

            m_totalAccumulatedAngle = 0.0;


            m_polygonSidesEntry.setInteger(0);

            m_totalTagsRadarEntry.setInteger(0);

            m_referenceTagEntry.setInteger(-1);


            m_polygonVisualPublisher.set(
                    new Pose2d[0]);


            m_polygonEdgesPublisher.set(
                    new Pose2d[0]);


            System.out.println(
                    "[POLYGON] Sides = "
                    + m_targetSides);
        }


        m_lastSidesButtonState =
                currentState;
    }


    // ============================================================
    // C BUTTON
    // ============================================================

    private void processButtonC() {

        boolean currentActivationState =
                m_joystick.getRawButton(
                        BUTTON_C);


        if (currentActivationState
                && !m_lastActivationButtonState) {

            if (!m_isScanning) {

                m_isScanning = true;

                m_scannedUniqueAngles.clear();

                m_detectedUniqueTagIDs.clear();

                m_totalAccumulatedAngle = 0.0;

                m_polygonSidesEntry.setInteger(0);

                m_totalTagsRadarEntry.setInteger(0);

                m_referenceTagEntry.setInteger(-1);


                clearTangentLines();

                clearPolygonVertices();


                m_polygonVisualPublisher.set(
                        new Pose2d[0]);


                m_polygonEdgesPublisher.set(
                        new Pose2d[0]);


                double dx =
                        m_robotPose.getX()
                        - SCANNER_CENTER_X;

                double dy =
                        m_robotPose.getY()
                        - SCANNER_CENTER_Y;


                m_lastRobotAngleToCenter =
                        Math.atan2(
                                dy,
                                dx);


                System.out.println(
                        "[RADAR] Scanning started.");

            } else {

                m_isScanning = false;

                System.out.println(
                        "[RADAR] Scanning canceled.");
            }
        }


        m_lastActivationButtonState =
                currentActivationState;
    }


    // ============================================================
    // VISION
    // ============================================================

    private void processVision(
            PhotonPipelineResult result) {

        if (m_fieldLayout == null) {
            return;
        }


        PhotonTrackedTarget closestVisibleTag =
                null;

        double shortestDistance =
                Double.MAX_VALUE;


        PhotonTrackedTarget lockedTargetVisible =
                null;


        if (result.hasTargets()) {

            for (PhotonTrackedTarget target :
                    result.getTargets()) {

                int id =
                        target.getFiducialId();


                if (id < 0) {
                    continue;
                }


                if (m_fieldLayout
                        .getTagPose(id)
                        .isEmpty()) {

                    continue;
                }


                Transform3d cameraToTarget =
                        target.getBestCameraToTarget();


                if (cameraToTarget == null) {
                    continue;
                }


                double distance =
                        cameraToTarget
                                .getTranslation()
                                .getNorm();


                if (!Double.isFinite(distance)
                        || distance <= 0.02
                        || distance > 20.0) {

                    continue;
                }


                if (distance < shortestDistance) {

                    shortestDistance =
                            distance;

                    closestVisibleTag =
                            target;
                }


                if (id == m_lockedTargetID) {

                    lockedTargetVisible =
                            target;
                }


                if (m_isScanning) {

                    processTagForRadar(
                            target);
                }
            }
        }


        updateRobustTarget(
                closestVisibleTag,
                lockedTargetVisible);


        if (m_lockedTargetID >= 0) {

            m_closestTagEntry.setInteger(
                    m_lockedTargetID);

            m_targetTagEntry.setInteger(
                    m_lockedTargetID);

            m_lockedTargetEntry.setBoolean(
                    true);

            m_lostFramesEntry.setInteger(
                    m_lockedTargetLostFrames);


            if (Double.isFinite(
                    m_lockedTargetDistance)) {

                m_closestTagDistanceEntry.setDouble(
                        m_lockedTargetDistance);
            }


            if (Double.isFinite(
                    m_lockedTargetYaw)) {

                m_closestTagYawEntry.setDouble(
                        m_lockedTargetYaw);
            }


            if (Double.isFinite(
                    m_lockedTargetPitch)) {

                m_closestTagPitchEntry.setDouble(
                        m_lockedTargetPitch);
            }


            updateTagOrientationOnField(
                    m_lockedTargetID);


            m_referenceTagEntry.setInteger(
                    m_lockedTargetID);

        } else {

            m_closestTagEntry.setInteger(-1);

            m_targetTagEntry.setInteger(-1);

            m_lockedTargetEntry.setBoolean(
                    false);

            m_lostFramesEntry.setInteger(0);

            m_closestTagDistanceEntry.setDouble(-1.0);

            m_closestTagYawEntry.setDouble(0.0);

            m_closestTagPitchEntry.setDouble(0.0);

            m_fieldTagRollEntry.setDouble(0.0);

            m_fieldTagPitchEntry.setDouble(0.0);

            m_fieldTagYawEntry.setDouble(0.0);


            if (!m_isScanning) {

                m_referenceTagEntry.setInteger(-1);
            }


            clearTagVisualization();
        }
    }


    // ============================================================
    // TAG ORIENTATION
    // ============================================================

    private void updateTagOrientationOnField(
            int tagID) {

        if (m_fieldLayout == null) {
            return;
        }


        var optPose =
                m_fieldLayout.getTagPose(
                        tagID);


        if (optPose.isEmpty()) {
            return;
        }


        Pose3d tagPose3d =
                optPose.get();


        Rotation3d tagRotation =
                tagPose3d.getRotation();


        double fieldRoll =
                Math.toDegrees(
                        tagRotation.getX());


        double fieldPitch =
                Math.toDegrees(
                        tagRotation.getY());


        double fieldYaw =
                Math.toDegrees(
                        tagRotation.getZ());


        m_lastTagRollField =
                fieldRoll;

        m_lastTagPitchField =
                fieldPitch;

        m_lastTagYawField =
                fieldYaw;


        m_fieldTagRollEntry.setDouble(
                fieldRoll);

        m_fieldTagPitchEntry.setDouble(
                fieldPitch);

        m_fieldTagYawEntry.setDouble(
                fieldYaw);


        publishTagLines(
                tagID,
                tagPose3d);
    }


    // ============================================================
    // TAG LINES
    // ============================================================

    private void publishTagLines(
            int tagID,
            Pose3d tagPose3d) {

        if (tagPose3d == null) {
            return;
        }


        double x =
                tagPose3d.getX();

        double y =
                tagPose3d.getY();


        double theta =
                normalizeAngle(
                        tagPose3d
                                .getRotation()
                                .getZ());


        double lineLength =
                getVisualizationLength();


        double cosTheta =
                Math.cos(theta);

        double sinTheta =
                Math.sin(theta);


        double startX =
                x
                - lineLength
                * cosTheta;


        double startY =
                y
                - lineLength
                * sinTheta;


        double endX =
                x
                + lineLength
                * cosTheta;


        double endY =
                y
                + lineLength
                * sinTheta;


        Pose2d orientationStart =
                new Pose2d(
                        startX,
                        startY,
                        new Rotation2d(theta));


        Pose2d orientationEnd =
                new Pose2d(
                        endX,
                        endY,
                        new Rotation2d(theta));


        m_tagOrientationLinePublisher.set(
                new Pose2d[] {
                        orientationStart,
                        orientationEnd
                });


        Pose2d arrowStart =
                new Pose2d(
                        x,
                        y,
                        new Rotation2d(theta));


        m_tagOrientationArrowPublisher.set(
                arrowStart);


        double tangentTheta =
                normalizeAngle(
                        theta
                        + Math.PI / 2.0);


        double tangentCos =
                Math.cos(
                        tangentTheta);


        double tangentSin =
                Math.sin(
                        tangentTheta);


        double tangentStartX =
                x
                - lineLength
                * tangentCos;


        double tangentStartY =
                y
                - lineLength
                * tangentSin;


        double tangentEndX =
                x
                + lineLength
                * tangentCos;


        double tangentEndY =
                y
                + lineLength
                * tangentSin;


        Pose2d tangentStart =
                new Pose2d(
                        tangentStartX,
                        tangentStartY,
                        new Rotation2d(
                                tangentTheta));


        Pose2d tangentEnd =
                new Pose2d(
                        tangentEndX,
                        tangentEndY,
                        new Rotation2d(
                                tangentTheta));


        // IMPORTANT:
        // This visualization is kept.
        m_tagTangentLinePublisher.set(
                new Pose2d[] {
                        tangentStart,
                        tangentEnd
                });
    }


    // ============================================================
    // LINES LENGTH
    // ============================================================

    private double getVisualizationLength() {

        if (m_fieldLayout == null) {

            return 10.0;
        }


        double length =
                m_fieldLayout.getFieldLength();

        double width =
                m_fieldLayout.getFieldWidth();


        double diagonal =
                Math.sqrt(
                        length * length
                        + width * width);


        return diagonal
                + FIELD_LINE_MARGIN_METERS;
    }


    // ============================================================
    // CLEAR VISUALIZATION
    // ============================================================

    private void clearTagVisualization() {

        if (m_tagOrientationArrowPublisher != null) {

            m_tagOrientationArrowPublisher.set(
                    new Pose2d(
                            -100.0,
                            -100.0,
                            new Rotation2d()));
        }


        if (m_tagOrientationLinePublisher != null) {

            m_tagOrientationLinePublisher.set(
                    new Pose2d[0]);
        }


        if (m_tagTangentLinePublisher != null) {

            m_tagTangentLinePublisher.set(
                    new Pose2d[0]);
        }


        NetworkTableInstance nt =
                NetworkTableInstance.getDefault();


        nt.getEntry(
                "SmartDashboard/TagOrientation/ID")
                .setInteger(-1);
    }


    // ============================================================
    // CLEAR LINES
    // ============================================================

    private void clearTangentLines() {

        m_tangentLines.clear();

        m_tangentLinesCount = 0;


        if (m_tangentLinesCountEntry != null) {

            m_tangentLinesCountEntry
                    .setInteger(0);
        }
    }


    // ============================================================
    // CLEAR VERTICES
    // ============================================================

    private void clearPolygonVertices() {

        m_polygonVertices.clear();


        if (m_polygonEdgesPublisher != null) {

            m_polygonEdgesPublisher.set(
                    new Pose2d[0]);
        }
    }


    // ============================================================
    // ROBUST SELECTOR
    // ============================================================

    private void updateRobustTarget(
            PhotonTrackedTarget closestVisibleTag,
            PhotonTrackedTarget lockedTargetVisible) {

        double now =
                Timer.getFPGATimestamp();


        if (m_lockedTargetID < 0) {

            if (closestVisibleTag != null) {

                lockTarget(
                        closestVisibleTag,
                        now);
            }

            return;
        }


        if (lockedTargetVisible != null) {

            double currentDistance =
                    getDistance(
                            lockedTargetVisible);


            if (Double.isFinite(
                    currentDistance)) {

                m_lockedTargetDistance =
                        currentDistance;

                m_lockedTargetYaw =
                        lockedTargetVisible
                                .getYaw();

                m_lockedTargetPitch =
                        lockedTargetVisible
                                .getPitch();

                m_lockedTargetLastSeenTimestamp =
                        now;

                m_lockedTargetLostFrames = 0;
            }


            if (closestVisibleTag != null
                    && closestVisibleTag
                            .getFiducialId()
                            != m_lockedTargetID) {

                double newDistance =
                        getDistance(
                                closestVisibleTag);


                if (shouldChangeTarget(
                        newDistance,
                        m_lockedTargetDistance)) {

                    lockTarget(
                            closestVisibleTag,
                            now);
                }
            }

            return;
        }


        m_lockedTargetLostFrames++;


        double timeSinceLastSeen =
                m_lockedTargetLastSeenTimestamp > 0.0
                        ? now
                        - m_lockedTargetLastSeenTimestamp
                        : Double.MAX_VALUE;


        if (timeSinceLastSeen
                <= TARGET_MEMORY_SECONDS) {

            return;
        }


        if (closestVisibleTag != null) {

            lockTarget(
                    closestVisibleTag,
                    now);

        } else {

            clearTarget();
        }
    }


    // ============================================================
    // CHANGE TARGET
    // ============================================================

    private boolean shouldChangeTarget(
            double newDistance,
            double currentDistance) {

        if (!Double.isFinite(newDistance)
                || !Double.isFinite(currentDistance)) {

            return false;
        }


        boolean absoluteImprovement =
                newDistance
                + TARGET_SWITCH_MARGIN_METERS
                < currentDistance;


        boolean relativeImprovement =
                newDistance
                < currentDistance
                * TARGET_SWITCH_RATIO;


        return absoluteImprovement
                || relativeImprovement;
    }


    // ============================================================
    // LOCK TARGET
    // ============================================================

    private void lockTarget(
            PhotonTrackedTarget target,
            double timestamp) {

        if (target == null) {
            return;
        }


        int id =
                target.getFiducialId();


        double distance =
                getDistance(target);


        if (id < 0
                || !Double.isFinite(distance)) {

            return;
        }


        if (m_fieldLayout == null
                || m_fieldLayout
                        .getTagPose(id)
                        .isEmpty()) {

            return;
        }


        m_lockedTargetID =
                id;

        m_lockedTargetDistance =
                distance;

        m_lockedTargetYaw =
                target.getYaw();

        m_lockedTargetPitch =
                target.getPitch();

        m_lockedTargetLastSeenTimestamp =
                timestamp;

        m_lockedTargetLostFrames =
                0;


        m_lastClosestTagID =
                id;

        m_lastClosestTagDistance =
                distance;

        m_lastClosestTagYaw =
                target.getYaw();

        m_lastClosestTagPitch =
                target.getPitch();


        updateTagOrientationOnField(
                id);
    }


    // ============================================================
    // DISTANCE
    // ============================================================

    private double getDistance(
            PhotonTrackedTarget target) {

        if (target == null) {
            return Double.NaN;
        }


        Transform3d cameraToTarget =
                target.getBestCameraToTarget();


        if (cameraToTarget == null) {
            return Double.NaN;
        }


        double distance =
                cameraToTarget
                        .getTranslation()
                        .getNorm();


        return Double.isFinite(distance)
                ? distance
                : Double.NaN;
    }


    // ============================================================
    // CLEAR TARGET
    // ============================================================

    private void clearTarget() {

        m_lockedTargetID = -1;

        m_lockedTargetDistance =
                Double.NaN;

        m_lockedTargetYaw =
                Double.NaN;

        m_lockedTargetPitch =
                Double.NaN;

        m_lockedTargetLastSeenTimestamp =
                -1.0;

        m_lockedTargetLostFrames = 0;


        m_lastTagRollField =
                Double.NaN;

        m_lastTagPitchField =
                Double.NaN;

        m_lastTagYawField =
                Double.NaN;


        clearTagVisualization();
    }


    // ============================================================
    // RADAR
    // ============================================================

    private void processTagForRadar(
            PhotonTrackedTarget target) {

        int masterID =
                target.getFiducialId();


        if (m_fieldLayout == null) {
            return;
        }


        var optPose =
                m_fieldLayout.getTagPose(
                        masterID);


        if (optPose.isEmpty()) {
            return;
        }


        Pose2d tagPose =
                optPose.get().toPose2d();


        double distanceToCenter =
                tagPose
                        .getTranslation()
                        .getDistance(
                                new Translation2d(
                                        SCANNER_CENTER_X,
                                        SCANNER_CENTER_Y));


        if (distanceToCenter >= 1.80) {
            return;
        }


        if (!m_detectedUniqueTagIDs
                .contains(masterID)) {

            m_detectedUniqueTagIDs.add(
                    masterID);


            m_totalTagsRadarEntry.setInteger(
                    m_detectedUniqueTagIDs.size());
        }


        double realTagOrientation =
                normalizeAngle(
                        tagPose
                                .getRotation()
                                .getRadians());


        boolean isNewSlope = true;


        for (double registeredAngle :
                m_scannedUniqueAngles) {

            double delta =
                    Math.abs(
                            realTagOrientation
                            - registeredAngle);


            delta =
                    normalizeAngle(
                            delta);


            delta =
                    Math.abs(delta);


            if (delta
                    < ANGULAR_TOLERANCE_RAD) {

                isNewSlope =
                        false;

                break;
            }
        }


        if (isNewSlope) {

            m_scannedUniqueAngles.add(
                    realTagOrientation);


            m_polygonSidesEntry.setInteger(
                    m_scannedUniqueAngles.size());


            publishPolygonalShapeAdvantageScope();
        }
    }


    // ============================================================
    // MOVE TO TAG
    // ============================================================

    @SuppressWarnings("removal")
    private void moveDirectlyToClosestTag() {

        int tagID =
                m_lockedTargetID;


        if (tagID < 0) {

            PhotonPipelineResult result =
                    m_camera.getLatestResult();


            if (result.hasTargets()) {

                PhotonTrackedTarget closestTag =
                        null;

                double shortestDistance =
                        Double.MAX_VALUE;


                for (PhotonTrackedTarget target :
                        result.getTargets()) {

                    int id =
                            target.getFiducialId();


                    if (id < 0) {
                        continue;
                    }


                    if (m_fieldLayout == null
                            || m_fieldLayout
                                    .getTagPose(id)
                                    .isEmpty()) {

                        continue;
                    }


                    double distance =
                            getDistance(
                                    target);


                    if (!Double.isFinite(
                            distance)) {

                        continue;
                    }


                    if (distance < shortestDistance) {

                        shortestDistance =
                                distance;

                        closestTag =
                                target;
                    }
                }


                if (closestTag != null) {

                    lockTarget(
                            closestTag,
                            Timer.getFPGATimestamp());


                    tagID =
                            m_lockedTargetID;
                }
            }
        }


        if (tagID < 0) {
            return;
        }


        if (m_fieldLayout == null) {
            return;
        }


        var optPose =
                m_fieldLayout.getTagPose(
                        tagID);


        if (optPose.isEmpty()) {
            return;
        }


        Pose3d tagPose3d =
                optPose.get();


        Pose2d tagPose =
                tagPose3d.toPose2d();


        double fieldTagAngle =
                normalizeAngle(
                        tagPose3d
                                .getRotation()
                                .getZ());


        double angleInFrontOfTag =
                normalizeAngle(
                        fieldTagAngle
                        + Math.PI);


        double newX =
                tagPose.getX()
                + DISTANCE_CLOSE_TO_TAG
                * Math.cos(
                        angleInFrontOfTag);


        double newY =
                tagPose.getY()
                + DISTANCE_CLOSE_TO_TAG
                * Math.sin(
                        angleInFrontOfTag);


        Rotation2d newRotation =
                new Rotation2d(
                        angleInFrontOfTag);


        m_robotPose =
                new Pose2d(
                        newX,
                        newY,
                        newRotation);


        // ========================================================
        // CAPTURE SIDE
        // ========================================================

        if (m_tangentLinesCount
                < m_targetSides) {

            captureTangentLine(
                    tagID,
                    tagPose3d);
        }


        m_referenceTagEntry.setInteger(
                tagID);

        m_targetTagEntry.setInteger(
                tagID);

        m_alignmentModeEntry.setBoolean(
                true);


        updateTagOrientationOnField(
                tagID);


        printPositionInfoInFrontOfTag(
                tagID,
                tagPose3d,
                fieldTagAngle,
                angleInFrontOfTag);
    }


    // ============================================================
    // CAPTURE LINE
    // ============================================================

    private void captureTangentLine(
            int tagID,
            Pose3d tagPose3d) {

        if (tagPose3d == null) {
            return;
        }


        if (m_tangentLinesCount
                >= m_targetSides) {

            return;
        }


        double x =
                tagPose3d.getX();

        double y =
                tagPose3d.getY();


        double tagAngle =
                normalizeAngle(
                        tagPose3d
                                .getRotation()
                                .getZ());


        double tangentAngle =
                normalizeAngle(
                        tagAngle
                        + Math.PI / 2.0);


        TangentLine line =
                new TangentLine(
                        tagID,
                        x,
                        y,
                        tangentAngle);


        m_tangentLines.add(
                line);


        m_tangentLinesCount =
                m_tangentLines.size();


        if (m_tangentLinesCountEntry != null) {

            m_tangentLinesCountEntry
                    .setInteger(
                            m_tangentLinesCount);
        }


        // ========================================================
        // SIDE X/N
        // ========================================================

        System.out.println("");

        System.out.println(
                "------------------------------------------");

        System.out.println(
                "SIDE "
                + m_tangentLinesCount
                + "/"
                + m_targetSides
                + " CAPTURED");

        System.out.println(
                "TAG ID = "
                + tagID);

        System.out.println(
                "Point = ("
                + String.format(
                        "%.6f",
                        x)
                + ", "
                + String.format(
                        "%.6f",
                        y)
                + ")");

        System.out.println(
                "Angle = "
                + String.format(
                        "%.4f",
                        Math.toDegrees(
                                tangentAngle))
                + " deg");


        printLineEquation(
                line,
                "SIDE "
                + m_tangentLinesCount
                + " EQUATION");


        System.out.println(
                "------------------------------------------");


        // ========================================================
        // IF WE ALREADY HAVE ALL LINES
        // ========================================================

        if (m_tangentLinesCount
                >= m_targetSides) {

            printDynamicTangentLines();

            calculateCompletePolygonMapping();
        }
    }


    // ============================================================
    // PRINT EQUATION
    // ============================================================

    private void printLineEquation(
            TangentLine line,
            String prefix) {

        if (line.isVertical) {

            System.out.println(
                    prefix
                    + ": x = "
                    + String.format(
                            "%.6f",
                            line.intercept));

        } else {

            System.out.println(
                    prefix
                    + ": y = "
                    + String.format(
                            "%.6f",
                            line.slope)
                    + "x "
                    + (line.intercept >= 0
                            ? "+ "
                            : "- ")
                    + String.format(
                            "%.6f",
                            Math.abs(
                                    line.intercept)));
        }
    }


    // ============================================================
    // PRINT LINES
    // ============================================================

    private void printDynamicTangentLines() {

        if (m_tangentLines.isEmpty()) {

            System.out.println(
                    "[TANGENT] No lines present.");

            return;
        }


        System.out.println("");

        System.out.println(
                "==========================================");

        System.out.println(
                "CAPTURED TANGENT LINES");

        System.out.println(
                "SIDES = "
                + m_targetSides);

        System.out.println(
                "LINES = "
                + m_tangentLines.size());

        System.out.println(
                "==========================================");


        for (int i = 0;
                i < m_tangentLines.size();
                i++) {

            TangentLine line =
                    m_tangentLines.get(i);


            System.out.println("");

            System.out.println(
                    "LINE "
                    + (i + 1)
                    + "/"
                    + m_targetSides);


            System.out.println(
                    "TAG ID = "
                    + line.tagID);


            System.out.println(
                    "Point = ("
                    + String.format(
                            "%.6f",
                            line.x)
                    + ", "
                    + String.format(
                            "%.6f",
                            line.y)
                    + ")");


            System.out.println(
                    "Angle = "
                    + String.format(
                            "%.4f",
                            Math.toDegrees(
                                    line.angle))
                    + " deg");


            printLineEquation(
                    line,
                    "EQUATION");
        }


        System.out.println("");

        System.out.println(
                "==========================================");
    }


    // ============================================================
    // COMPLETE POLYGON MAPPING AND EXPORT
    // ============================================================

    private void calculateCompletePolygonMapping() {

        if (m_tangentLines.size() < 3) {

            System.out.println("");

            System.out.println(
                    "[POLYGON] At least 3 sides are needed "
                    + "to calculate vertices.");

            return;
        }


        if (m_tangentLines.size()
                != m_targetSides) {

            System.out.println(
                    "[POLYGON] Not all sides are present.");

            return;
        }


        // ========================================================
        // COPY FOR SORTING
        // ========================================================

        List<TangentLine> sortedLines =
                new ArrayList<>(
                        m_tangentLines);


        // ========================================================
        // 0-360 ANGULAR ORDER
        // ========================================================

        sortedLines.sort(
                Comparator.comparingDouble(
                        this::getAngle360));


        System.out.println("");

        System.out.println(
                "==========================================");

        System.out.println(
                "       COMPLETE POLYGON");

        System.out.println(
                "       "
                + m_tangentLines.size()
                + "/"
                + m_targetSides
                + " SIDES CAPTURED");

        System.out.println(
                "==========================================");


        // ========================================================
        // SEQUENTIAL MAPPING
        // ========================================================

        System.out.println("");

        System.out.println(
                "SEQUENTIAL SIDES MAPPING");

        System.out.println(
                "------------------------------------------");


        for (int i = 0;
                i < sortedLines.size();
                i++) {

            TangentLine line =
                    sortedLines.get(i);


            double originalAngle =
                    Math.toDegrees(
                            line.angle);


            double finalAngle =
                    getAngle360(
                            line);


            System.out.println(
                    "ORDER "
                    + (i + 1)
                    + " -> ORIGINAL SIDE "
                    + getOriginalIndex(
                            line)
                    + " | TAG "
                    + line.tagID
                    + " | Original Angle = "
                    + String.format(
                            "%.4f",
                            originalAngle)
                    + " deg"
                    + " | Final Angle = "
                    + String.format(
                            "%.4f",
                            finalAngle)
                    + " deg");
        }


        System.out.println(
                "------------------------------------------");


        // ========================================================
        // SEQUENTIAL ORDER EQUATIONS
        // ========================================================

        System.out.println("");

        System.out.println(
                "EQUATIONS IN SEQUENTIAL ORDER");


        for (int i = 0;
                i < sortedLines.size();
                i++) {

            TangentLine line =
                    sortedLines.get(i);


            System.out.println("");

            System.out.println(
                    "SIDE "
                    + (i + 1)
                    + "/"
                    + m_targetSides
                    + " | TAG = "
                    + line.tagID);


            printLineEquation(
                    line,
                    "SIDE "
                    + (i + 1)
                    + " EQUATION");
        }


        // ========================================================
        // CALCULATE INTERSECTIONS
        // ========================================================

        m_polygonVertices.clear();


        System.out.println("");

        System.out.println(
                "==========================================");

        System.out.println(
                "POLYGON VERTICES");

        System.out.println(
                "CONSECUTIVE LINES INTERSECTION");

        System.out.println(
                "==========================================");


        int count =
                sortedLines.size();


        for (int i = 0;
                i < count;
                i++) {

            TangentLine lineA =
                    sortedLines.get(i);


            TangentLine lineB =
                    sortedLines.get(
                            (i + 1) % count);


            PolygonPoint intersection =
                    calculateIntersection(
                            lineA,
                            lineB);


            System.out.println("");

            System.out.println(
                    "EDGE "
                    + (i + 1)
                    + "/"
                    + count);


            System.out.println(
                    "Between SIDE "
                    + (i + 1)
                    + " and SIDE "
                    + ((i + 1) % count + 1));


            if (intersection == null) {

                System.out.println(
                        "ERROR: The lines are "
                        + "parallel or coincident.");

                continue;
            }


            m_polygonVertices.add(
                    intersection);


            System.out.println(
                    "EDGE "
                    + (i + 1)
                    + " COORDINATES: ("
                    + String.format(
                            "%.6f",
                            intersection.x)
                    + ", "
                    + String.format(
                            "%.6f",
                            intersection.y)
                    + ")");
        }


        // ========================================================
        // PUBLISH VERTICES
        // ========================================================

        publishPolygonVertices();


        // ========================================================
        // FINAL SUMMARY
        // ========================================================

        System.out.println("");

        System.out.println(
                "==========================================");

        System.out.println(
                "FINAL POLYGON MAPPING");

        System.out.println(
                "==========================================");


        for (int i = 0;
                i < m_polygonVertices.size();
                i++) {

            PolygonPoint point =
                    m_polygonVertices.get(i);


            System.out.println(
                    "VERTEX "
                    + (i + 1)
                    + " = ("
                    + String.format(
                            "%.6f",
                            point.x)
                    + ", "
                    + String.format(
                            "%.6f",
                            point.y)
                    + ")");
        }


        System.out.println(
                "==========================================");

        System.out.println(
                "CALCULATED VERTICES = "
                + m_polygonVertices.size()
                + "/"
                + m_targetSides);

        System.out.println(
                "POLYGON = "
                + m_targetSides
                + "/"
                + m_targetSides);

        System.out.println(
                "==========================================");


        // ========================================================
        // EXPORT TO C:\Downloads\map_obstacles.txt (Append Mode)
        // ========================================================
        exportObstacleToNotepad(sortedLines);
    }

    private void exportObstacleToNotepad(List<TangentLine> sortedLines) {
        String filePath = "C:\\Downloads\\map_obstacles.txt";

        try {
            File file = new File(filePath);
            File parentDir = file.getParentFile();
            if (parentDir != null && !parentDir.exists()) {
                parentDir.mkdirs();
            }

            try (FileWriter writer = new FileWriter(file, true)) {
                writer.write("/* OBSTACLE */\n");
                writer.write("[\n");

                int numVertices = m_polygonVertices.size();
                for (int i = 0; i < numVertices; i++) {
                    int prevVertexIndex = (i - 1 + numVertices) % numVertices;
                    int currentVertexIndex = i;

                    PolygonPoint p1 = m_polygonVertices.get(prevVertexIndex);
                    PolygonPoint p2 = m_polygonVertices.get(currentVertexIndex);
                    TangentLine line = sortedLines.get(i);

                    double minX = Math.min(p1.x, p2.x);
                    double maxX = Math.max(p1.x, p2.x);
                    double minY = Math.min(p1.y, p2.y);
                    double maxY = Math.max(p1.y, p2.y);

                    double slopeVal = line.isVertical ? 0.0 : line.slope;
                    double interceptVal = line.intercept;

                    String fieldTag = m_loadedFieldName != null ? m_loadedFieldName : "Unknown Field";

                    writer.write(String.format(Locale.US, "  [%.6f, %.6f, [%.6f, %.6f], [%.6f, %.6f], \"%s\"]",
                            slopeVal, interceptVal, minX, maxX, minY, maxY, fieldTag));

                    if (i < numVertices - 1) {
                        writer.write(",");
                    }
                    writer.write("\n");
                }

                writer.write("]\n");
                writer.write("/* END_OBSTACLE */\n\n");
                System.out.println("[EXPORTER] Obstacle appended to " + filePath);
            }
        } catch (IOException e) {
            System.out.println("[EXPORTER] Error writing to disk: " + e.getMessage());
        }
    }


    // ============================================================
    // 0-360 ANGLE
    // ============================================================

    private double getAngle360(
            TangentLine line) {

        double degrees =
                Math.toDegrees(
                        line.angle);


        while (degrees < 0.0) {

            degrees += 360.0;
        }


        while (degrees >= 360.0) {

            degrees -= 360.0;
        }


        return degrees;
    }


    // ============================================================
    // ORIGINAL INDEX
    // ============================================================

    private int getOriginalIndex(
            TangentLine line) {

        for (int i = 0;
                i < m_tangentLines.size();
                i++) {

            if (m_tangentLines.get(i)
                    == line) {

                return i + 1;
            }
        }


        return -1;
    }


    // ============================================================
    // TWO LINES INTERSECTION (TO DRAW POLYGON)
    // ============================================================

    private PolygonPoint calculateIntersection(
            TangentLine lineA,
            TangentLine lineB) {

        if (lineA.isVertical
                && lineB.isVertical) {

            return null;
        }


        if (lineA.isVertical) {

            double x =
                    lineA.intercept;


            double y =
                    lineB.slope
                    * x
                    + lineB.intercept;


            if (!Double.isFinite(y)) {

                return null;
            }


            return new PolygonPoint(
                    x,
                    y);
        }


        if (lineB.isVertical) {

            double x =
                    lineB.intercept;


            double y =
                    lineA.slope
                    * x
                    + lineA.intercept;


            if (!Double.isFinite(y)) {

                return null;
            }


            return new PolygonPoint(
                    x,
                    y);
        }


        double slopesDifference =
                lineA.slope
                - lineB.slope;


        if (Math.abs(
                slopesDifference)
                < 1e-9) {

            return null;
        }


        double x =
                (lineB.intercept
                - lineA.intercept)
                / slopesDifference;


        double y =
                lineA.slope
                * x
                + lineA.intercept;


        if (!Double.isFinite(x)
                || !Double.isFinite(y)) {

            return null;
        }


        return new PolygonPoint(
                x,
                y);
    }


    // ============================================================
    // PUBLISH VERTICES
    // ============================================================

    private void publishPolygonVertices() {

        if (m_polygonEdgesPublisher == null) {
            return;
        }


        if (m_polygonVertices.isEmpty()) {

            m_polygonEdgesPublisher.set(
                    new Pose2d[0]);

            return;
        }


        Pose2d[] vertices =
                new Pose2d[
                        m_polygonVertices.size()
                        + 1];


        for (int i = 0;
                i < m_polygonVertices.size();
                i++) {

            PolygonPoint point =
                    m_polygonVertices.get(i);


            vertices[i] =
                    new Pose2d(
                            point.x,
                            point.y,
                            new Rotation2d());
        }


        vertices[
                m_polygonVertices.size()] =
                vertices[0];


        m_polygonEdgesPublisher.set(
                vertices);
    }


    // ============================================================
    // TAG INFORMATION
    // ============================================================

    private void printPositionInfoInFrontOfTag(
            int tagID,
            Pose3d tagPose3d,
            double fieldTagAngle,
            double angleInFrontOfTag) {

        double x =
                tagPose3d.getX();

        double y =
                tagPose3d.getY();


        double tangentAngle =
                normalizeAngle(
                        fieldTagAngle
                        + Math.PI / 2.0);


        System.out.println(
                "===============================");

        System.out.println(
                "[APRILTAG] TAG = "
                + tagID);

        System.out.println(
                "[APRILTAG] Point = ("
                + String.format(
                        "%.3f",
                        x)
                + ", "
                + String.format(
                        "%.3f",
                        y)
                + ")");

        System.out.println(
                "[APRILTAG] Orientation = "
                + String.format(
                        "%.2f",
                        Math.toDegrees(
                                fieldTagAngle))
                + " deg");

        System.out.println(
                "[APRILTAG] ROBOT Front = "
                + String.format(
                        "%.2f",
                        Math.toDegrees(
                                angleInFrontOfTag))
                + " deg");


        if (Math.abs(
                Math.cos(
                        fieldTagAngle))
                < VERTICAL_ANGLE_EPSILON) {

            System.out.println(
                    "[TAG ORIENTATION LINE] x = "
                    + String.format(
                            "%.3f",
                            x));

        } else {

            double m =
                    Math.tan(
                            fieldTagAngle);


            double b =
                    y - m * x;


            System.out.println(
                    "[TAG ORIENTATION LINE] y = "
                    + String.format(
                            "%.6f",
                            m)
                    + "x + "
                    + String.format(
                            "%.6f",
                            b));
        }


        if (Math.abs(
                Math.cos(
                        tangentAngle))
                < VERTICAL_ANGLE_EPSILON) {

            System.out.println(
                    "[TAG TANGENT LINE] x = "
                    + String.format(
                            "%.3f",
                            x));

        } else {

            double m =
                    Math.tan(
                            tangentAngle);


            double b =
                    y - m * x;


            System.out.println(
                    "[TAG TANGENT LINE] y = "
                    + String.format(
                            "%.6f",
                            m)
                    + "x "
                    + (b >= 0
                            ? "+ "
                            : "- ")
                    + String.format(
                            "%.6f",
                            Math.abs(b)));
        }


        System.out.println(
                "[APRILTAG] Robot = ("
                + String.format(
                        "%.3f",
                        m_robotPose.getX())
                + ", "
                + String.format(
                        "%.3f",
                        m_robotPose.getY())
                + ")");


        System.out.println(
                "===============================");
    }


    // ============================================================
    // LOAD OBSTACLES FROM FILE
    // ============================================================

    private void loadObstaclesFromFile(String selectedField) {

        m_obstacles.clear();

        if (selectedField == null || selectedField.isBlank()) {
            System.out.println("[COLLISION] Invalid field to load obstacles.");
            return;
        }

        File file = new File(OBSTACLES_FILE_PATH);

        if (!file.exists() || !file.isFile()) {
            System.out.println("[COLLISION] Obstacles file not found: " + OBSTACLES_FILE_PATH);
            return;
        }

        StringBuilder content = new StringBuilder();

        try (java.io.BufferedReader reader =
                     new java.io.BufferedReader(
                             new java.io.FileReader(file))) {

            String line;

            while ((line = reader.readLine()) != null) {
                content.append(line).append('\n');
            }

        } catch (IOException e) {
            System.out.println("[COLLISION] Error reading obstacles: " + e.getMessage());
            return;
        }

        String number =
                "([-+]?(?:\\d+(?:\\.\\d*)?|\\.\\d+)(?:[eE][-+]?\\d+)?)";

        Pattern blockPattern = Pattern.compile(
                "/\\*\\s*OBSTACLE\\s*\\*/(.*?)/\\*\\s*END_OBSTACLE\\s*\\*/",
                Pattern.DOTALL | Pattern.CASE_INSENSITIVE);

        Pattern segmentPattern = Pattern.compile(
                "\\[\\s*"
                        + number
                        + "\\s*,\\s*"
                        + number
                        + "\\s*,\\s*\\[\\s*"
                        + number
                        + "\\s*,\\s*"
                        + number
                        + "\\s*\\]\\s*,\\s*\\[\\s*"
                        + number
                        + "\\s*,\\s*"
                        + number
                        + "\\s*\\]\\s*,\\s*\\\"([^\\\"]*)\\\"\\s*\\]",
                Pattern.DOTALL | Pattern.CASE_INSENSITIVE);

        Matcher blockMatcher =
                blockPattern.matcher(content.toString());

        int fieldBlocks = 0;

        while (blockMatcher.find()) {

            String blockBody = blockMatcher.group(1);

            Matcher segmentMatcher =
                    segmentPattern.matcher(blockBody);

            while (segmentMatcher.find()) {

                double m = Double.parseDouble(segmentMatcher.group(1));
                double b = Double.parseDouble(segmentMatcher.group(2));
                double minX = Double.parseDouble(segmentMatcher.group(3));
                double maxX = Double.parseDouble(segmentMatcher.group(4));
                double minY = Double.parseDouble(segmentMatcher.group(5));
                double maxY = Double.parseDouble(segmentMatcher.group(6));
                String obstacleField = segmentMatcher.group(7).trim();

                if (!selectedField.trim().equalsIgnoreCase(obstacleField)) {
                    continue;
                }

                double[][] segment =
                        buildObstacleSegment(
                                m,
                                b,
                                minX,
                                maxX,
                                minY,
                                maxY);

                if (segment != null) {
                    m_obstacles.add(segment);
                    fieldBlocks++;
                }
            }
        }

        System.out.println(
                "[COLLISION] Obstacles loaded for field '"
                        + selectedField
                        + "' = "
                        + fieldBlocks);
    }


    // ============================================================
    // BUILD OBSTACLE SEGMENT
    // ============================================================

    private double[][] buildObstacleSegment(
            double m,
            double b,
            double minX,
            double maxX,
            double minY,
            double maxY) {

        if (!Double.isFinite(minX)
                || !Double.isFinite(maxX)
                || !Double.isFinite(minY)
                || !Double.isFinite(maxY)) {
            return null;
        }

        double x1;
        double y1;
        double x2;
        double y2;

        double rangeX = Math.abs(maxX - minX);
        double rangeY = Math.abs(maxY - minY);

        // Vertical segments are exported with minX ~= maxX.
        if (rangeX <= COLLISION_EPSILON && rangeY > COLLISION_EPSILON) {

            double x = (minX + maxX) / 2.0;

            x1 = x;
            y1 = minY;
            x2 = x;
            y2 = maxY;

        } else {

            if (!Double.isFinite(m) || !Double.isFinite(b)) {
                return null;
            }

            x1 = minX;
            y1 = m * x1 + b;
            x2 = maxX;
            y2 = m * x2 + b;

            // Clamp computed endpoints when the exported Y bounds are the
            // actual limiting interval of the stored segment.
            if (rangeY > COLLISION_EPSILON) {
                y1 = Math.max(minY, Math.min(maxY, y1));
                y2 = Math.max(minY, Math.min(maxY, y2));
            } else {
                double y = (minY + maxY) / 2.0;
                y1 = y;
                y2 = y;
            }
        }

        if (!Double.isFinite(x1)
                || !Double.isFinite(y1)
                || !Double.isFinite(x2)
                || !Double.isFinite(y2)) {
            return null;
        }

        return new double[][] {
                {x1, y1},
                {x2, y2}
        };
    }


    // ============================================================
    // ROBOT HITBOX
    // ============================================================

    private Translation2d[] getCorners(Pose2d pose) {

        Translation2d[] relatives = new Translation2d[] {
                m_frontLeft,
                m_frontRight,
                m_backRight,
                m_backLeft
        };

        Translation2d[] absolutes =
                new Translation2d[relatives.length];

        for (int i = 0; i < relatives.length; i++) {
            absolutes[i] =
                    relatives[i]
                            .rotateBy(pose.getRotation())
                            .plus(pose.getTranslation());
        }

        return absolutes;
    }


    // ============================================================
    // CCW ORIENTATION
    // ============================================================

    private int orientationCCW(
            double[] a,
            double[] b,
            double[] c) {

        double value =
                (b[0] - a[0]) * (c[1] - a[1])
                - (b[1] - a[1]) * (c[0] - a[0]);

        if (Math.abs(value) <= COLLISION_EPSILON) {
            return 0;
        }

        return value > 0.0 ? 1 : -1;
    }


    // ============================================================
    // POINT ON SEGMENT
    // ============================================================

    private boolean onSegment(
            double[] a,
            double[] b,
            double[] p) {

        if (Math.abs(
                (b[0] - a[0]) * (p[1] - a[1])
                - (b[1] - a[1]) * (p[0] - a[0]))
                > COLLISION_EPSILON) {
            return false;
        }

        return p[0] >= Math.min(a[0], b[0]) - COLLISION_EPSILON
                && p[0] <= Math.max(a[0], b[0]) + COLLISION_EPSILON
                && p[1] >= Math.min(a[1], b[1]) - COLLISION_EPSILON
                && p[1] <= Math.max(a[1], b[1]) + COLLISION_EPSILON;
    }


    // ============================================================
    // SEGMENT INTERSECTION
    // ============================================================

    private boolean segmentsIntersect(
            double[] a,
            double[] b,
            double[] c,
            double[] d) {

        int o1 = orientationCCW(a, b, c);
        int o2 = orientationCCW(a, b, d);
        int o3 = orientationCCW(c, d, a);
        int o4 = orientationCCW(c, d, b);

        if (o1 != o2 && o3 != o4) {
            return true;
        }

        if (o1 == 0 && onSegment(a, b, c)) {
            return true;
        }

        if (o2 == 0 && onSegment(a, b, d)) {
            return true;
        }

        if (o3 == 0 && onSegment(c, d, a)) {
            return true;
        }

        return o4 == 0 && onSegment(c, d, b);
    }


    // ============================================================
    // COMPLETE ROBOT COLLISION CHECK
    // ============================================================

    private boolean collidesFullPose(Pose2d pose) {

        if (pose == null || m_obstacles.isEmpty()) {
            return false;
        }

        Translation2d[] corners = getCorners(pose);

        double[][] robotPoints = new double[4][2];

        for (int i = 0; i < 4; i++) {
            robotPoints[i][0] = corners[i].getX();
            robotPoints[i][1] = corners[i].getY();
        }

        // Outer perimeter.
        int[][] sides = new int[][] {
                {0, 1},
                {1, 2},
                {2, 3},
                {3, 0}
        };

        // Internal diagonals form an X and protect thin objects crossing the body.
        int[][] diagonals = new int[][] {
                {0, 2},
                {1, 3}
        };

        for (double[][] obstacle : m_obstacles) {

            if (obstacle == null
                    || obstacle.length < 2
                    || obstacle[0] == null
                    || obstacle[1] == null
                    || obstacle[0].length < 2
                    || obstacle[1].length < 2) {
                continue;
            }

            double[] p1 = obstacle[0];
            double[] p2 = obstacle[1];

            for (int[] side : sides) {
                if (segmentsIntersect(
                        robotPoints[side[0]],
                        robotPoints[side[1]],
                        p1,
                        p2)) {
                    return true;
                }
            }

            for (int[] diagonal : diagonals) {
                if (segmentsIntersect(
                        robotPoints[diagonal[0]],
                        robotPoints[diagonal[1]],
                        p1,
                        p2)) {
                    return true;
                }
            }
        }

        return false;
    }


    // ============================================================
    // NORMALIZE ANGLE
    // ============================================================

    private double normalizeAngle(
            double angle) {

        while (angle > Math.PI) {

            angle -=
                    2.0 * Math.PI;
        }


        while (angle < -Math.PI) {

            angle +=
                    2.0 * Math.PI;
        }


        return angle;
    }


    // ============================================================
    // SIMULATION PERIODIC
    // ============================================================

    @Override
    public void simulationPeriodic() {

        double forwardSpeed = 0.0;
        double strafeSpeed = 0.0;
        double turnSpeed = 0.0;

        if (m_isScanning) {
            double dx = m_robotPose.getX() - SCANNER_CENTER_X;
            double dy = m_robotPose.getY() - SCANNER_CENTER_Y;
            double currentAngleToCenter = Math.atan2(dy, dx);
            double currentDistance = Math.sqrt(dx * dx + dy * dy);
            double radiusError = AUTONOMOUS_ORBIT_RADIUS - currentDistance;
            double radialCorrection = radiusError * 0.18;

            forwardSpeed = (-ORBIT_SPEED * Math.sin(currentAngleToCenter) * 0.02) + (radialCorrection * Math.cos(currentAngleToCenter));
            strafeSpeed  = (ORBIT_SPEED * Math.cos(currentAngleToCenter) * 0.02) + (radialCorrection * Math.sin(currentAngleToCenter));

            double lookAngleTowardsCenter = normalizeAngle(currentAngleToCenter + Math.PI);
            double rotationError = normalizeAngle(lookAngleTowardsCenter - m_robotPose.getRotation().getRadians());
            turnSpeed = rotationError * 0.15;

            double angularStep = normalizeAngle(currentAngleToCenter - m_lastRobotAngleToCenter);
            m_totalAccumulatedAngle += Math.abs(angularStep);
            m_lastRobotAngleToCenter = currentAngleToCenter;

            if (m_totalAccumulatedAngle >= 2.0 * Math.PI) {
                m_isScanning = false;
            }

        } else if (m_rotatingWithV) {
            double currentRotation = m_robotPose.getRotation().getRadians();
            double rotationError = normalizeAngle(m_rotationTargetV - currentRotation);
            turnSpeed = rotationError * 0.12;

            double maxTurnSpeed = Math.toRadians(3.0);
            turnSpeed = Math.max(-maxTurnSpeed, Math.min(maxTurnSpeed, turnSpeed));

            if (Math.abs(rotationError) <= TURN_TOLERANCE_V) {
                m_robotPose = new Pose2d(m_robotPose.getX(), m_robotPose.getY(), new Rotation2d(m_rotationTargetV));
                m_rotatingWithV = false;
                turnSpeed = 0.0;
            }

        } else {
            forwardSpeed = -m_joystick.getRawAxis(1) * 0.12;
            strafeSpeed  = -m_joystick.getRawAxis(0) * 0.12;
            turnSpeed    = -m_joystick.getRawAxis(4) * 0.06;
        }

        forwardSpeed = Math.max(-0.12, Math.min(0.12, forwardSpeed));
        strafeSpeed  = Math.max(-0.12, Math.min(0.12, strafeSpeed));

        if (!m_rotatingWithV) {
            turnSpeed = Math.max(-0.06, Math.min(0.06, turnSpeed));
        }

        // ========================================================
        // FIELD RESET APPLICATION (FORWARD HEADING)
        // ========================================================
        Translation2d translation = new Translation2d(forwardSpeed, strafeSpeed)
                .rotateBy(m_robotPose.getRotation().minus(m_forwardHeading));

        Pose2d futurePose =
                m_robotPose.transformBy(
                        new Transform2d(
                                translation,
                                new Rotation2d(
                                        turnSpeed)));

        if (!collidesFullPose(futurePose)) {
            m_robotPose = futurePose;
            m_collisionBlockedLastCycle = false;
        } else {
            // Block the entire motion step when the future hitbox intersects an obstacle.
            if (!m_collisionBlockedLastCycle) {
                System.out.println("[COLLISION] Movement blocked by obstacle.");
            }
            m_collisionBlockedLastCycle = true;
        }

        if (m_visionSim != null) {
            m_visionSim.update(
                    m_robotPose);
        }
    }


    // ============================================================
    // ORIGINAL VISUAL POLYGON
    // ============================================================

    private void publishPolygonalShapeAdvantageScope() {
        int numSides = m_scannedUniqueAngles.size();
        if (numSides < 3) return;

        Pose2d[] visualVertices = new Pose2d[numSides + 1];
        double realStructureRadius = 1.05;

        for (int i = 0; i < numSides; i++) {
            double radiansStep = (2.0 * Math.PI / numSides) * i;
            visualVertices[i] = new Pose2d(
                    SCANNER_CENTER_X + realStructureRadius * Math.cos(radiansStep),
                    SCANNER_CENTER_Y + realStructureRadius * Math.sin(radiansStep),
                    new Rotation2d(radiansStep));
        }

        visualVertices[numSides] = visualVertices[0];
        m_polygonVisualPublisher.set(visualVertices);
    }


    // ============================================================
    // TELEOP
    // ============================================================

    @Override
    public void teleopInit() {}


    @Override
    public void teleopPeriodic() {}
}