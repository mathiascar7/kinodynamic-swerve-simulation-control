package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Random;
import java.util.TreeSet;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Simulador Físico Live Kinodynamic Swerve en Java (WPILib / AdvantageScope)
 * - Réplica exacta del controlador analítico O(1) validado en Python (DARE + Observador 2D de Agarre + Carga Dinámica)
 * - Planta rígida SE(2) a 5 kHz (10 subciclos de 2 ms por frame) con Pacejka Combined-Slip e inercia de rueda
 * - Resolvedor continuo SAT multi-vértice sin teletransportación ni atascos + replanificación de recuperación
 * - Publicación en vivo para AdvantageScope (camino resumido en forma de L, puntos de choque, hielo negro y módulos)
 * - Registro riguroso en TXT en "C:\Users\HP ENVY\Downloads\Java advantage scope data"
 */
public class Robot extends TimedRobot {

    // =========================================================================
    // 1. CONSTANTES DE TIEMPO REAL, CAMPO Y EXPORTACIÓN DE DATOS
    // =========================================================================
    private static final double DT_FRAME = 0.020;                          // 50 Hz bucle principal WPILib
    private static final int CONTROL_SUBCYCLES = 10;                       // 10 ciclos de control/frame = 500 Hz
    private static final double DT_CONTROL = DT_FRAME / CONTROL_SUBCYCLES; // 0.002 s exacto igual a Python
    private static final int PHYS_SUBSTEPS = 10;                           // 10 subpasos físicos/ciclo = 5 kHz
    private static final double G = 9.81;

    private static final double CAMPO_LARGO_M = 16.54;
    private static final double CAMPO_ANCHO_M = 8.07;
    private static final String DEFAULT_FIELD = "2026 - Rebuilt Welded";

    private static final String EXPORT_FOLDER_WIN = "02_o1_blind_kinodynamic_controller/benchmark_data";
    private static final String MASTER_SUMMARY_FILENAME = "simulation_master_summary.txt";

    private static final String[] OBSTACLES_CANDIDATE_PATHS = {
        "01_analytical_obstacle_mapping/maps/map_obstacles_calibrated.txt",
        "../01_analytical_obstacle_mapping/maps/map_obstacles_calibrated.txt"
    };

    private static final double SEPARACION_BORDE_M = 0.06;
    private static final double RADIO_CURVA_MAX_M = 0.65;
    private static final double MARGEN_SEGURIDAD_DEFAULT_M = 0.26;
    private static final double UMBRAL_MICRO_ESCALON_M = 0.22;
    private static final double COLLISION_EPSILON = 1e-9;
    private static final double VERTICAL_BORDER_X_RANGE_TOLERANCE = 0.0000011;

    private static final int BUTTON_Z = 1; // Reset pose/observador
    private static final int BUTTON_X = 2; // Inyectar impacto manual en vivo
    private static final int BUTTON_C = 3; // Iniciar/Detener simulaciones Monte Carlo en vivo

    // =========================================================================
    // 2. CLASES DE ESPECIFICACIÓN FÍSICA (IDÉNTICAS AL BENCHMARK PYTHON)
    // =========================================================================
    public static class RobotSpec {
        public final String name;
        public final double mNom;
        public final double izNom;
        public final double lx;
        public final double ly;
        public final double halfL;
        public final double halfW;
        public final double hCg;
        public final double rW;
        public final double jW;
        public final double fStall;
        public final double vFree;
        public final double tauDrive;
        public final double tauSteer;
        public final double maxSteerRate;
        public final double dragLin;
        public final double dragYaw;
        public final double[][] corners;
        public final double rSq;

        public RobotSpec(String name, double mNom, double izNom, double lx, double ly,
                         double halfL, double halfW, double hCg, double rW, double jW,
                         double fStall, double vFree, double tauDrive, double tauSteer,
                         double maxSteerRate, double dragLin, double dragYaw) {
            this.name = name;
            this.mNom = mNom;
            this.izNom = izNom;
            this.lx = lx;
            this.ly = ly;
            this.halfL = halfL;
            this.halfW = halfW;
            this.hCg = hCg;
            this.rW = rW;
            this.jW = jW;
            this.fStall = fStall;
            this.vFree = vFree;
            this.tauDrive = tauDrive;
            this.tauSteer = tauSteer;
            this.maxSteerRate = maxSteerRate;
            this.dragLin = dragLin;
            this.dragYaw = dragYaw;
            this.corners = new double[][]{
                {+lx, +ly}, // FL (0)
                {+lx, -ly}, // FR (1)
                {-lx, +ly}, // RL (2)
                {-lx, -ly}  // RR (3)
            };
            this.rSq = lx * lx + ly * ly;
        }
    }

    private static final RobotSpec[] ROBOT_CLASSES = new RobotSpec[]{
        new RobotSpec("Class1_AgileMini_32kg", 32.0, 1.65, 0.21, 0.21, 0.31, 0.31,
            0.16, 0.038, 0.008, 220.0, 5.50, 0.006, 0.009, 24.0, 2.5, 0.40),
        new RobotSpec("Class2_Standard_72kg", 72.0, 5.20, 0.29, 0.29, 0.42, 0.42,
            0.24, 0.0508, 0.018, 320.0, 4.80, 0.010, 0.014, 18.0, 4.5, 0.85),
        new RobotSpec("Class3_HeavyAMR_145kg", 145.0, 16.5, 0.42, 0.38, 0.56, 0.52,
            0.34, 0.076, 0.055, 480.0, 3.80, 0.016, 0.022, 13.0, 8.0, 1.80)
    };

    public static class EstadoTrayectoria {
        public final double x, y, theta, v, t;
        public EstadoTrayectoria(double x, double y, double theta, double v, double t) {
            this.x = x;
            this.y = y;
            this.theta = theta;
            this.v = v;
            this.t = t;
        }
    }

    private static class CollisionEvent {
        final double tStart;
        final double duration;
        final double jImp;
        final double nx, ny;
        final double rcx, rcy;
        final String sourceType;

        CollisionEvent(double tStart, double duration, double jImp, double nx, double ny,
                       double rcx, double rcy, String sourceType) {
            this.tStart = tStart;
            this.duration = duration;
            this.jImp = jImp;
            this.nx = nx;
            this.ny = ny;
            this.rcx = rcx;
            this.rcy = rcy;
            this.sourceType = sourceType;
        }
    }

    private static class ImpactRecord {
        final int id;
        final String type;
        final double timeSec;
        final double x, y, yawDeg;
        final double impulseNs;
        final double peakForceN;
        final double nx, ny;
        final double rcx, rcy;
        final double deltaVMps;
        final double deltaOmegaRps;
        double peakPostDeflectionMm = 0.0;
        double recoveryTimeMs = -1.0;

        ImpactRecord(int id, String type, double timeSec, double x, double y, double yawDeg,
                     double impulseNs, double peakForceN, double nx, double ny,
                     double rcx, double rcy, double deltaVMps, double deltaOmegaRps) {
            this.id = id;
            this.type = type;
            this.timeSec = timeSec;
            this.x = x;
            this.y = y;
            this.yawDeg = yawDeg;
            this.impulseNs = impulseNs;
            this.peakForceN = peakForceN;
            this.nx = nx;
            this.ny = ny;
            this.rcx = rcx;
            this.rcy = rcy;
            this.deltaVMps = deltaVMps;
            this.deltaOmegaRps = deltaOmegaRps;
        }
    }

    private static class TraceSample {
        final double t, refX, refY, refYawDeg, trueX, trueY, trueYawDeg;
        final double estX, estY, odomX, odomY, vxW, vyW, omega;
        final double posErrMm, yawErrDeg, muTrue, muHat;
        final boolean onIce, inCollision;
        final double colForceN, slipComb;
        final double n0, n1, n2, n3;
        final double latencyUs;
        final String eventFlag;

        TraceSample(double t, double refX, double refY, double refYawDeg,
                    double trueX, double trueY, double trueYawDeg,
                    double estX, double estY, double odomX, double odomY,
                    double vxW, double vyW, double omega,
                    double posErrMm, double yawErrDeg, double muTrue, double muHat,
                    boolean onIce, boolean inCollision, double colForceN, double slipComb,
                    double n0, double n1, double n2, double n3,
                    double latencyUs, String eventFlag) {
            this.t = t;
            this.refX = refX;
            this.refY = refY;
            this.refYawDeg = refYawDeg;
            this.trueX = trueX;
            this.trueY = trueY;
            this.trueYawDeg = trueYawDeg;
            this.estX = estX;
            this.estY = estY;
            this.odomX = odomX;
            this.odomY = odomY;
            this.vxW = vxW;
            this.vyW = vyW;
            this.omega = omega;
            this.posErrMm = posErrMm;
            this.yawErrDeg = yawErrDeg;
            this.muTrue = muTrue;
            this.muHat = muHat;
            this.onIce = onIce;
            this.inCollision = inCollision;
            this.colForceN = colForceN;
            this.slipComb = slipComb;
            this.n0 = n0;
            this.n1 = n1;
            this.n2 = n2;
            this.n3 = n3;
            this.latencyUs = latencyUs;
            this.eventFlag = eventFlag;
        }
    }

    // =========================================================================
    // 3. ESTADO DEL SIMULADOR, PLANTA Y TELEMETRÍA
    // =========================================================================
    private int m_selectedClassIdx = 1; // Default: Class2_Standard_72kg
    private RobotSpec m_spec = ROBOT_CLASSES[m_selectedClassIdx];

    private Joystick m_joystick;
    private final Random m_rng = new Random(2026L);
    private Path m_exportDir;

    private MultiClassSwerveSE2Plant m_plant;
    private ProposedO1SwerveController m_controller;

    private Pose2d m_trueWorldPose = new Pose2d(2.0, 2.0, new Rotation2d());
    private Pose2d m_estimatedObserverPose = new Pose2d(2.0, 2.0, new Rotation2d());
    private Pose2d m_rawWheelOdometryPose = new Pose2d(2.0, 2.0, new Rotation2d());
    private Pose2d m_ghostPoseGlobal = new Pose2d(2.0, 2.0, new Rotation2d());

    // Parámetros reales ocultos del ensayo actual (Blind Following)
    private double m_mTrueActual = 72.0;
    private double m_izTrueActual = 5.20;
    private double[] m_motorHealthActual = new double[]{1.0, 1.0, 1.0, 1.0};
    private double m_muNomActual = 0.84;
    private double m_muIceActual = 0.18;
    private double m_roughnessActual = 0.35;
    private final List<double[]> m_blackIceZones = new ArrayList<>(); // [xmin, xmax, ymin, ymax]
    private final List<CollisionEvent> m_scheduledCollisions = new ArrayList<>();
    private final boolean[] m_collisionTriggered = new boolean[16];

    // Obstáculos calibrados del campo
    private final List<double[][]> m_obstaculos = new ArrayList<>();
    private final List<double[]> m_cajasObstaculosReales = new ArrayList<>();

    // Trayectorias y resumen geométrico en forma de L
    private List<EstadoTrayectoria> m_trayectoriaActiva = new ArrayList<>();
    private final List<double[]> m_plannedLVertices = new ArrayList<>();
    private final List<double[]> m_rawActualPathPoints = new ArrayList<>(); // [x, y, psi, t]
    private final List<ImpactRecord> m_trialImpacts = new ArrayList<>();
    private final List<TraceSample> m_trialTraceSamples = new ArrayList<>();

    // Métricas en vivo del episodio
    private double m_simTrialTime = 0.0;
    private double m_refTrajectoryTime = 0.0;
    private double m_totalTrajectoryDuration = 0.0;
    private double m_lastWallImpactLogTime = -10.0;
    private double m_lastReplanTime = -10.0;
    private double m_stuckTimer = 0.0;
    private int m_trialCount = 0;

    private double m_trialSqErrSum = 0.0;
    private double m_trialMaxErrMm = 0.0;
    private double m_trialPeakYawErrDeg = 0.0;
    private double m_trialPeakIceErrMm = 0.0;
    private double m_trialPeakColErrMm = 0.0;
    private double m_trialSqSlipSum = 0.0;
    private int m_trialSamples = 0;
    private int m_tpIceSteps = 0, m_fpIceSteps = 0, m_iceGtSteps = 0, m_nonIceGtSteps = 0;
    private int m_deadlineMissCount = 0;
    private final List<Double> m_stepLatenciesUs = new ArrayList<>();
    private String m_pendingEventFlag = "NORMAL";

    private boolean m_siguiendoRuta = false;
    private boolean m_modoPruebasAutomaticas = false;
    private boolean m_physicsRanThisLoop = false;
    private String m_ultimoEstadoPlanificador = "Listo. Presiona [C] o activa 'Run Live Benchmark' en Shuffleboard.";

    // Publicadores AdvantageScope / NetworkTables
    private final List<Pose2d> m_impactMarkerPoses = new ArrayList<>();
    private StructPublisher<Pose2d> m_trueWorldPosePublisher;
    private StructPublisher<Pose2d> m_estimatedPosePublisher;
    private StructPublisher<Pose2d> m_rawOdometryPosePublisher;
    private StructPublisher<Pose2d> m_ghostPosePublisher;
    private StructPublisher<Pose2d> m_startPosePublisher;
    private StructPublisher<Pose2d> m_endPosePublisher;
    private StructPublisher<Pose2d> m_lastImpactPosePublisher;
    private StructArrayPublisher<Pose2d> m_trajectoryPublisher;
    private StructArrayPublisher<Pose2d> m_plannedLPathPublisher;
    private StructArrayPublisher<Pose2d> m_actualLPathSummaryPublisher;
    private StructArrayPublisher<Pose2d> m_impactMarkersPublisher;
    private StructArrayPublisher<Pose2d> m_iceZonesPublisher;
    private StructArrayPublisher<Pose2d> m_icePolygonBoundsPublisher;
    private StructArrayPublisher<SwerveModuleState> m_measuredSwerveStatesPub;
    private StructArrayPublisher<SwerveModuleState> m_desiredSwerveStatesPub;

    private DoublePublisher m_muHatPublisher;
    private DoublePublisher m_muTruePublisher;
    private DoublePublisher m_trackingErrorMmPublisher;
    private DoublePublisher m_yawErrorDegPublisher;
    private DoublePublisher m_stepLatencyUsPublisher;
    private DoublePublisher m_combinedSlipPublisher;
    private DoubleArrayPublisher m_normalLoadsPublisher;
    private BooleanPublisher m_inCollisionPublisher;
    private BooleanPublisher m_inBlackIcePublisher;
    private StringPublisher m_lastImpactInfoPublisher;
    private StringPublisher m_trialVerdictPublisher;

    // Shuffleboard
    private ShuffleboardTab m_tab;
    private GenericEntry m_fieldEntry;
    private GenericEntry m_runBenchmarkEntry;
    private GenericEntry m_robotClassEntry;
    private GenericEntry m_collisionsEnabledEntry;
    private GenericEntry m_safetyMarginEntry;
    private GenericEntry m_plannerStatusEntry;
    private GenericEntry m_observerTelemetryEntry;
    private Field2d m_field2d;

    private AprilTagFieldLayout m_fieldLayout;
    private String m_loadedFieldName = null;

    // =========================================================================
    // 4. INICIALIZACIÓN WPILIB Y CARPETA DE EXPORTACIÓN TXT
    // =========================================================================
    @Override
    public void robotInit() {
        m_joystick = new Joystick(0);
        inicializarCarpetaExportacion();

        m_plant = new MultiClassSwerveSE2Plant(m_spec, m_spec.mNom, m_spec.izNom, new double[]{1, 1, 1, 1}, DT_CONTROL, PHYS_SUBSTEPS);
        m_controller = new ProposedO1SwerveController(m_spec, DT_CONTROL, true);

        m_tab = Shuffleboard.getTab("Navigation");
        m_fieldEntry = m_tab.add("Field Layout", DEFAULT_FIELD).withPosition(0, 0).withSize(2, 1).getEntry();
        m_runBenchmarkEntry = m_tab.add("Run Live Benchmark", false).withWidget(BuiltInWidgets.kToggleButton).withPosition(2, 0).withSize(2, 1).getEntry();
        m_robotClassEntry = m_tab.add("Robot Class (0=32kg,1=72kg,2=145kg)", m_selectedClassIdx).withPosition(4, 0).withSize(2, 1).getEntry();
        m_collisionsEnabledEntry = m_tab.add("Collisions Enabled", true).withWidget(BuiltInWidgets.kToggleButton).withPosition(6, 0).withSize(2, 1).getEntry();
        m_safetyMarginEntry = m_tab.add("Safety Margin (m)", MARGEN_SEGURIDAD_DEFAULT_M).withPosition(8, 0).withSize(2, 1).getEntry();

        m_plannerStatusEntry = m_tab.add("Planner Status", m_ultimoEstadoPlanificador).withPosition(0, 6).withSize(5, 1).getEntry();
        m_observerTelemetryEntry = m_tab.add("O(1) Kinodynamic Control", "mu_hat: 0.84 | Ready").withPosition(5, 6).withSize(5, 1).getEntry();

        m_field2d = new Field2d();
        m_tab.add("Field2d", m_field2d).withPosition(0, 1).withSize(10, 5);

        NetworkTableInstance nt = NetworkTableInstance.getDefault();
        m_trueWorldPosePublisher = nt.getStructTopic("SmartDashboard/RobotPose", Pose2d.struct).publish();
        m_estimatedPosePublisher = nt.getStructTopic("SmartDashboard/PathPlanner/EstimatedObserverPose", Pose2d.struct).publish();
        m_rawOdometryPosePublisher = nt.getStructTopic("SmartDashboard/PathPlanner/RawWheelOdometryPose", Pose2d.struct).publish();
        m_ghostPosePublisher = nt.getStructTopic("SmartDashboard/PathPlanner/GhostPoseGlobal", Pose2d.struct).publish();
        m_startPosePublisher = nt.getStructTopic("SmartDashboard/PathPlanner/StartPose", Pose2d.struct).publish();
        m_endPosePublisher = nt.getStructTopic("SmartDashboard/PathPlanner/EndPose", Pose2d.struct).publish();
        m_lastImpactPosePublisher = nt.getStructTopic("SmartDashboard/PathPlanner/LastImpactPose", Pose2d.struct).publish();

        m_trajectoryPublisher = nt.getStructArrayTopic("SmartDashboard/PathPlanner/Trajectory", Pose2d.struct).publish();
        m_plannedLPathPublisher = nt.getStructArrayTopic("SmartDashboard/PathPlanner/PlannedLPathVertices", Pose2d.struct).publish();
        m_actualLPathSummaryPublisher = nt.getStructArrayTopic("SmartDashboard/PathPlanner/ActualTakenPathSummary", Pose2d.struct).publish();
        m_impactMarkersPublisher = nt.getStructArrayTopic("SmartDashboard/PathPlanner/ImpactMarkers", Pose2d.struct).publish();
        m_iceZonesPublisher = nt.getStructArrayTopic("SmartDashboard/PathPlanner/BlackIceZones", Pose2d.struct).publish();
        m_icePolygonBoundsPublisher = nt.getStructArrayTopic("SmartDashboard/PathPlanner/BlackIcePolygonBounds", Pose2d.struct).publish();

        m_measuredSwerveStatesPub = nt.getStructArrayTopic("SmartDashboard/Swerve/MeasuredStates", SwerveModuleState.struct).publish();
        m_desiredSwerveStatesPub = nt.getStructArrayTopic("SmartDashboard/Swerve/DesiredStates", SwerveModuleState.struct).publish();

        m_muHatPublisher = nt.getDoubleTopic("SmartDashboard/Control/EstimatedMuHat").publish();
        m_muTruePublisher = nt.getDoubleTopic("SmartDashboard/Control/TrueLocalMu").publish();
        m_trackingErrorMmPublisher = nt.getDoubleTopic("SmartDashboard/Control/TrackingError_mm").publish();
        m_yawErrorDegPublisher = nt.getDoubleTopic("SmartDashboard/Control/YawError_deg").publish();
        m_stepLatencyUsPublisher = nt.getDoubleTopic("SmartDashboard/Control/StepLatency_us").publish();
        m_combinedSlipPublisher = nt.getDoubleTopic("SmartDashboard/Control/CombinedTireSlip").publish();
        m_normalLoadsPublisher = nt.getDoubleArrayTopic("SmartDashboard/Control/DynamicNormalLoads_N").publish();
        m_inCollisionPublisher = nt.getBooleanTopic("SmartDashboard/Control/InCollision").publish();
        m_inBlackIcePublisher = nt.getBooleanTopic("SmartDashboard/Control/InBlackIceTrip").publish();
        m_lastImpactInfoPublisher = nt.getStringTopic("SmartDashboard/PathPlanner/LastImpactInfo").publish();
        m_trialVerdictPublisher = nt.getStringTopic("SmartDashboard/PathPlanner/LastTrialVerdict").publish();

        cargarCampo();
        cargarObstaculosDesdeArchivo(m_loadedFieldName);
        posicionarEnReposoSeguro(new Pose2d(2.0, 2.0, new Rotation2d()));
    }

    private void inicializarCarpetaExportacion() {
        try {
            Path preferred = Paths.get(EXPORT_FOLDER_WIN);
            Path parent = preferred.getParent();
            if (parent != null && Files.exists(parent)) {
                Files.createDirectories(preferred);
                m_exportDir = preferred;
            } else {
                Path fallback = Paths.get(System.getProperty("user.home"), "Downloads", "Java advantage scope data");
                Files.createDirectories(fallback);
                m_exportDir = fallback;
            }
            Path masterFile = m_exportDir.resolve(MASTER_SUMMARY_FILENAME);
            if (!Files.exists(masterFile)) {
                String header = String.join("\n",
                    "==========================================================================================================",
                    "  O(1) ANALYTIC DYNAMIC-LOAD & GRIP-AWARE SWERVE CONTROL — LIVE ADVANTAGESCOPE SIMULATION TELEMETRY LOG",
                    "  Generated: " + LocalDateTime.now().format(DateTimeFormatter.ISO_LOCAL_DATE_TIME),
                    "  Control Loop: 500 Hz (dt = 0.002 s) | Physics Integration: 5000 Hz (dt_sub = 0.0002 s) | Pacejka + SAT",
                    "==========================================================================================================",
                    ""
                );
                Files.writeString(masterFile, header, StandardCharsets.UTF_8, StandardOpenOption.CREATE);
            }
            System.out.println("[EXPORT OK] Telemetría TXT configurada en: " + m_exportDir.toAbsolutePath());
        } catch (IOException e) {
            System.err.println("[EXPORT ERROR] No se pudo crear carpeta de datos: " + e.getMessage());
        }
    }

    private void posicionarEnReposoSeguro(Pose2d pose) {
        m_plant.reset(new double[]{pose.getX(), pose.getY(), pose.getRotation().getRadians(), 0, 0, 0}, pose.getRotation().getRadians());
        m_controller.reset(0.84, pose.getX(), pose.getY(), pose.getRotation().getRadians());
        m_trueWorldPose = pose;
        m_estimatedObserverPose = pose;
        m_rawWheelOdometryPose = pose;
        m_ghostPoseGlobal = pose;
    }

    @Override
    public void robotPeriodic() {
        m_physicsRanThisLoop = false; // Reiniciar guardia de ejecución única de 20 ms
        comprobarCambioDeMapaYClase();

        // Sincronizar botón toggle de Shuffleboard con tecla C
        if (m_runBenchmarkEntry != null) {
            boolean sbToggle = m_runBenchmarkEntry.getBoolean(false);
            if (sbToggle && !m_modoPruebasAutomaticas) {
                m_modoPruebasAutomaticas = true;
                System.out.println("[BENCHMARK] Bucle Monte Carlo en vivo ACTIVADO desde Shuffleboard.");
                ejecutarSiguientePruebaAleatoria();
            } else if (!sbToggle && m_modoPruebasAutomaticas) {
                m_modoPruebasAutomaticas = false;
                m_siguiendoRuta = false;
                m_trayectoriaActiva.clear();
                m_ultimoEstadoPlanificador = "Bucle detenido desde Shuffleboard.";
            }
        }

        // Publicar poses en AdvantageScope y Field2d
        if (Double.isFinite(m_trueWorldPose.getX()) && Double.isFinite(m_trueWorldPose.getY())) {
            m_trueWorldPosePublisher.set(m_trueWorldPose);
            m_field2d.setRobotPose(m_trueWorldPose);
        }
        m_estimatedPosePublisher.set(m_estimatedObserverPose);
        m_rawOdometryPosePublisher.set(m_rawWheelOdometryPose);
        m_ghostPosePublisher.set(m_ghostPoseGlobal);
        m_field2d.getObject("GhostPose").setPose(m_ghostPoseGlobal);
        m_field2d.getObject("ImpactMarkers").setPoses(new ArrayList<>(m_impactMarkerPoses));

        // Publicar resumen en vivo del camino real tomado en forma de L
        if (!m_rawActualPathPoints.isEmpty()) {
            List<double[]> simplifiedActual = simplificarPolilineaRDP(m_rawActualPathPoints, 0.065);
            Pose2d[] actualPoses = new Pose2d[simplifiedActual.size()];
            List<Pose2d> actualList = new ArrayList<>(simplifiedActual.size());
            for (int i = 0; i < simplifiedActual.size(); i++) {
                double[] pt = simplifiedActual.get(i);
                Pose2d p = new Pose2d(pt[0], pt[1], new Rotation2d(pt[2]));
                actualPoses[i] = p;
                actualList.add(p);
            }
            m_actualLPathSummaryPublisher.set(actualPoses);
            m_field2d.getObject("ActualSummaryPath").setPoses(actualList);
        }

        // Publicar estados reales y deseados de los 4 módulos swerve
        SwerveModuleState[] measStates = new SwerveModuleState[4];
        SwerveModuleState[] desStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            measStates[i] = new SwerveModuleState(m_plant.omegaW[i] * m_spec.rW, Rotation2d.fromRadians(m_plant.deltas[i]));
            double vDes = Math.max(-m_spec.vFree, Math.min(m_spec.vFree, m_controller.lastFCmds[i] / Math.max(1.0, m_spec.fStall / m_spec.vFree)));
            desStates[i] = new SwerveModuleState(vDes, Rotation2d.fromRadians(m_controller.lastDCmds[i]));
        }
        m_measuredSwerveStatesPub.set(measStates);
        m_desiredSwerveStatesPub.set(desStates);

        double errMm = Math.hypot(
            m_trueWorldPose.getX() - m_ghostPoseGlobal.getX(),
            m_trueWorldPose.getY() - m_ghostPoseGlobal.getY()
        ) * 1000.0;
        double yawErrDeg = Math.abs(Math.toDegrees(wrapAngle(
            m_trueWorldPose.getRotation().getRadians() - m_ghostPoseGlobal.getRotation().getRadians()
        )));

        m_plannerStatusEntry.setString(m_ultimoEstadoPlanificador);
        m_observerTelemetryEntry.setString(String.format(
            Locale.US, "%s | mu_hat: %.2f | Col: %s | Ice: %s | Err: %.0f mm | Lat: %.1f us",
            m_spec.name, m_controller.muHat,
            m_controller.inCollision ? "SHOCK" : "OK",
            m_controller.inIceTrip ? "SLIP" : "DRY",
            errMm, m_controller.lastStepLatencyUs
        ));

        m_muHatPublisher.set(m_controller.muHat);
        m_muTruePublisher.set(m_plant.lastLocalMu);
        m_trackingErrorMmPublisher.set(errMm);
        m_yawErrorDegPublisher.set(yawErrDeg);
        m_stepLatencyUsPublisher.set(m_controller.lastStepLatencyUs);
        m_combinedSlipPublisher.set(m_plant.lastCombinedSlip);
        m_normalLoadsPublisher.set(m_controller.lastNormalLoads);
        m_inCollisionPublisher.set(m_controller.inCollision);
        m_inBlackIcePublisher.set(m_controller.inIceTrip);

        // Tecla Z: Reset limpio
        if (m_joystick.getRawButtonPressed(BUTTON_Z)) {
            posicionarEnReposoSeguro(m_trueWorldPose);
            m_ultimoEstadoPlanificador = "Observador y odometría reiniciados.";
        }

        // Tecla X: Inyectar choque dinámico SE(2) en vivo
        if (m_joystick.getRawButtonPressed(BUTTON_X)) {
            inyectarChoqueManual();
        }

        // Tecla C: Iniciar / Detener bucle continuo Monte Carlo
        if (m_joystick.getRawButtonPressed(BUTTON_C)) {
            m_modoPruebasAutomaticas = !m_modoPruebasAutomaticas;
            if (m_runBenchmarkEntry != null) m_runBenchmarkEntry.setBoolean(m_modoPruebasAutomaticas);
            if (m_modoPruebasAutomaticas) {
                System.out.println("[BENCHMARK] Bucle Monte Carlo en vivo ACTIVADO [Tecla C].");
                ejecutarSiguientePruebaAleatoria();
            } else {
                m_siguiendoRuta = false;
                m_trayectoriaActiva.clear();
                m_ultimoEstadoPlanificador = "Bucle detenido por usuario [Tecla C].";
            }
        }
    }

    @Override
    public void teleopPeriodic() {
        // Solo ejecutar aquí si es robot físico real (en simulación lo ejecuta simulationPeriodic una sola vez)
        if (!RobotBase.isSimulation()) {
            ejecutarUnicoPasoDeFrame();
        }
    }

    @Override
    public void simulationPeriodic() {
        ejecutarUnicoPasoDeFrame();
    }

    private void ejecutarUnicoPasoDeFrame() {
        if (m_physicsRanThisLoop) return;
        m_physicsRanThisLoop = true;
        procesarCicloFisicoYControl();
    }

    // =========================================================================
    // 5. GENERACIÓN DE PRUEBAS CON CAMINO EN L Y ENTORNOS ESTOCÁSTICOS
    // =========================================================================
    private void ejecutarSiguientePruebaAleatoria() {
        m_siguiendoRuta = false;
        m_trayectoriaActiva.clear();
        m_plannedLVertices.clear();
        m_rawActualPathPoints.clear();
        m_trialImpacts.clear();
        m_trialTraceSamples.clear();
        m_stepLatenciesUs.clear();
        m_trialCount++;

        double safetyMargin = m_safetyMarginEntry != null
            ? m_safetyMarginEntry.getDouble(MARGEN_SEGURIDAD_DEFAULT_M)
            : MARGEN_SEGURIDAD_DEFAULT_M;

        List<EstadoTrayectoria> rutaCandidata = null;
        List<double[]> verticesLCandidatos = null;
        Pose2d startPose = null;
        Pose2d endPose = null;

        for (int intento = 0; intento < 100; intento++) {
            boolean izqADer = m_rng.nextBoolean();
            double x1 = izqADer ? (1.2 + m_rng.nextDouble() * 2.0) : (13.3 + m_rng.nextDouble() * 2.0);
            double y1 = 1.1 + m_rng.nextDouble() * (CAMPO_ANCHO_M - 2.2);
            double theta1 = normalizarAngulo((m_rng.nextDouble() - 0.5) * 1.6);

            double x2 = izqADer ? (13.3 + m_rng.nextDouble() * 2.0) : (1.2 + m_rng.nextDouble() * 2.0);
            double y2 = 1.1 + m_rng.nextDouble() * (CAMPO_ANCHO_M - 2.2);
            double theta2 = normalizarAngulo((m_rng.nextDouble() - 0.5) * 1.6);

            // Punto intermedio desplazado lateralmente para forzar geometría clara en forma de L
            double xMid = 0.5 * (x1 + x2) + (m_rng.nextDouble() - 0.5) * 2.0;
            double yMid = (y1 < 0.5 * CAMPO_ANCHO_M)
                ? (4.6 + m_rng.nextDouble() * 2.2)
                : (1.2 + m_rng.nextDouble() * 2.2);

            Pose2d pStart = new Pose2d(x1, y1, new Rotation2d(theta1));
            Pose2d pMid   = new Pose2d(xMid, yMid, new Rotation2d(theta1));
            Pose2d pEnd   = new Pose2d(x2, y2, new Rotation2d(theta2));

            double checkMargin = safetyMargin + 0.12;
            if (verificarPosicionRobotLibre(pStart.getX(), pStart.getY(), theta1, checkMargin)
                && verificarPosicionRobotLibre(pMid.getX(), pMid.getY(), theta1, checkMargin)
                && verificarPosicionRobotLibre(pEnd.getX(), pEnd.getY(), theta2, checkMargin)) {

                List<double[]> outLVertices = new ArrayList<>();
                List<EstadoTrayectoria> cand = obtenerTrayectoriaCompletaM(
                    Arrays.asList(pStart, pMid, pEnd), safetyMargin, outLVertices
                );
                if (cand != null && cand.size() >= 25 && outLVertices.size() >= 3) {
                    rutaCandidata = cand;
                    verticesLCandidatos = outLVertices;
                    startPose = pStart;
                    endPose = pEnd;
                    break;
                }
            }
        }

        if (rutaCandidata == null || rutaCandidata.isEmpty()) {
            m_ultimoEstadoPlanificador = "Buscando ruta libre de obstáculos...";
            return;
        }

        m_trayectoriaActiva = rutaCandidata;
        m_plannedLVertices.addAll(verticesLCandidatos);
        m_totalTrajectoryDuration = rutaCandidata.get(rutaCandidata.size() - 1).t;

        // Reiniciar métricas de episodio
        m_simTrialTime = 0.0;
        m_refTrajectoryTime = 0.0;
        m_lastWallImpactLogTime = -10.0;
        m_lastReplanTime = -10.0;
        m_stuckTimer = 0.0;
        m_trialSqErrSum = 0.0;
        m_trialMaxErrMm = 0.0;
        m_trialPeakYawErrDeg = 0.0;
        m_trialPeakIceErrMm = 0.0;
        m_trialPeakColErrMm = 0.0;
        m_trialSqSlipSum = 0.0;
        m_trialSamples = 0;
        m_tpIceSteps = m_fpIceSteps = m_iceGtSteps = m_nonIceGtSteps = 0;
        m_deadlineMissCount = 0;
        m_pendingEventFlag = "TRIAL_START";

        // 1. Aleatorizar parámetros físicos reales de la planta (idéntico a Python)
        m_mTrueActual = m_spec.mNom * (0.92 + m_rng.nextDouble() * 0.20);
        m_izTrueActual = m_spec.izNom * (0.92 + m_rng.nextDouble() * 0.20);
        m_motorHealthActual = new double[4];
        for (int i = 0; i < 4; i++) {
            m_motorHealthActual[i] = 0.82 + m_rng.nextDouble() * 0.18;
        }

        m_muNomActual = 0.84;
        m_muIceActual = m_rng.nextDouble() < 0.33 ? 0.10 : 0.18;
        m_roughnessActual = (m_muIceActual < 0.15) ? 0.70 : 0.35;

        // 2. Generar 1 o 2 parches de hielo negro sobre los tramos de la ruta
        m_blackIceZones.clear();
        List<Pose2d> iceCenters = new ArrayList<>();
        List<Pose2d> iceBounds = new ArrayList<>();
        int numIcePatches = 1 + m_rng.nextInt(2);
        for (int p = 0; p < numIcePatches; p++) {
            double frac = (p == 0) ? (0.30 + m_rng.nextDouble() * 0.15) : (0.62 + m_rng.nextDouble() * 0.15);
            int idx = Math.min(rutaCandidata.size() - 1, (int) (frac * rutaCandidata.size()));
            EstadoTrayectoria stIce = rutaCandidata.get(idx);
            double hx = 0.85 + m_rng.nextDouble() * 0.40;
            double hy = 0.90 + m_rng.nextDouble() * 0.40;
            double[] z = new double[]{stIce.x - hx, stIce.x + hx, stIce.y - hy, stIce.y + hy};
            m_blackIceZones.add(z);
            iceCenters.add(new Pose2d(stIce.x, stIce.y, new Rotation2d()));
            // Contorno cerrado del parche de hielo para AdvantageScope
            iceBounds.add(new Pose2d(z[0], z[2], new Rotation2d()));
            iceBounds.add(new Pose2d(z[1], z[2], new Rotation2d()));
            iceBounds.add(new Pose2d(z[1], z[3], new Rotation2d()));
            iceBounds.add(new Pose2d(z[0], z[3], new Rotation2d()));
            iceBounds.add(new Pose2d(z[0], z[2], new Rotation2d()));
        }
        m_iceZonesPublisher.set(iceCenters.toArray(new Pose2d[0]));
        m_icePolygonBoundsPublisher.set(iceBounds.toArray(new Pose2d[0]));
        m_field2d.getObject("BlackIceZones").setPoses(iceCenters);

        // 3. Programar 2 choques dinámicos SE(2) aleatorios (impulso medio seno 20 ms)
        m_scheduledCollisions.clear();
        Arrays.fill(m_collisionTriggered, false);
        double[] fracs = {0.26 + m_rng.nextDouble() * 0.08, 0.64 + m_rng.nextDouble() * 0.10};
        for (double f : fracs) {
            double tCol = f * m_totalTrajectoryDuration;
            if (tCol > 0.35 && tCol < m_totalTrajectoryDuration - 0.35) {
                m_scheduledCollisions.add(crearEventoChoqueAleatorio(tCol, "DYNAMIC_ROBOT_IMPACT"));
            }
        }

        // 4. Inicializar planta física y controlador O(1) en reposo exacto (cero salto inicial)
        m_plant = new MultiClassSwerveSE2Plant(m_spec, m_mTrueActual, m_izTrueActual, m_motorHealthActual, DT_CONTROL, PHYS_SUBSTEPS);
        double initTangent = (rutaCandidata.size() > 1)
            ? Math.atan2(rutaCandidata.get(1).y - startPose.getY(), rutaCandidata.get(1).x - startPose.getX())
            : startPose.getRotation().getRadians();
        m_plant.reset(
            new double[]{startPose.getX(), startPose.getY(), startPose.getRotation().getRadians(), 0.0, 0.0, 0.0},
            initTangent
        );
        m_controller.reset(0.84, startPose.getX(), startPose.getY(), startPose.getRotation().getRadians());

        m_trueWorldPose = startPose;
        m_estimatedObserverPose = startPose;
        m_rawWheelOdometryPose = startPose;
        m_ghostPoseGlobal = startPose;
        m_rawActualPathPoints.add(new double[]{startPose.getX(), startPose.getY(), startPose.getRotation().getRadians(), 0.0});

        m_impactMarkerPoses.clear();
        m_impactMarkersPublisher.set(new Pose2d[0]);
        m_lastImpactInfoPublisher.set("Sin impactos todavía");
        m_field2d.getObject("ImpactMarkers").setPoses(Collections.emptyList());

        publicarTrayectoriaVisual(rutaCandidata, verticesLCandidatos);
        m_startPosePublisher.set(startPose);
        m_endPosePublisher.set(endPose);

        m_siguiendoRuta = true;
        m_ultimoEstadoPlanificador = String.format(
            Locale.US, "Prueba #%d [%s] | m=%.1fkg | mu_ice=%.2f | L-Vertices=%d | T=%.2fs",
            m_trialCount, m_spec.name, m_mTrueActual, m_muIceActual, verticesLCandidatos.size(), m_totalTrajectoryDuration
        );
        System.out.println("[TRIAL START] " + m_ultimoEstadoPlanificador);
    }

    private CollisionEvent crearEventoChoqueAleatorio(double tStart, String sourceType) {
        double mOpp = (0.60 + m_rng.nextDouble() * 0.70) * m_spec.mNom;
        double vRelN = 0.75 + m_rng.nextDouble() * 0.70;
        double restitution = 0.25 + m_rng.nextDouble() * 0.30;
        double impactAngle = (m_rng.nextDouble() - 0.5) * 2.0 * Math.PI;
        double nx = Math.cos(impactAngle);
        double ny = Math.sin(impactAngle);

        double rcx = (m_rng.nextBoolean() ? 1.0 : -1.0) * m_spec.lx;
        double rcy = (m_rng.nextDouble() * 1.70 - 0.85) * m_spec.ly;
        double rCrossN = rcx * ny - rcy * nx;

        double invEffMass = (1.0 / m_spec.mNom) + (1.0 / mOpp) + (rCrossN * rCrossN) / m_spec.izNom;
        double jImp = (1.0 + restitution) * vRelN / invEffMass;
        return new CollisionEvent(tStart, 0.020, jImp, nx, ny, rcx, rcy, sourceType);
    }

    private void inyectarChoqueManual() {
        CollisionEvent ev = crearEventoChoqueAleatorio(m_simTrialTime, "MANUAL_USER_IMPACT");
        m_scheduledCollisions.add(ev);
    }

    // =========================================================================
    // 6. BUCLE FÍSICO Y CONTROL A 500 HZ (10 SUBCICLOS DE 2 MS POR FRAME)
    // =========================================================================
    private void procesarCicloFisicoYControl() {
        double rawForward = -m_joystick.getRawAxis(1);
        double rawStrafe = -m_joystick.getRawAxis(0);
        double rawTurn = -m_joystick.getRawAxis(4);

        if (Math.abs(rawForward) > 0.12 || Math.abs(rawStrafe) > 0.12 || Math.abs(rawTurn) > 0.12) {
            if (m_siguiendoRuta) {
                m_siguiendoRuta = false;
                m_modoPruebasAutomaticas = false;
                if (m_runBenchmarkEntry != null) m_runBenchmarkEntry.setBoolean(false);
                m_ultimoEstadoPlanificador = "Interrupción manual por Joystick.";
            }
        }

        boolean collisionsEnabled = (m_collisionsEnabledEntry == null) || m_collisionsEnabledEntry.getBoolean(true);

        if (m_siguiendoRuta && !m_trayectoriaActiva.isEmpty()) {
            boolean wasOnIceFrame = false;
            boolean wasInColFrame = false;
            double maxColForceFrame = 0.0;

            for (int sub = 0; sub < CONTROL_SUBCYCLES; sub++) {
                double[] zNoise = new double[24];
                for (int k = 0; k < 24; k++) zNoise[k] = m_rng.nextGaussian();

                // Mediciones ruidosas (Blind Following: sin mapa de hielo ni fuerza externa)
                double[] measVelW = m_plant.getMeasuredVelWorld(zNoise);
                double[] measAccB = m_plant.getMeasuredAccelBody(zNoise);
                double[] measWs   = m_plant.getMeasuredWheelSpeeds(zNoise);
                double[] measDel  = m_plant.getMeasuredDeltas(zNoise);

                // Reloj elástico de referencia (evita adelantarse si hubo choque fuerte)
                double[] ref0 = muestrearReferencia(m_trayectoriaActiva, m_refTrajectoryTime);
                double posErrEst = Math.hypot(m_controller.ox - ref0[0], m_controller.oy - ref0[1]);
                double dtRef = DT_CONTROL / (1.0 + 1.25 * Math.max(0.0, posErrEst - 0.30));
                m_refTrajectoryTime = Math.min(m_totalTrajectoryDuration, m_refTrajectoryTime + dtRef);

                double[] refCurr = muestrearReferencia(m_trayectoriaActiva, m_refTrajectoryTime);
                double[] refNext = muestrearReferencia(m_trayectoriaActiva, Math.min(m_totalTrajectoryDuration, m_refTrajectoryTime + dtRef));

                // Paso exacto del controlador analítico O(1) validado en Python
                ControllerOutput out = m_controller.step(
                    measVelW[2], measAccB, measWs, measDel, refCurr, refNext
                );
                m_stepLatenciesUs.add(out.latencyUs);
                if (out.latencyUs > 2000.0) m_deadlineMissCount++;

                // Evaluar choques dinámicos programados en m_simTrialTime
                double fxExt = 0.0, fyExt = 0.0, tzExt = 0.0;
                for (int cIdx = 0; cIdx < m_scheduledCollisions.size(); cIdx++) {
                    CollisionEvent ev = m_scheduledCollisions.get(cIdx);
                    if (m_simTrialTime >= ev.tStart && m_simTrialTime < ev.tStart + ev.duration) {
                        double tau = (m_simTrialTime - ev.tStart) / ev.duration;
                        double fMag = (0.5 * Math.PI / ev.duration) * ev.jImp * Math.sin(Math.PI * tau);
                        maxColForceFrame = Math.max(maxColForceFrame, fMag);
                        fxExt += fMag * ev.nx;
                        fyExt += fMag * ev.ny;
                        tzExt += ev.rcx * (fMag * ev.ny) - ev.rcy * (fMag * ev.nx);

                        if (cIdx < m_collisionTriggered.length && !m_collisionTriggered[cIdx]) {
                            m_collisionTriggered[cIdx] = true;
                            registrarImpactoEnVivo(
                                ev.sourceType, m_plant.state[0], m_plant.state[1], m_plant.state[2],
                                ev.jImp, (0.5 * Math.PI / ev.duration) * ev.jImp, ev.nx, ev.ny, ev.rcx, ev.rcy
                            );
                        }
                    }
                }

                // Avanzar planta física SE(2) a 5 kHz + resolución SAT multi-vértice
                m_plant.step(
                    out.fCmds, out.dCmds,
                    m_muNomActual, m_muIceActual, m_blackIceZones,
                    m_roughnessActual, fxExt, fyExt, tzExt,
                    zNoise, collisionsEnabled ? m_cajasObstaculosReales : Collections.emptyList()
                );

                // Registrar si hubo contacto físico con pared/obstáculo del campo
                if (m_plant.wallImpactOccurredThisStep && (m_simTrialTime - m_lastWallImpactLogTime) > 0.25) {
                    m_lastWallImpactLogTime = m_simTrialTime;
                    registrarImpactoEnVivo(
                        "FIELD_WALL_OR_OBSTACLE", m_plant.state[0], m_plant.state[1], m_plant.state[2],
                        m_plant.lastWallImpulseNs, m_plant.lastWallImpulseNs / DT_CONTROL,
                        m_plant.lastWallNx, m_plant.lastWallNy, m_plant.lastWallCrx, m_plant.lastWallCry
                    );
                }

                m_simTrialTime += DT_CONTROL;
                double errMm = Math.hypot(m_plant.state[0] - refCurr[0], m_plant.state[1] - refCurr[1]) * 1000.0;
                double yawDeg = Math.abs(Math.toDegrees(wrapAngle(m_plant.state[2] - refCurr[2])));

                m_trialSqErrSum += errMm * errMm;
                m_trialMaxErrMm = Math.max(m_trialMaxErrMm, errMm);
                m_trialPeakYawErrDeg = Math.max(m_trialPeakYawErrDeg, yawDeg);
                m_trialSqSlipSum += m_plant.lastCombinedSlip * m_plant.lastCombinedSlip;
                m_trialSamples++;

                if (m_plant.anyWheelOnIce) {
                    wasOnIceFrame = true;
                    m_trialPeakIceErrMm = Math.max(m_trialPeakIceErrMm, errMm);
                    m_iceGtSteps++;
                    if (m_controller.inIceTrip) m_tpIceSteps++;
                } else {
                    m_nonIceGtSteps++;
                    if (m_controller.inIceTrip) m_fpIceSteps++;
                }

                if (m_controller.inCollision || maxColForceFrame > 1.0) {
                    wasInColFrame = true;
                }

                // Actualizar deflexión pico y tiempo de recuperación de cada impacto registrado
                for (ImpactRecord imp : m_trialImpacts) {
                    double dtSince = m_simTrialTime - imp.timeSec;
                    if (dtSince >= 0.0 && dtSince <= 0.45) {
                        imp.peakPostDeflectionMm = Math.max(imp.peakPostDeflectionMm, errMm);
                        m_trialPeakColErrMm = Math.max(m_trialPeakColErrMm, errMm);
                        if (dtSince > 0.025 && errMm < 150.0 && imp.recoveryTimeMs < 0.0) {
                            imp.recoveryTimeMs = dtSince * 1000.0;
                        }
                    }
                }

                m_ghostPoseGlobal = new Pose2d(refCurr[0], refCurr[1], new Rotation2d(refCurr[2]));
            }

            m_trueWorldPose = new Pose2d(m_plant.state[0], m_plant.state[1], new Rotation2d(m_plant.state[2]));
            m_estimatedObserverPose = new Pose2d(m_controller.ox, m_controller.oy, new Rotation2d(m_controller.opsi));
            m_rawWheelOdometryPose = new Pose2d(m_plant.wheelOdomX, m_plant.wheelOdomY, new Rotation2d(m_plant.state[2]));

            // Guardar punto de trayectoria real para el resumen en forma de L
            double[] lastPt = m_rawActualPathPoints.get(m_rawActualPathPoints.size() - 1);
            if (Math.hypot(m_plant.state[0] - lastPt[0], m_plant.state[1] - lastPt[1]) >= 0.03) {
                m_rawActualPathPoints.add(new double[]{m_plant.state[0], m_plant.state[1], m_plant.state[2], m_simTrialTime});
            }

            // Registrar muestra de traza en vivo (50 Hz) para el archivo TXT
            double errFrameMm = Math.hypot(m_plant.state[0] - m_ghostPoseGlobal.getX(), m_plant.state[1] - m_ghostPoseGlobal.getY()) * 1000.0;
            double yawFrameDeg = Math.abs(Math.toDegrees(wrapAngle(m_plant.state[2] - m_ghostPoseGlobal.getRotation().getRadians())));
            String flagNow = !m_pendingEventFlag.equals("NORMAL") ? m_pendingEventFlag : (wasOnIceFrame ? "ON_BLACK_ICE" : "NORMAL");
            m_trialTraceSamples.add(new TraceSample(
                m_simTrialTime,
                m_ghostPoseGlobal.getX(), m_ghostPoseGlobal.getY(), m_ghostPoseGlobal.getRotation().getDegrees(),
                m_plant.state[0], m_plant.state[1], Math.toDegrees(m_plant.state[2]),
                m_controller.ox, m_controller.oy,
                m_plant.wheelOdomX, m_plant.wheelOdomY,
                m_plant.state[3], m_plant.state[4], m_plant.state[5],
                errFrameMm, yawFrameDeg,
                m_plant.lastLocalMu, m_controller.muHat,
                wasOnIceFrame, wasInColFrame, maxColForceFrame, m_plant.lastCombinedSlip,
                m_controller.lastNormalLoads[0], m_controller.lastNormalLoads[1],
                m_controller.lastNormalLoads[2], m_controller.lastNormalLoads[3],
                m_controller.lastStepLatencyUs, flagNow
            ));
            m_pendingEventFlag = "NORMAL";

            // Guardia anti-atasco: si un choque empuja al robot detrás de una esquina, replanifica ruta en L limpia
            EstadoTrayectoria fin = m_trayectoriaActiva.get(m_trayectoriaActiva.size() - 1);
            double distGoal = Math.hypot(m_plant.state[0] - fin.x, m_plant.state[1] - fin.y);
            double speed = Math.hypot(m_plant.state[3], m_plant.state[4]);
            if (errFrameMm > 1250.0 && speed < 0.14 && distGoal > 0.45) {
                m_stuckTimer += DT_FRAME;
                if (m_stuckTimer > 0.45 && (m_simTrialTime - m_lastReplanTime) > 1.2) {
                    m_lastReplanTime = m_simTrialTime;
                    m_stuckTimer = 0.0;
                    replanificarRecuperacionEnL(new Pose2d(fin.x, fin.y, new Rotation2d(fin.theta)));
                }
            } else {
                m_stuckTimer = 0.0;
            }

            // Verificar llegada limpia a la meta
            double yawErrGoal = Math.abs(wrapAngle(m_plant.state[2] - fin.theta));
            boolean settledAtGoal = (m_refTrajectoryTime >= m_totalTrajectoryDuration - 1e-4)
                && (distGoal <= 0.15) && (yawErrGoal <= 0.15) && (speed <= 0.18);
            boolean timeoutGraceReached = (m_simTrialTime >= m_totalTrajectoryDuration + 1.60);

            if (settledAtGoal || timeoutGraceReached) {
                m_rawActualPathPoints.add(new double[]{m_plant.state[0], m_plant.state[1], m_plant.state[2], m_simTrialTime});
                exportarResultadosSimulacionATxt(settledAtGoal, distGoal * 1000.0);

                if (m_modoPruebasAutomaticas) {
                    ejecutarSiguientePruebaAleatoria();
                } else {
                    m_siguiendoRuta = false;
                    double rmseMm = Math.sqrt(m_trialSqErrSum / Math.max(1, m_trialSamples));
                    m_ultimoEstadoPlanificador = String.format(Locale.US, "Simulación completada | RMSE: %.1f mm | Guardado en TXT", rmseMm);
                }
            }
        } else {
            // Modo teleoperado manual
            double vxField = Math.max(-1.0, Math.min(1.0, rawForward)) * 3.0;
            double vyField = Math.max(-1.0, Math.min(1.0, rawStrafe)) * 3.0;
            double omega = Math.max(-1.0, Math.min(1.0, rawTurn)) * 3.5;

            for (int sub = 0; sub < CONTROL_SUBCYCLES; sub++) {
                double[] zNoise = new double[24];
                for (int k = 0; k < 24; k++) zNoise[k] = m_rng.nextGaussian();
                double[] measVelW = m_plant.getMeasuredVelWorld(zNoise);
                double[] measAccB = m_plant.getMeasuredAccelBody(zNoise);
                double[] measWs   = m_plant.getMeasuredWheelSpeeds(zNoise);
                double[] measDel  = m_plant.getMeasuredDeltas(zNoise);

                double[] refCurr = {m_controller.ox, m_controller.oy, m_controller.opsi, vxField, vyField, omega};
                double[] refNext = {m_controller.ox + vxField * DT_CONTROL, m_controller.oy + vyField * DT_CONTROL,
                    wrapAngle(m_controller.opsi + omega * DT_CONTROL), vxField, vyField, omega};

                ControllerOutput out = m_controller.step(measVelW[2], measAccB, measWs, measDel, refCurr, refNext);
                m_plant.step(out.fCmds, out.dCmds, m_muNomActual, m_muIceActual, m_blackIceZones,
                    m_roughnessActual, 0.0, 0.0, 0.0, zNoise,
                    collisionsEnabled ? m_cajasObstaculosReales : Collections.emptyList());
            }
            m_trueWorldPose = new Pose2d(m_plant.state[0], m_plant.state[1], new Rotation2d(m_plant.state[2]));
            m_estimatedObserverPose = new Pose2d(m_controller.ox, m_controller.oy, new Rotation2d(m_controller.opsi));
            m_rawWheelOdometryPose = new Pose2d(m_plant.wheelOdomX, m_plant.wheelOdomY, new Rotation2d(m_plant.state[2]));
        }
    }

    private void registrarImpactoEnVivo(String type, double x, double y, double psi,
                                        double impulseNs, double peakForceN,
                                        double nx, double ny, double rcx, double rcy) {
        int id = m_trialImpacts.size() + 1;
        double rCrossN = rcx * ny - rcy * nx;
        double dv = impulseNs / m_mTrueActual;
        double dOmega = Math.abs(impulseNs * rCrossN) / m_izTrueActual;
        double impactHeading = Math.atan2(ny, nx);

        ImpactRecord rec = new ImpactRecord(
            id, type, m_simTrialTime, x, y, Math.toDegrees(psi),
            impulseNs, peakForceN, nx, ny, rcx, rcy, dv, dOmega
        );
        m_trialImpacts.add(rec);
        m_pendingEventFlag = "IMPACT_" + type;

        Pose2d marker = new Pose2d(x, y, new Rotation2d(impactHeading));
        m_impactMarkerPoses.add(marker);
        m_impactMarkersPublisher.set(m_impactMarkerPoses.toArray(new Pose2d[0]));
        m_lastImpactPosePublisher.set(marker);

        String info = String.format(
            Locale.US, "#%d [%s] t=%.2fs @ (%.2fm, %.2fm) | J=%.1f N*s | dv=%.2f m/s",
            id, type, m_simTrialTime, x, y, impulseNs, dv
        );
        m_lastImpactInfoPublisher.set(info);
        System.out.println("[IMPACT LIVE] " + info);
    }

    private void replanificarRecuperacionEnL(Pose2d goalPose) {
        double safetyMargin = m_safetyMarginEntry != null
            ? m_safetyMarginEntry.getDouble(MARGEN_SEGURIDAD_DEFAULT_M)
            : MARGEN_SEGURIDAD_DEFAULT_M;
        Pose2d currPose = new Pose2d(m_plant.state[0], m_plant.state[1], new Rotation2d(m_plant.state[2]));
        List<double[]> newL = new ArrayList<>();
        List<EstadoTrayectoria> newTraj = obtenerTrayectoriaCompletaM(Arrays.asList(currPose, goalPose), safetyMargin * 0.85, newL);
        if (newTraj != null && newTraj.size() >= 10) {
            m_trayectoriaActiva = newTraj;
            m_totalTrajectoryDuration = newTraj.get(newTraj.size() - 1).t;
            m_simTrialTime = 0.0;
            m_refTrajectoryTime = 0.0;
            m_pendingEventFlag = "L_PATH_REPLAN_RECOVERY";
            publicarTrayectoriaVisual(newTraj, newL);
            System.out.println("[RECOVERY REPLAN] Nueva ruta en L generada desde (" + String.format(Locale.US, "%.2f, %.2f", currPose.getX(), currPose.getY()) + ")");
        }
    }

    // =========================================================================
    // 7. EXPORTACIÓN RIGUROSA A TXT EN "C:\Users\HP ENVY\Downloads\Java advantage scope data"
    // =========================================================================
    private void exportarResultadosSimulacionATxt(boolean settledAtGoal, double finalGoalErrMm) {
        if (m_exportDir == null) return;

        // Finalizar tiempos de recuperación pendientes de impactos
        for (ImpactRecord imp : m_trialImpacts) {
            if (imp.recoveryTimeMs < 0.0) {
                imp.recoveryTimeMs = Math.min(450.0, Math.max(20.0, (m_simTrialTime - imp.timeSec) * 1000.0));
            }
        }

        double rmseMm = Math.sqrt(m_trialSqErrSum / Math.max(1, m_trialSamples));
        double slipRms = Math.sqrt(m_trialSqSlipSum / Math.max(1, m_trialSamples));
        double tprPct = 100.0 * m_tpIceSteps / Math.max(1, m_iceGtSteps);
        double fprPct = 100.0 * m_fpIceSteps / Math.max(1, m_nonIceGtSteps);
        double deadlineMissPct = 100.0 * m_deadlineMissCount / Math.max(1, m_stepLatenciesUs.size());

        double meanLatUs = 0.0, p99LatUs = 0.0;
        if (!m_stepLatenciesUs.isEmpty()) {
            Collections.sort(m_stepLatenciesUs);
            double sum = 0.0;
            for (double v : m_stepLatenciesUs) sum += v;
            meanLatUs = sum / m_stepLatenciesUs.size();
            int p99Idx = Math.min(m_stepLatenciesUs.size() - 1, (int) (0.99 * m_stepLatenciesUs.size()));
            p99LatUs = m_stepLatenciesUs.get(p99Idx);
        }

        double meanRecMs = 0.0;
        if (!m_trialImpacts.isEmpty()) {
            double sRec = 0.0;
            for (ImpactRecord imp : m_trialImpacts) sRec += imp.recoveryTimeMs;
            meanRecMs = sRec / m_trialImpacts.size();
        }

        // Simplificar el camino real tomado por el robot en sus vértices geométricos en forma de L (RDP 6.5 cm)
        List<double[]> actualLVertices = simplificarPolilineaRDP(m_rawActualPathPoints, 0.065);
        double plannedLengthM = calcularLongitudPolilinea(m_plannedLVertices);
        double actualLengthM = calcularLongitudPolilinea(actualLVertices);

        boolean hadWallContact = m_trialImpacts.stream().anyMatch(i -> i.type.contains("WALL"));
        String verdict;
        if (settledAtGoal && m_trialImpacts.isEmpty()) {
            verdict = "PASSED_CLEAN_NO_COLLISIONS";
        } else if (settledAtGoal && !hadWallContact) {
            verdict = "PASSED_WITH_DYNAMIC_IMPACTS (" + m_trialImpacts.size() + " impacts absorbed)";
        } else if (settledAtGoal) {
            verdict = "PASSED_WITH_OBSTACLE_RECOVERY (" + m_trialImpacts.size() + " contacts)";
        } else {
            verdict = "TIMEOUT_GRACE_REACHED (FinalErr=" + String.format(Locale.US, "%.1f", finalGoalErrMm) + "mm)";
        }
        m_trialVerdictPublisher.set(verdict);

        // 1. Escribir bloque riguroso en simulation_master_summary.txt
        StringBuilder sb = new StringBuilder();
        sb.append("====================================================================================================\n");
        sb.append(String.format(Locale.US, "TRIAL #%04d | Timestamp: %s | Platform: %s | Verdict: %s\n",
            m_trialCount, LocalDateTime.now().format(DateTimeFormatter.ISO_LOCAL_DATE_TIME), m_spec.name, verdict));
        sb.append("----------------------------------------------------------------------------------------------------\n");
        sb.append(String.format(Locale.US,
            "[PLANT_GROUND_TRUTH] m_true=%.3f kg (nom=%.1f) | Iz_true=%.3f kg*m^2 (nom=%.2f) | h_cg=%.3f m | Health=[%.2f, %.2f, %.2f, %.2f] | mu_nom=%.2f | mu_ice=%.2f\n",
            m_mTrueActual, m_spec.mNom, m_izTrueActual, m_spec.izNom, m_spec.hCg,
            m_motorHealthActual[0], m_motorHealthActual[1], m_motorHealthActual[2], m_motorHealthActual[3],
            m_muNomActual, m_muIceActual));

        sb.append("[BLACK_ICE_ZONES_M] ");
        if (m_blackIceZones.isEmpty()) {
            sb.append("None\n");
        } else {
            for (int i = 0; i < m_blackIceZones.size(); i++) {
                double[] z = m_blackIceZones.get(i);
                sb.append(String.format(Locale.US, "Zone%d=[x:%.2f..%.2f, y:%.2f..%.2f] ", i + 1, z[0], z[1], z[2], z[3]));
            }
            sb.append("\n");
        }

        // Camino resumido en forma de L (Planificado vs Ejecutado Real)
        sb.append(String.format(Locale.US, "[L_PATH_PLANNED_VERTICES] (Count=%d, Length=%.3f m): ", m_plannedLVertices.size(), plannedLengthM));
        for (int i = 0; i < m_plannedLVertices.size(); i++) {
            double[] v = m_plannedLVertices.get(i);
            sb.append(String.format(Locale.US, "V%d(%.3f, %.3f)", i, v[0], v[1]));
            if (i < m_plannedLVertices.size() - 1) sb.append(" -> ");
        }
        sb.append("\n");

        sb.append(String.format(Locale.US, "[L_PATH_ACTUAL_EXECUTED_SUMMARY] (Count=%d, Length=%.3f m, Ratio=%.3f): ",
            actualLVertices.size(), actualLengthM, actualLengthM / Math.max(0.01, plannedLengthM)));
        for (int i = 0; i < actualLVertices.size(); i++) {
            double[] v = actualLVertices.get(i);
            sb.append(String.format(Locale.US, "P%d(%.3f, %.3f @ t=%.2fs)", i, v[0], v[1], v[3]));
            if (i < actualLVertices.size() - 1) sb.append(" -> ");
        }
        sb.append("\n");

        // Registro de todos los choques en vivo
        sb.append(String.format(Locale.US, "[COLLISIONS_AND_CONTACTS_IN_LIVE_RUN] TotalEvents=%d\n", m_trialImpacts.size()));
        if (m_trialImpacts.isEmpty()) {
            sb.append("   -> CLEAN RUN: Zero collisions or wall contacts occurred.\n");
        } else {
            for (ImpactRecord imp : m_trialImpacts) {
                sb.append(String.format(Locale.US,
                    "   -> Impact #%d | Type=%-22s | t=%.3fs | Pos=(%.3f, %.3f)m | Yaw=%.1fdeg | J=%.2f N*s | F_peak=%.1f N | n=(%.2f,%.2f) | r_arm=(%.2f,%.2f)m | dV=%.2f m/s | dOmega=%.2f rad/s | PeakDefl=%.1f mm | RecTime=%.1f ms\n",
                    imp.id, imp.type, imp.timeSec, imp.x, imp.y, imp.yawDeg,
                    imp.impulseNs, imp.peakForceN, imp.nx, imp.ny, imp.rcx, imp.rcy,
                    imp.deltaVMps, imp.deltaOmegaRps, imp.peakPostDeflectionMm, imp.recoveryTimeMs));
            }
        }

        // Métricas cuantitativas globales del ensayo
        sb.append(String.format(Locale.US,
            "[PERFORMANCE_METRICS] RMSE=%.2f mm | MaxErr=%.2f mm | FinalGoalErr=%.2f mm | PeakYawErr=%.2f deg | PeakIceExcursion=%.2f mm | PeakColDeflection=%.2f mm | MeanColRecovery=%.1f ms | TireSlipRMS=%.4f\n",
            rmseMm, m_trialMaxErrMm, finalGoalErrMm, m_trialPeakYawErrDeg,
            m_trialPeakIceErrMm, m_trialPeakColErrMm, meanRecMs, slipRms));
        sb.append(String.format(Locale.US,
            "[REALTIME_AND_OBSERVER] MeanCtrlLatency=%.2f us | P99CtrlLatency=%.2f us | DeadlineMiss500Hz=%.4f%% | GripObserver_TPR=%.1f%% | GripObserver_FPR=%.1f%%\n\n",
            meanLatUs, p99LatUs, deadlineMissPct, tprPct, fprPct));

        try {
            Files.writeString(
                m_exportDir.resolve(MASTER_SUMMARY_FILENAME),
                sb.toString(),
                StandardCharsets.UTF_8,
                StandardOpenOption.CREATE,
                StandardOpenOption.APPEND
            );

            // 2. Escribir archivo tabular detallado de la traza en vivo del ensayo
            String traceFilename = String.format(Locale.US, "trial_%04d_live_trace.txt", m_trialCount);
            Path tracePath = m_exportDir.resolve(traceFilename);
            try (BufferedWriter writer = Files.newBufferedWriter(tracePath, StandardCharsets.UTF_8)) {
                writer.write("# Trial=" + m_trialCount + " | Platform=" + m_spec.name + " | Verdict=" + verdict + "\n");
                writer.write("time_s\tref_x_m\tref_y_m\tref_yaw_deg\ttrue_x_m\ttrue_y_m\ttrue_yaw_deg\test_x_m\test_y_m\todom_x_m\todom_y_m\tvx_world_mps\tvy_world_mps\tomega_rps\tpos_err_mm\tyaw_err_deg\tmu_true\tmu_hat\ton_ice\tin_collision\tcol_force_N\tslip_comb\tN_FL_N\tN_FR_N\tN_RL_N\tN_RR_N\tctrl_latency_us\tevent_flag\n");
                for (TraceSample s : m_trialTraceSamples) {
                    writer.write(String.format(Locale.US,
                        "%.3f\t%.4f\t%.4f\t%.2f\t%.4f\t%.4f\t%.2f\t%.4f\t%.4f\t%.4f\t%.4f\t%.3f\t%.3f\t%.3f\t%.2f\t%.2f\t%.3f\t%.3f\t%d\t%d\t%.1f\t%.4f\t%.1f\t%.1f\t%.1f\t%.1f\t%.2f\t%s\n",
                        s.t, s.refX, s.refY, s.refYawDeg, s.trueX, s.trueY, s.trueYawDeg,
                        s.estX, s.estY, s.odomX, s.odomY, s.vxW, s.vyW, s.omega,
                        s.posErrMm, s.yawErrDeg, s.muTrue, s.muHat,
                        s.onIce ? 1 : 0, s.inCollision ? 1 : 0,
                        s.colForceN, s.slipComb, s.n0, s.n1, s.n2, s.n3, s.latencyUs, s.eventFlag
                    ));
                }
            }
            System.out.println(String.format(Locale.US,
                "[TRIAL #%04d SAVED] RMSE=%.1f mm | Impacts=%d | Lat=%.2f us -> %s",
                m_trialCount, rmseMm, m_trialImpacts.size(), meanLatUs, traceFilename));
        } catch (IOException e) {
            System.err.println("[EXPORT ERROR] Error escribiendo TXT: " + e.getMessage());
        }
    }

    private double[] muestrearReferencia(List<EstadoTrayectoria> traj, double tQuery) {
        int n = traj.size();
        if (tQuery <= 0.0) {
            EstadoTrayectoria p0 = traj.get(0);
            return new double[]{p0.x, p0.y, p0.theta, 0.0, 0.0, 0.0};
        }
        EstadoTrayectoria last = traj.get(n - 1);
        if (tQuery >= last.t) {
            return new double[]{last.x, last.y, last.theta, 0.0, 0.0, 0.0};
        }

        int lo = 0, hi = n - 2;
        while (lo < hi) {
            int mid = (lo + hi + 1) >>> 1;
            if (traj.get(mid).t <= tQuery) lo = mid;
            else hi = mid - 1;
        }
        EstadoTrayectoria p0 = traj.get(lo);
        EstadoTrayectoria p1 = traj.get(lo + 1);
        double dtSeg = Math.max(1e-6, p1.t - p0.t);
        double alpha = Math.max(0.0, Math.min(1.0, (tQuery - p0.t) / dtSeg));

        double rx = p0.x + alpha * (p1.x - p0.x);
        double ry = p0.y + alpha * (p1.y - p0.y);
        double dth = wrapAngle(p1.theta - p0.theta);
        double rpsi = wrapAngle(p0.theta + alpha * dth);
        double rvx = (p1.x - p0.x) / dtSeg;
        double rvy = (p1.y - p0.y) / dtSeg;
        double rr = dth / dtSeg;
        return new double[]{rx, ry, rpsi, rvx, rvy, rr};
    }

    // =========================================================================
    // 8. CONTROLADOR ANALÍTICO O(1) EXACTO DE PYTHON (CONGELADO Y VALIDADO)
    // =========================================================================
    public static class ControllerOutput {
        public final double[] fCmds;
        public final double[] dCmds;
        public final double latencyUs;
        public ControllerOutput(double[] fCmds, double[] dCmds, double latencyUs) {
            this.fCmds = fCmds;
            this.dCmds = dCmds;
            this.latencyUs = latencyUs;
        }
    }

    public static class ProposedO1SwerveController {
        private final RobotSpec spec;
        private final double dt;
        private final boolean useLoadAndObs;
        private final double kp, kv, kPsi, kR;
        private final double inv4R2, rChar, betaInt, betaMuPost, alphaKinFilt, cgAspect, massScale;

        // Estado del observador complementario SE(2)
        public double ox = 0.0, oy = 0.0, opsi = 0.0, ovx = 0.0, ovy = 0.0, or_ = 0.0;

        // Estado del observador 2D de agarre e inmunidad inercial
        public double muHat = 0.84;
        public boolean inCollision = false;
        public boolean inIceTrip = false;
        public double lastStepLatencyUs = 0.0;
        public final double[] lastNormalLoads = new double[4];
        public final double[] lastFCmds = new double[4];
        public final double[] lastDCmds = new double[4];

        private double ivImuX = 0.0, ivImuY = 0.0, ivKinX = 0.0, ivKinY = 0.0;
        private double ivImuTrip0X = 0.0, ivImuTrip0Y = 0.0, ivKinTrip0X = 0.0, ivKinTrip0Y = 0.0;
        private double postTripImuE = 0.0, postTripKinE = 0.0;

        private double iw0 = 0.25, iw1 = 0.25, iw2 = 0.25, iw3 = 0.25;
        private double jColX = 0.0, jColY = 0.0, jColR = 0.0, rhoRotCol = 0.0;
        private double vbxKinPrev = 0.0, vbyKinPrev = 0.0, angVelPrev = 0.0;
        private double axKinFilt = 0.0, ayKinFilt = 0.0;
        private double axKinPrev = 0.0, ayKinPrev = 0.0;
        private final double[] dsPrev = new double[4];
        private double daExtPrevX = 0.0, daExtPrevY = 0.0;
        private int colSteps = 0, postColDampSteps = 0, iceStepsDur = 0, sigLossHold = 0;
        private double axBPrev = 0.0, ayBPrev = 0.0, axBFilt = 0.0, ayBFilt = 0.0;
        private double fbXLagged = 0.0, fbYLagged = 0.0;
        private boolean firstStep = true;

        public ProposedO1SwerveController(RobotSpec spec, double dt, boolean useLoadAndObs) {
            this.spec = spec;
            this.dt = dt;
            this.useLoadAndObs = useLoadAndObs;

            this.inv4R2 = 1.0 / (4.0 * spec.rSq);
            this.rChar = Math.sqrt(spec.rSq);
            this.betaInt = Math.exp(-dt / 0.020);
            this.betaMuPost = Math.exp(-dt / 0.028);
            this.alphaKinFilt = dt / (0.012 + dt);
            this.cgAspect = spec.hCg / Math.max(spec.lx, 1e-3);
            this.massScale = Math.sqrt(spec.mNom / 72.0);

            // Síntesis exacta DARE invariante a escala (derive_scale_consistent_gains)
            double dragTerm = 1.0 - spec.dragLin * dt / spec.mNom;
            double[][] A = {
                {1.0, 0.0, dt,  0.0},
                {0.0, 1.0, 0.0, dt },
                {0.0, 0.0, dragTerm, 0.0},
                {0.0, 0.0, 0.0, dragTerm}
            };
            double bPos = 0.5 * dt * dt / spec.mNom;
            double bVel = dt / spec.mNom;
            double[][] B = {
                {bPos, 0.0},
                {0.0, bPos},
                {bVel, 0.0},
                {0.0, bVel}
            };
            double mRatio = spec.mNom / 72.0;
            double[][] Q = {
                {4800.0 * mRatio, 0, 0, 0},
                {0, 4800.0 * mRatio, 0, 0},
                {0, 0, 280.0 * mRatio, 0},
                {0, 0, 0, 280.0 * mRatio}
            };
            double[][] R = {
                {0.002 / mRatio, 0.0},
                {0.0, 0.002 / mRatio}
            };

            double[][] Qf = solveDareExact(A, B, Q, R);
            double[][] K = computeRiccatiGain2x4(A, B, Qf, R);
            this.kp = K[0][0];
            this.kv = K[0][2];

            double izRatio = spec.izNom / 5.20;
            double dampBoost = Math.max(1.0, Math.pow(5.20 / spec.izNom, 0.25));
            this.kPsi = 220.0 * izRatio;
            this.kR = 34.0 * izRatio * dampBoost;

            reset(0.84, 0.0, 0.0, 0.0);
        }

        public void reset(double muInit, double x0, double y0, double psi0) {
            this.ox = x0;
            this.oy = y0;
            this.opsi = psi0;
            this.ovx = 0.0;
            this.ovy = 0.0;
            this.or_ = 0.0;

            this.muHat = muInit;
            this.inCollision = false;
            this.inIceTrip = false;
            this.ivImuX = this.ivImuY = this.ivKinX = this.ivKinY = 0.0;
            this.ivImuTrip0X = this.ivImuTrip0Y = this.ivKinTrip0X = this.ivKinTrip0Y = 0.0;
            this.postTripImuE = this.postTripKinE = 0.0;
            this.iw0 = this.iw1 = this.iw2 = this.iw3 = 0.25;
            this.jColX = this.jColY = this.jColR = this.rhoRotCol = 0.0;
            this.vbxKinPrev = this.vbyKinPrev = this.angVelPrev = 0.0;
            this.axKinFilt = this.ayKinFilt = this.axKinPrev = this.ayKinPrev = 0.0;
            this.daExtPrevX = this.daExtPrevY = 0.0;
            this.colSteps = this.postColDampSteps = this.iceStepsDur = this.sigLossHold = 0;
            this.axBPrev = this.ayBPrev = this.axBFilt = this.ayBFilt = 0.0;
            this.fbXLagged = this.fbYLagged = 0.0;
            this.firstStep = true;
            Arrays.fill(dsPrev, 0.0);
            Arrays.fill(lastFCmds, 0.0);
            Arrays.fill(lastDCmds, 0.0);
            Arrays.fill(lastNormalLoads, 0.25 * spec.mNom * G);
        }

        public ControllerOutput step(double rGyro, double[] accelB,
                                     double[] wheelSpeeds, double[] deltasMeas,
                                     double[] refCurr, double[] refNext) {
            long tStartNs = System.nanoTime();

            double axBM = accelB[0], ayBM = accelB[1];
            if (firstStep) {
                System.arraycopy(deltasMeas, 0, dsPrev, 0, 4);
            }

            // 1. Ruptura de tasa de giro de dirección y curl cinemático
            double maxSteerBreak = 0.0;
            for (int i = 0; i < 4; i++) {
                double dDelta = Math.abs(wrapAngle(deltasMeas[i] - dsPrev[i]));
                double ratioRate = (dDelta / dt) / Math.max(spec.maxSteerRate, 1e-6);
                maxSteerBreak = Math.max(maxSteerBreak, Math.max(0.0, ratioRate - 0.85));
            }
            System.arraycopy(deltasMeas, 0, dsPrev, 0, 4);

            double[] vbxList = new double[4];
            double[] vbyList = new double[4];
            double vbxSum = 0.0, vbySum = 0.0, rKinNum = 0.0;
            for (int i = 0; i < 4; i++) {
                double cx = spec.corners[i][0], cy = spec.corners[i][1];
                double cd = Math.cos(deltasMeas[i]), sd = Math.sin(deltasMeas[i]);
                double vbxI = wheelSpeeds[i] * cd + rGyro * cy;
                double vbyI = wheelSpeeds[i] * sd - rGyro * cx;
                vbxList[i] = vbxI;
                vbyList[i] = vbyI;
                vbxSum += vbxI;
                vbySum += vbyI;
                rKinNum += cx * (wheelSpeeds[i] * sd) - cy * (wheelSpeeds[i] * cd);
            }

            double rKin = rKinNum * inv4R2;
            double curlBreak = Math.abs(rKin - rGyro);
            double vbxMean = 0.25 * vbxSum, vbyMean = 0.25 * vbySum;

            // Fusión cinemática con rechazo exponencial de ruedas que patinan
            double vbxK = 0.0, vbyK = 0.0, wKSum = 0.0;
            for (int i = 0; i < 4; i++) {
                double devI = Math.hypot(vbxList[i] - vbxMean, vbyList[i] - vbyMean);
                double wi = useLoadAndObs ? Math.exp(-3.5 * Math.max(0.0, devI - 0.14)) : 1.0;
                vbxK += wi * vbxList[i];
                vbyK += wi * vbyList[i];
                wKSum += wi;
            }
            vbxK /= Math.max(wKSum, 1e-6);
            vbyK /= Math.max(wKSum, 1e-6);

            double angVel = Math.atan2(vbyK, vbxK);
            if (firstStep) {
                vbxKinPrev = vbxK;
                vbyKinPrev = vbyK;
                angVelPrev = angVel;
            }
            double dAngVel = Math.abs(wrapAngle(angVel - angVelPrev));
            angVelPrev = angVel;

            double axKinRaw = (vbxK - vbxKinPrev) / dt - rGyro * vbyK;
            double ayKinRaw = (vbyK - vbyKinPrev) / dt + rGyro * vbxK;
            axKinFilt += alphaKinFilt * (axKinRaw - axKinFilt);
            ayKinFilt += alphaKinFilt * (ayKinRaw - ayKinFilt);
            double axKin = axKinFilt, ayKin = ayKinFilt;

            if (firstStep) {
                axKinPrev = axKin;
                ayKinPrev = ayKin;
                firstStep = false;
            }
            double d2KinMag = Math.hypot(axKin - axKinPrev, ayKin - ayKinPrev);
            axKinPrev = axKin;
            ayKinPrev = ayKin;
            vbxKinPrev = vbxK;
            vbyKinPrev = vbyK;

            double aMag = Math.hypot(axBM, ayBM);
            double daMag = Math.hypot(axBM - axBPrev, ayBM - ayBPrev);
            axBPrev = axBM;
            ayBPrev = ayBM;

            double fCmdMag = Math.hypot(fbXLagged, fbYLagged);
            double scCap = Math.min(1.0, (0.84 * spec.mNom * G) / Math.max(fCmdMag, 1e-6));
            double axExp = (fbXLagged * scCap - spec.dragLin * vbxK) / spec.mNom;
            double ayExp = (fbYLagged * scCap - spec.dragLin * vbyK) / spec.mNom;
            double daExtX = axBM - axExp;
            double daExtY = ayBM - ayExp;
            double daExtMag = Math.hypot(daExtX, daExtY);

            // 2. Observador 2D de agarre e inmunidad inercial a choques
            if (useLoadAndObs) {
                boolean diffBreakpoint = (maxSteerBreak > 0.15) || (dAngVel > 0.065) || (curlBreak > 0.85);
                boolean derivFlip = ((daExtX * daExtPrevX + daExtY * daExtPrevY) < -1.5)
                    || (Math.abs(daExtMag - Math.hypot(daExtPrevX, daExtPrevY)) > 2.2);
                daExtPrevX = daExtX;
                daExtPrevY = daExtY;
                boolean synchronizedBoundary = diffBreakpoint && derivFlip;

                double accelGap = Math.hypot(axKin - axBM, ayKin - ayBM);
                double centripetalGuard = Math.max(32.0, (20.0 + 5.0 * Math.abs(rGyro) * Math.hypot(vbxK, vbyK)) * massScale);
                boolean signOpposition = ((axKin * axBM + ayKin * ayBM) < -3.5) && (accelGap > 5.0);
                boolean bDiff = (d2KinMag > centripetalGuard) || signOpposition || (curlBreak > 1.05);
                boolean binarySignalLoss = bDiff && (!synchronizedBoundary);

                if (binarySignalLoss) sigLossHold = 6;
                else if (sigLossHold > 0) sigLossHold--;

                if (!inCollision) {
                    if ((aMag > 0.96 * G || daMag > 1.25) && daExtMag > 0.75 * G) {
                        inCollision = true;
                        colSteps = 1;
                        jColX = daExtX * dt;
                        jColY = daExtY * dt;
                        jColR = rGyro - or_;
                        ivImuX = ivImuY = ivKinX = ivKinY = 0.0;
                        inIceTrip = false;
                        muHat = 0.84;
                    }
                } else {
                    colSteps++;
                    jColX += daExtX * dt;
                    jColY += daExtY * dt;
                    jColR += rGyro - or_;
                    if ((daExtMag < 0.45 * G && colSteps >= 6) || colSteps >= 35) {
                        inCollision = false;
                        postColDampSteps = Math.min(55, Math.max(20, colSteps * 3));
                        double pLin = spec.mNom * rChar * Math.hypot(jColX, jColY);
                        double pRot = spec.izNom * Math.abs(jColR);
                        rhoRotCol = pRot / Math.max(pRot + pLin, 1e-6);
                    }
                }

                if (postColDampSteps > 0) postColDampSteps--;
                else rhoRotCol = 0.0;

                if (synchronizedBoundary && !inCollision) {
                    ivImuX = ivImuY = ivKinX = ivKinY = 0.0;
                    if (inIceTrip && iceStepsDur >= 4) {
                        inIceTrip = false;
                        muHat = 0.84;
                    }
                }

                if (!inCollision && postColDampSteps == 0) {
                    ivImuX = betaInt * ivImuX + axBM * dt;
                    ivImuY = betaInt * ivImuY + ayBM * dt;
                    ivKinX = betaInt * ivKinX + axKin * dt;
                    ivKinY = betaInt * ivKinY + ayKin * dt;

                    double jxKin = ivKinX - ivImuX, jyKin = ivKinY - ivImuY;
                    double kinJump = Math.hypot(jxKin, jyKin);
                    double dxKinD = axKin - axBM, dyKinD = ayKin - ayBM;
                    double kinRateDot = jxKin * dxKinD + jyKin * dyKinD;

                    double jumpTh = 0.095 * Math.sqrt(massScale);
                    double gapTh = 3.0 * massScale;

                    if (!inIceTrip) {
                        if ((kinJump > jumpTh && kinRateDot > 0.022 && accelGap > gapTh)
                            || (sigLossHold >= 5 && kinJump > 0.82 * jumpTh && accelGap > 0.85 * gapTh)) {
                            inIceTrip = true;
                            iceStepsDur = 1;
                            ivImuTrip0X = ivImuX;
                            ivImuTrip0Y = ivImuY;
                            ivKinTrip0X = ivKinX;
                            ivKinTrip0Y = ivKinY;
                            postTripImuE = 0.0;
                            postTripKinE = 0.0;
                            muHat = 0.84;
                        }
                    } else {
                        iceStepsDur++;
                        double dImuMag = Math.hypot(ivImuX - ivImuTrip0X, ivImuY - ivImuTrip0Y);
                        double dKinMag = Math.hypot(ivKinX - ivKinTrip0X, ivKinY - ivKinTrip0Y);
                        postTripImuE += dImuMag * dt;
                        postTripKinE += Math.max(dKinMag, dImuMag) * dt;
                        if (postTripKinE > 1e-5) {
                            double ratioRaw = 0.84 * (postTripImuE / postTripKinE);
                            double ratioClamped = Math.max(0.14, Math.min(0.84, ratioRaw));
                            muHat = betaMuPost * muHat + (1.0 - betaMuPost) * ratioClamped;
                        }
                        if (((kinJump < 0.60 * jumpTh || kinRateDot <= 0.0) && sigLossHold == 0 && iceStepsDur >= 4)
                            || iceStepsDur >= 250) {
                            inIceTrip = false;
                            muHat = 0.84;
                        }
                    }
                } else {
                    inIceTrip = false;
                    muHat = 0.84;
                }
            }

            // 3. Observador complementario de estado (IMU + Cinemática con peso adaptativo)
            double rMid = 0.5 * (or_ + rGyro);
            double opsiNext = wrapAngle(opsi + rMid * dt);
            double opsiMid = wrapAngle(opsi + 0.5 * rMid * dt);
            double cMid = Math.cos(opsiMid), sMid = Math.sin(opsiMid);
            double vxPred = ovx + (cMid * axBM - sMid * ayBM) * dt;
            double vyPred = ovy + (sMid * axBM + cMid * ayBM) * dt;

            double cN = Math.cos(opsiNext), sN = Math.sin(opsiNext);
            double vxKinW = cN * vbxK - sN * vbyK;
            double vyKinW = sN * vbxK + cN * vbyK;
            double alphaKin = inCollision ? 0.05 : ((inIceTrip || sigLossHold > 0) ? 0.10 : 0.36);

            double ovxNext = (1.0 - alphaKin) * vxPred + alphaKin * vxKinW;
            double ovyNext = (1.0 - alphaKin) * vyPred + alphaKin * vyKinW;
            ox += 0.5 * (ovx + ovxNext) * dt;
            oy += 0.5 * (ovy + ovyNext) * dt;
            opsi = opsiNext;
            ovx = ovxNext;
            ovy = ovyNext;
            or_ = rGyro;

            double cPsi = Math.cos(opsi), sPsi = Math.sin(opsi);

            // 4. Transferencia dinámica de carga normal N_i(ax, ay)
            double n0, n1, n2, n3, w0, w1, w2, w3;
            if (!useLoadAndObs) {
                n0 = n1 = n2 = n3 = 0.25 * spec.mNom * G;
                w0 = w1 = w2 = w3 = 0.25;
            } else {
                axBFilt = 0.92 * axBFilt + 0.08 * axBM;
                ayBFilt = 0.92 * ayBFilt + 0.08 * ayBM;
                double baseN = 0.25 * spec.mNom * G;
                double dxN = (spec.mNom * axBFilt * spec.hCg) / (4.0 * spec.lx);
                double dyN = (spec.mNom * ayBFilt * spec.hCg) / (4.0 * spec.ly);
                double minN = 0.08 * baseN;
                n0 = Math.max(minN, baseN - dxN - dyN);
                n1 = Math.max(minN, baseN - dxN + dyN);
                n2 = Math.max(minN, baseN + dxN - dyN);
                n3 = Math.max(minN, baseN + dxN + dyN);
                double invS = 1.0 / (n0 + n1 + n2 + n3);
                w0 = n0 * invS;
                w1 = n1 * invS;
                w2 = n2 * invS;
                w3 = n3 * invS;
                iw0 = betaInt * iw0 + (1.0 - betaInt) * w0;
                iw1 = betaInt * iw1 + (1.0 - betaInt) * w1;
                iw2 = betaInt * iw2 + (1.0 - betaInt) * w2;
                iw3 = betaInt * iw3 + (1.0 - betaInt) * w3;
            }
            lastNormalLoads[0] = n0;
            lastNormalLoads[1] = n1;
            lastNormalLoads[2] = n2;
            lastNormalLoads[3] = n3;

            // 5. Ley de control DARE + Feedforward + Amortiguamiento anisotrópico post-colisión
            double rx = refCurr[0], ry = refCurr[1], rpsi = refCurr[2];
            double rvx = refCurr[3], rvy = refCurr[4], rr = refCurr[5];
            double nrvx = refNext[3], nrvy = refNext[4], nrr = refNext[5];

            double axFf = (nrvx - rvx) / dt;
            double ayFf = (nrvy - rvy) / dt;
            double alphaFf = (nrr - rr) / dt;

            double dampLin = (!inCollision && postColDampSteps > 0) ? (1.0 + 0.35 * (1.0 - rhoRotCol)) : 1.0;
            double dampRot = (!inCollision && postColDampSteps > 0) ? (1.0 + 0.65 * rhoRotCol) : 1.0;

            double fxWDes = spec.mNom * axFf + spec.dragLin * rvx - kp * (ox - rx) - (kv * dampLin) * (ovx - rvx);
            double fyWDes = spec.mNom * ayFf + spec.dragLin * rvy - kp * (oy - ry) - (kv * dampLin) * (ovy - rvy);
            double fbXDes = cPsi * fxWDes + sPsi * fyWDes;
            double fbYDes = -sPsi * fxWDes + cPsi * fyWDes;

            double fTransMag = Math.hypot(fbXDes, fbYDes);
            double fTransCap = 0.96 * 0.84 * spec.mNom * G;
            if (fTransMag > fTransCap && fTransMag > 1e-9) {
                double scT = fTransCap / fTransMag;
                fbXDes *= scT;
                fbYDes *= scT;
            }

            double tzDes = spec.izNom * alphaFf + spec.dragYaw * rr - kPsi * wrapAngle(opsi - rpsi) - (kR * dampRot) * (or_ - rr);

            // 6. Asignación analítica O(1) con círculo de fricción dinámico
            double gTrans = inIceTrip ? (1.0 - Math.min(0.24, 0.28 * cgAspect) * (1.0 - muHat / 0.84)) : 1.0;
            double pDiagA = 0.5 * (iw0 + iw3);
            double pDiagB = 0.5 * (iw1 + iw2);
            double pSum = Math.max(pDiagA + pDiagB, 1e-6);
            double wa = 0.5 + 0.25 * (pDiagA / pSum - 0.5);
            double wb = 1.0 - wa;
            double[] rotWeights = {wa, wb, wb, wa};
            double[] ws = {w0, w1, w2, w3};
            double[] ns = {n0, n1, n2, n3};

            double fbXAct = 0.0, fbYAct = 0.0;
            for (int i = 0; i < 4; i++) {
                double cx = spec.corners[i][0], cy = spec.corners[i][1];
                double ftX = gTrans * ws[i] * fbXDes;
                double ftY = gTrans * ws[i] * fbYDes;
                double rwI = 2.0 * rotWeights[i];
                double frX = -cy * tzDes * inv4R2 * rwI;
                double frY = cx * tzDes * inv4R2 * rwI;

                double fCapI = 0.84 * ns[i];
                double frNorm = Math.hypot(frX, frY);
                double fxi, fyi;

                if (frNorm >= fCapI) {
                    if (frNorm > 1e-9) {
                        fxi = frX * fCapI / frNorm;
                        fyi = frY * fCapI / frNorm;
                    } else {
                        fxi = 0.0;
                        fyi = 0.0;
                    }
                } else {
                    fxi = ftX + frX;
                    fyi = ftY + frY;
                    if (Math.hypot(fxi, fyi) > fCapI) {
                        double aQ = ftX * ftX + ftY * ftY;
                        if (aQ > 1e-12) {
                            double bQ = ftX * frX + ftY * frY;
                            double cQ = frNorm * frNorm - fCapI * fCapI;
                            double alphaQ = Math.max(0.0, Math.min(1.0, (-bQ + Math.sqrt(Math.max(0.0, bQ * bQ - aQ * cQ))) / aQ));
                            fxi = alphaQ * ftX + frX;
                            fyi = alphaQ * ftY + frY;
                        } else {
                            fxi = frX;
                            fyi = frY;
                        }
                    }
                }

                double fBemfI = Math.max(20.0, spec.fStall * (1.0 - Math.abs(wheelSpeeds[i]) / spec.vFree));
                double fMag = Math.hypot(fxi, fyi);
                if (fMag > fBemfI && fMag > 1e-9) {
                    double sc = fBemfI / fMag;
                    fxi *= sc;
                    fyi *= sc;
                    fMag = fBemfI;
                }

                if (fMag > 0.5) {
                    double ang = Math.atan2(fyi, fxi);
                    double errAng = wrapAngle(ang - deltasMeas[i]);
                    if (Math.abs(errAng) > 0.62 * Math.PI) {
                        ang = wrapAngle(ang + Math.PI);
                        errAng = wrapAngle(ang - deltasMeas[i]);
                        fMag = -fMag;
                    }
                    double alignGain = Math.pow(Math.cos(errAng), 2);
                    lastFCmds[i] = fMag * alignGain;
                    lastDCmds[i] = ang;
                    fbXAct += fxi * alignGain;
                    fbYAct += fyi * alignGain;
                } else {
                    lastFCmds[i] = 0.0;
                    lastDCmds[i] = deltasMeas[i];
                }
            }

            double latencyUs = (System.nanoTime() - tStartNs) / 1000.0;
            this.lastStepLatencyUs = latencyUs;

            double alphaMot = dt / (spec.tauDrive + dt);
            fbXLagged += alphaMot * (fbXAct - fbXLagged);
            fbYLagged += alphaMot * (fbYAct - fbYLagged);

            return new ControllerOutput(lastFCmds.clone(), lastDCmds.clone(), latencyUs);
        }

        private static double[][] solveDareExact(double[][] A, double[][] B, double[][] Q, double[][] R) {
            double[][] P = new double[4][4];
            for (int i = 0; i < 4; i++) System.arraycopy(Q[i], 0, P[i], 0, 4);

            for (int iter = 0; iter < 3000; iter++) {
                double[][] K = computeRiccatiGain2x4(A, B, P, R);
                double[][] ABK = new double[4][4];
                for (int i = 0; i < 4; i++) {
                    for (int j = 0; j < 4; j++) {
                        ABK[i][j] = A[i][j] - (B[i][0] * K[0][j] + B[i][1] * K[1][j]);
                    }
                }
                double[][] Pn = new double[4][4];
                double maxDiff = 0.0;
                for (int i = 0; i < 4; i++) {
                    for (int j = 0; j < 4; j++) {
                        double acc = 0.0;
                        for (int r = 0; r < 4; r++) {
                            for (int c = 0; c < 4; c++) {
                                acc += A[r][i] * P[r][c] * ABK[c][j];
                            }
                        }
                        Pn[i][j] = Q[i][j] + acc;
                    }
                }
                for (int i = 0; i < 4; i++) {
                    for (int j = 0; j < 4; j++) {
                        double sym = 0.5 * (Pn[i][j] + Pn[j][i]);
                        maxDiff = Math.max(maxDiff, Math.abs(sym - P[i][j]));
                        P[i][j] = sym;
                    }
                }
                if (maxDiff < 1e-14) break;
            }
            return P;
        }

        private static double[][] computeRiccatiGain2x4(double[][] A, double[][] B, double[][] P, double[][] R) {
            double[][] PB = new double[4][2];
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 2; j++) {
                    for (int k = 0; k < 4; k++) PB[i][j] += P[i][k] * B[k][j];
                }
            }
            double[][] S = new double[2][2];
            for (int i = 0; i < 2; i++) {
                for (int j = 0; j < 2; j++) {
                    S[i][j] = R[i][j];
                    for (int k = 0; k < 4; k++) S[i][j] += B[k][i] * PB[k][j];
                }
            }
            double det = S[0][0] * S[1][1] - S[0][1] * S[1][0];
            double[][] Sinv = {
                { S[1][1] / det, -S[0][1] / det},
                {-S[1][0] / det,  S[0][0] / det}
            };
            double[][] BTPA = new double[2][4];
            for (int i = 0; i < 2; i++) {
                for (int j = 0; j < 4; j++) {
                    for (int k = 0; k < 4; k++) BTPA[i][j] += PB[k][i] * A[k][j];
                }
            }
            double[][] K = new double[2][4];
            for (int i = 0; i < 2; i++) {
                for (int j = 0; j < 4; j++) {
                    K[i][j] = Sinv[i][0] * BTPA[0][j] + Sinv[i][1] * BTPA[1][j];
                }
            }
            return K;
        }
    }

    // =========================================================================
    // 9. PLANTA FÍSICA SE(2) A 5 KHZ + SAT MULTI-VÉRTICE SIN TELETRANSPORTACIÓN
    // =========================================================================
    public static class MultiClassSwerveSE2Plant {
        private final RobotSpec spec;
        private final double m;
        private final double iz;
        private final double[] motorHealth;
        private final double dtSub;
        private final int nSub;
        private final double bWheel;

        public final double[] state = new double[6]; // [px, py, psi, vx_w, vy_w, r]
        public final double[] omegaW = new double[4];
        public final double[] deltas = new double[4];
        public final double[] fMot = new double[4];
        public double axBFilt = 0.0, ayBFilt = 0.0;
        public double axBLast = 0.0, ayBLast = 0.0;
        public double wheelOdomX = 0.0, wheelOdomY = 0.0;

        public boolean anyWheelOnIce = false;
        public double lastLocalMu = 0.84;
        public double lastCombinedSlip = 0.0;

        public boolean wallImpactOccurredThisStep = false;
        public double lastWallImpulseNs = 0.0;
        public double lastWallNx = 0.0, lastWallNy = 0.0, lastWallCrx = 0.0, lastWallCry = 0.0;

        public MultiClassSwerveSE2Plant(RobotSpec spec, double mTrue, double izTrue,
                                        double[] motorHealth, double dtControl, int nSub) {
            this.spec = spec;
            this.m = mTrue;
            this.iz = izTrue;
            this.motorHealth = motorHealth.clone();
            this.nSub = nSub;
            this.dtSub = dtControl / nSub;
            this.bWheel = 0.015 * (spec.rW / 0.0508);
        }

        public void reset(double[] x0, double initialWheelWorldAngle) {
            System.arraycopy(x0, 0, state, 0, 6);
            this.wheelOdomX = x0[0];
            this.wheelOdomY = x0[1];
            this.axBFilt = this.ayBFilt = this.axBLast = this.ayBLast = 0.0;
            this.anyWheelOnIce = false;
            this.lastLocalMu = 0.84;
            this.lastCombinedSlip = 0.0;
            Arrays.fill(omegaW, 0.0);
            Arrays.fill(fMot, 0.0);
            double initLocalDelta = wrapAngle(initialWheelWorldAngle - x0[2]);
            Arrays.fill(deltas, initLocalDelta);
        }

        public void step(double[] fCmd, double[] dCmd, double muNom, double muIce,
                         List<double[]> iceZones, double roughness,
                         double fxExtW, double fyExtW, double tzExt,
                         double[] zNoise, List<double[]> obstacleBoxes) {
            double alphaAcc = dtSub / (0.012 + dtSub);
            double bP = 12.0, cP = 1.55, eP = 0.25;
            double baseN = 0.25 * m * G;

            double px = state[0], py = state[1], psi = state[2];
            double vxW = state[3], vyW = state[4], r = state[5];

            anyWheelOnIce = false;
            wallImpactOccurredThisStep = false;
            lastWallImpulseNs = 0.0;
            double slipAccum = 0.0;
            double muMinSub = muNom;

            for (int sub = 0; sub < nSub; sub++) {
                double cPsi = Math.cos(psi), sPsi = Math.sin(psi);
                double vxB = cPsi * vxW + sPsi * vyW;
                double vyB = -sPsi * vxW + cPsi * vyW;

                // 1. Actuadores de dirección y tracción con back-EMF y salud asimétrica
                for (int i = 0; i < 4; i++) {
                    double hI = motorHealth[i];
                    double tauSI = spec.tauSteer / Math.max(0.5, hI);
                    double tauDI = spec.tauDrive / Math.max(0.5, hI);
                    double alphaS = dtSub / (tauSI + dtSub);
                    double alphaD = dtSub / (tauDI + dtSub);
                    double maxSr = spec.maxSteerRate * hI;

                    double errD = wrapAngle(dCmd[i] - deltas[i]);
                    double dRate = Math.max(-maxSr, Math.min(maxSr, (alphaS * errD) / dtSub));
                    deltas[i] = wrapAngle(deltas[i] + dRate * dtSub);

                    double vTread = omegaW[i] * spec.rW;
                    double fStallI = spec.fStall * hI;
                    double vFreeI = spec.vFree * (0.85 + 0.15 * hI);
                    double fBemf = Math.max(0.0, fStallI * (1.0 - Math.abs(vTread) / vFreeI));
                    double fTarget = Math.max(-fBemf, Math.min(fBemf, fCmd[i]));
                    fMot[i] += alphaD * (fTarget - fMot[i]);
                }

                // 2. Transferencia dinámica de carga normal N_i(ax, ay)
                double dxN = (m * axBFilt * spec.hCg) / (4.0 * spec.lx);
                double dyN = (m * ayBFilt * spec.hCg) / (4.0 * spec.ly);
                double minN = 0.08 * baseN;
                double[] nLoads = {
                    Math.max(minN, baseN - dxN - dyN),
                    Math.max(minN, baseN - dxN + dyN),
                    Math.max(minN, baseN + dxN - dyN),
                    Math.max(minN, baseN + dxN + dyN)
                };

                double fxBTotal = 0.0, fyBTotal = 0.0, tzTotal = 0.0;
                double odomVxB = 0.0, odomVyB = 0.0;

                // 3. Neumáticos Pacejka Combined-Slip e inercia semi-implícita de rueda
                for (int i = 0; i < 4; i++) {
                    double cx = spec.corners[i][0], cy = spec.corners[i][1];
                    double xwWorld = px + cx * cPsi - cy * sPsi;
                    double ywWorld = py + cx * sPsi + cy * cPsi;

                    boolean onIce = false;
                    for (double[] z : iceZones) {
                        if (xwWorld >= z[0] && xwWorld <= z[1] && ywWorld >= z[2] && ywWorld <= z[3]) {
                            onIce = true;
                            break;
                        }
                    }
                    if (onIce) anyWheelOnIce = true;
                    double muLoc = onIce ? muIce : muNom;
                    muMinSub = Math.min(muMinSub, muLoc);

                    double vbxI = vxB - r * cy;
                    double vbyI = vyB + r * cx;
                    double cd = Math.cos(deltas[i]), sd = Math.sin(deltas[i]);
                    double vwxI = vbxI * cd + vbyI * sd;
                    double vwyI = -vbxI * sd + vbyI * cd;

                    double vTread = omegaW[i] * spec.rW;
                    odomVxB += 0.25 * (vTread * cd);
                    odomVyB += 0.25 * (vTread * sd);

                    double vDenReg = Math.max(Math.hypot(vwxI, vwyI), 0.15);
                    double denomK = Math.max(Math.abs(vTread), vDenReg);
                    double kappa = (vTread - vwxI) / denomK;
                    double lam = -vwyI / vDenReg;
                    double sComb = Math.hypot(kappa, lam);
                    slipAccum += sComb;

                    double noiseScale = roughness * 1.4 * (m / 72.0);
                    double noiseX = zNoise[2 * i] * noiseScale;
                    double noiseY = zNoise[2 * i + 1] * noiseScale;

                    double fwX, fwY;
                    if (sComb > 1e-9) {
                        double arg = bP * sComb;
                        double muEff = muLoc * Math.sin(cP * Math.atan(arg - eP * (arg - Math.atan(arg))));
                        double fMag = muEff * nLoads[i];
                        fwX = fMag * (kappa / sComb) + noiseX;
                        fwY = fMag * (lam / sComb) + noiseY;
                    } else {
                        fwX = noiseX;
                        fwY = noiseY;
                    }

                    double kTireLin = (muLoc * cP * bP * nLoads[i] * spec.rW) / denomK;
                    double impDenom = 1.0 + dtSub * (kTireLin * spec.rW + bWheel) / spec.jW;
                    double dwExplicit = ((fMot[i] - fwX) * spec.rW - bWheel * omegaW[i]) / spec.jW;
                    omegaW[i] += (dtSub * dwExplicit) / impDenom;

                    double fbX = fwX * cd - fwY * sd;
                    double fbY = fwX * sd + fwY * cd;
                    fxBTotal += fbX;
                    fyBTotal += fbY;
                    tzTotal += cx * fbY - cy * fbX;
                }

                wheelOdomX += (cPsi * odomVxB - sPsi * odomVyB) * dtSub;
                wheelOdomY += (sPsi * odomVxB + cPsi * odomVyB) * dtSub;

                // 4. Integración trapezoidal SE(2) del cuerpo rígido
                double fxExtB = cPsi * fxExtW + sPsi * fyExtW;
                double fyExtB = -sPsi * fxExtW + cPsi * fyExtW;
                fxBTotal += fxExtB - spec.dragLin * vxB;
                fyBTotal += fyExtB - spec.dragLin * vyB;
                tzTotal += tzExt - spec.dragYaw * r;

                double axB = fxBTotal / m;
                double ayB = fyBTotal / m;
                axBFilt += alphaAcc * (axB - axBFilt);
                ayBFilt += alphaAcc * (ayB - ayBFilt);
                axBLast = axB;
                ayBLast = ayB;

                double axWorld = cPsi * axB - sPsi * ayB;
                double ayWorld = sPsi * axB + cPsi * ayB;
                double alphaYaw = tzTotal / iz;

                double vxWNext = vxW + axWorld * dtSub;
                double vyWNext = vyW + ayWorld * dtSub;
                px += 0.5 * (vxW + vxWNext) * dtSub;
                py += 0.5 * (vyW + vyWNext) * dtSub;
                vxW = vxWNext;
                vyW = vyWNext;

                double rNext = r + alphaYaw * dtSub;
                psi = wrapAngle(psi + 0.5 * (r + rNext) * dtSub);
                r = rNext;

                // 5. Resolución continua de colisiones SAT con centroide de contacto (cero teletransportación)
                for (int iter = 0; iter < 2; iter++) {
                    for (double[] box : obstacleBoxes) {
                        double[] res = resolveObbAabbSatManifold(px, py, psi, spec.halfL, spec.halfW, box);
                        if (res[0] > 0.5) {
                            double nx = res[1], ny = res[2], depth = res[3], crx = res[4], cry = res[5];
                            double safeCorr = Math.min(depth + 1e-4, 0.012); // Tope estricto 12mm/substep
                            px += nx * safeCorr;
                            py += ny * safeCorr;
                            double[] velAfter = applyWallImpulse(vxW, vyW, r, m, iz, nx, ny, crx, cry);
                            vxW = velAfter[0];
                            vyW = velAfter[1];
                            r = velAfter[2];
                            if (velAfter[3] > 2.5) {
                                wallImpactOccurredThisStep = true;
                                if (velAfter[3] > lastWallImpulseNs) {
                                    lastWallImpulseNs = velAfter[3];
                                    lastWallNx = nx;
                                    lastWallNy = ny;
                                    lastWallCrx = crx;
                                    lastWallCry = cry;
                                }
                            }
                        }
                    }

                    // Perímetro exterior del campo
                    double cp = Math.cos(psi), sp = Math.sin(psi);
                    double extX = spec.halfL * Math.abs(cp) + spec.halfW * Math.abs(sp);
                    double extY = spec.halfL * Math.abs(sp) + spec.halfW * Math.abs(cp);

                    if (px - extX < 0.02) {
                        px += Math.min(0.02 - (px - extX) + 1e-4, 0.012);
                        double[] v = applyWallImpulse(vxW, vyW, r, m, iz, 1.0, 0.0, -extX, 0.0);
                        vxW = v[0]; vyW = v[1]; r = v[2];
                    } else if (px + extX > CAMPO_LARGO_M - 0.02) {
                        px -= Math.min((px + extX) - (CAMPO_LARGO_M - 0.02) + 1e-4, 0.012);
                        double[] v = applyWallImpulse(vxW, vyW, r, m, iz, -1.0, 0.0, extX, 0.0);
                        vxW = v[0]; vyW = v[1]; r = v[2];
                    }

                    if (py - extY < 0.02) {
                        py += Math.min(0.02 - (py - extY) + 1e-4, 0.012);
                        double[] v = applyWallImpulse(vxW, vyW, r, m, iz, 0.0, 1.0, 0.0, -extY);
                        vxW = v[0]; vyW = v[1]; r = v[2];
                    } else if (py + extY > CAMPO_ANCHO_M - 0.02) {
                        py -= Math.min((py + extY) - (CAMPO_ANCHO_M - 0.02) + 1e-4, 0.012);
                        double[] v = applyWallImpulse(vxW, vyW, r, m, iz, 0.0, -1.0, 0.0, extY);
                        vxW = v[0]; vyW = v[1]; r = v[2];
                    }
                }
            }

            state[0] = px; state[1] = py; state[2] = psi;
            state[3] = vxW; state[4] = vyW; state[5] = r;
            lastLocalMu = muMinSub;
            lastCombinedSlip = slipAccum / (4.0 * nSub);
        }

        public double[] getMeasuredVelWorld(double[] z) {
            return new double[]{state[3] + z[11] * 0.008, state[4] + z[12] * 0.008, state[5] + z[13] * 0.006};
        }

        public double[] getMeasuredAccelBody(double[] z) {
            return new double[]{axBLast + z[14] * 0.04, ayBLast + z[15] * 0.04};
        }

        public double[] getMeasuredWheelSpeeds(double[] z) {
            double[] ws = new double[4];
            for (int i = 0; i < 4; i++) ws[i] = omegaW[i] * spec.rW + z[16 + i] * 0.015;
            return ws;
        }

        public double[] getMeasuredDeltas(double[] z) {
            double[] dm = new double[4];
            for (int i = 0; i < 4; i++) dm[i] = wrapAngle(deltas[i] + z[20 + i] * 0.002);
            return dm;
        }

        /**
         * SAT con promedio de centroide de contacto (Contact Manifold): evita giros ficticios
         * cuando una cara plana del robot impacta contra una pared del campo.
         */
        private static double[] resolveObbAabbSatManifold(double px, double py, double theta,
                                                          double halfL, double halfW, double[] box) {
            double xmin = box[0], xmax = box[1], ymin = box[2], ymax = box[3];
            double bxC = 0.5 * (xmin + xmax);
            double byC = 0.5 * (ymin + ymax);
            double bxH = 0.5 * (xmax - xmin);
            double byH = 0.5 * (ymax - ymin);

            double c = Math.cos(theta), s = Math.sin(theta);
            double dx = px - bxC, dy = py - byC;

            double[][] axes = {{1.0, 0.0}, {0.0, 1.0}, {c, s}, {-s, c}};
            double minOverlap = Double.POSITIVE_INFINITY;
            double bestNx = 0.0, bestNy = 0.0;

            for (double[] ax : axes) {
                double rRob = halfL * Math.abs(ax[0] * c + ax[1] * s) + halfW * Math.abs(-ax[0] * s + ax[1] * c);
                double rBox = bxH * Math.abs(ax[0]) + byH * Math.abs(ax[1]);
                double distC = dx * ax[0] + dy * ax[1];
                double overlap = (rRob + rBox) - Math.abs(distC);
                if (overlap <= 0.0) {
                    return new double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                }
                if (overlap < minOverlap) {
                    minOverlap = overlap;
                    double sign = (distC >= 0.0) ? 1.0 : -1.0;
                    bestNx = ax[0] * sign;
                    bestNy = ax[1] * sign;
                }
            }

            // Encontrar proyección mínima y promediar todos los vértices dentro de 2 mm del plano de contacto
            double[] signsX = {-halfL, halfL};
            double[] signsY = {-halfW, halfW};
            double[][] relCorners = new double[4][2];
            double[] projs = new double[4];
            double minProj = Double.POSITIVE_INFINITY;
            int idx = 0;

            for (double sx : signsX) {
                for (double sy : signsY) {
                    double rx = sx * c - sy * s;
                    double ry = sx * s + sy * c;
                    double proj = rx * bestNx + ry * bestNy;
                    relCorners[idx][0] = rx;
                    relCorners[idx][1] = ry;
                    projs[idx] = proj;
                    minProj = Math.min(minProj, proj);
                    idx++;
                }
            }

            double crxSum = 0.0, crySum = 0.0;
            int count = 0;
            for (int k = 0; k < 4; k++) {
                if (projs[k] <= minProj + 0.002) {
                    crxSum += relCorners[k][0];
                    crySum += relCorners[k][1];
                    count++;
                }
            }
            double crx = (count > 0) ? (crxSum / count) : 0.0;
            double cry = (count > 0) ? (crySum / count) : 0.0;
            return new double[]{1.0, bestNx, bestNy, minOverlap, crx, cry};
        }

        private static double[] applyWallImpulse(double vxW, double vyW, double r,
                                                 double m, double iz,
                                                 double nx, double ny, double crx, double cry) {
            double vcx = vxW - r * cry;
            double vcy = vyW + r * crx;
            double vn = vcx * nx + vcy * ny;
            if (vn >= 0.0) return new double[]{vxW, vyW, r, 0.0};

            double eWall = 0.20;
            double muWall = 0.22;
            double rcn = crx * ny - cry * nx;
            double invMEff = (1.0 / m) + (rcn * rcn) / iz;
            double jn = -(1.0 + eWall) * vn / invMEff;

            vxW += (jn / m) * nx;
            vyW += (jn / m) * ny;
            r += (jn * rcn) / iz;

            double tx = -ny, ty = nx;
            double vt = (vxW - r * cry) * tx + (vyW + r * crx) * ty;
            double rct = crx * ty - cry * tx;
            double invMt = (1.0 / m) + (rct * rct) / iz;
            double jt = Math.max(-muWall * jn, Math.min(muWall * jn, -vt / invMt));

            vxW += (jt / m) * tx;
            vyW += (jt / m) * ty;
            r += (jt * rct) / iz;
            return new double[]{vxW, vyW, r, jn};
        }
    }

    // =========================================================================
    // 10. PLANIFICADOR EN FORMA DE L, SUAVIZADO C^2 Y PERFIL QUÍNTICO DE GIRO
    // =========================================================================
    private List<EstadoTrayectoria> obtenerTrayectoriaCompletaM(List<Pose2d> waypoints, double margenSeguridadM,
                                                                List<double[]> outLVertices) {
        if (waypoints == null || waypoints.size() < 2) return Collections.emptyList();
        List<double[]> cajasInfladas = crearCajasObstaculosInfladas(margenSeguridadM);

        List<double[]> nodosTotales = new ArrayList<>();
        for (int i = 0; i < waypoints.size() - 1; i++) {
            double[] ini = {waypoints.get(i).getX(), waypoints.get(i).getY()};
            double[] fin = {waypoints.get(i + 1).getX(), waypoints.get(i + 1).getY()};
            List<double[]> tramo = planificarRutaOrtogonalM(ini, fin, cajasInfladas);
            if (tramo.isEmpty()) return Collections.emptyList();
            if (nodosTotales.isEmpty()) nodosTotales.addAll(tramo);
            else for (int k = 1; k < tramo.size(); k++) nodosTotales.add(tramo.get(k));
        }

        // Simplificar puntos duplicados y fusionar segmentos colineales para obtener los vértices puros en L
        nodosTotales = simplificarCaminoColineal(nodosTotales);
        List<double[]> nodosFiltrados = simplificarCaminoColineal(eliminarMicroEscalones(nodosTotales, cajasInfladas));

        if (outLVertices != null) {
            outLVertices.clear();
            for (double[] v : nodosFiltrados) outLVertices.add(new double[]{v[0], v[1]});
        }

        List<double[]> posicionesSuaves = suavizarCaminoOrtogonalM(nodosFiltrados);
        List<EstadoTrayectoria> perfilada = perfilamientoFisicoON(posicionesSuaves, waypoints);

        for (EstadoTrayectoria st : perfilada) {
            if (!verificarPosicionRobotLibre(st.x, st.y, st.theta, 0.02)) {
                return Collections.emptyList();
            }
        }
        return perfilada;
    }

    private List<double[]> suavizarCaminoOrtogonalM(List<double[]> camino) {
        if (camino.size() < 3) return camino;
        List<double[]> salida = new ArrayList<>();
        salida.add(camino.get(0));

        for (int i = 1; i < camino.size() - 1; i++) {
            double[] a = camino.get(i - 1), b = camino.get(i), c = camino.get(i + 1);
            double lenIn = dist(a, b), lenOut = dist(b, c);
            if (lenIn < 1e-5 || lenOut < 1e-5) continue;

            double[] uIn = {(b[0] - a[0]) / lenIn, (b[1] - a[1]) / lenIn};
            double[] uOut = {(c[0] - b[0]) / lenOut, (c[1] - b[1]) / lenOut};

            double dIn = Math.min(RADIO_CURVA_MAX_M, lenIn * 0.42);
            double dOut = Math.min(RADIO_CURVA_MAX_M, lenOut * 0.42);

            double[] entrada = {b[0] - uIn[0] * dIn, b[1] - uIn[1] * dIn};
            double[] salidaCurva = {b[0] + uOut[0] * dOut, b[1] + uOut[1] * dOut};
            salida.addAll(puntosBezierCuadratica(entrada, b, salidaCurva));
        }
        salida.add(camino.get(camino.size() - 1));

        List<double[]> denso = new ArrayList<>();
        denso.add(salida.get(0));
        for (int i = 1; i < salida.size(); i++) {
            double[] p0 = denso.get(denso.size() - 1);
            double[] p1 = salida.get(i);
            double d = dist(p0, p1);
            if (d > 1e-5) {
                int steps = Math.max(1, (int) Math.ceil(d / 0.03));
                for (int s = 1; s <= steps; s++) {
                    double al = (double) s / steps;
                    denso.add(new double[]{p0[0] + al * (p1[0] - p0[0]), p0[1] + al * (p1[1] - p0[1])});
                }
            }
        }
        return denso;
    }

    private List<double[]> puntosBezierCuadratica(double[] p0, double[] p1, double[] p2) {
        List<double[]> pts = new ArrayList<>();
        int n = 16;
        for (int i = 0; i <= n; i++) {
            double t = (double) i / n;
            double u = 1.0 - t;
            pts.add(new double[]{
                u * u * p0[0] + 2 * u * t * p1[0] + t * t * p2[0],
                u * u * p0[1] + 2 * u * t * p1[1] + t * t * p2[1]
            });
        }
        return pts;
    }

    private List<EstadoTrayectoria> perfilamientoFisicoON(List<double[]> trayectoriaPosM, List<Pose2d> waypoints) {
        List<EstadoTrayectoria> resultado = new ArrayList<>();
        int n = trayectoriaPosM.size();
        double[] distAcum = new double[n];
        for (int i = 1; i < n; i++) {
            distAcum[i] = distAcum[i - 1] + dist(trayectoriaPosM.get(i - 1), trayectoriaPosM.get(i));
        }

        double totalDist = distAcum[n - 1];
        double thetaStart = waypoints.get(0).getRotation().getRadians();
        double thetaEnd = waypoints.get(waypoints.size() - 1).getRotation().getRadians();
        double deltaTheta = wrapAngle(thetaEnd - thetaStart);

        double vMax = 2.40;
        double aMax = 2.80;
        double aLatMax = 0.52 * G;

        double[] vFwd = new double[n];
        vFwd[0] = 0.0;

        for (int i = 1; i < n; i++) {
            double ds = distAcum[i] - distAcum[i - 1];
            double kCurv = 0.0;
            if (i < n - 1) {
                double[] p1 = trayectoriaPosM.get(i - 1), p2 = trayectoriaPosM.get(i), p3 = trayectoriaPosM.get(i + 1);
                double d1 = dist(p1, p2), d2 = dist(p2, p3), d3 = dist(p1, p3);
                if (d1 > 1e-4 && d2 > 1e-4 && d3 > 1e-4) {
                    double cross = Math.abs((p2[0] - p1[0]) * (p3[1] - p2[1]) - (p2[1] - p1[1]) * (p3[0] - p2[0]));
                    kCurv = 2.0 * cross / (d1 * d2 * d3 + 1e-6);
                }
            }
            double vLimCurva = (kCurv > 0.08) ? Math.min(vMax, Math.sqrt(aLatMax / kCurv)) : vMax;
            vFwd[i] = Math.min(vLimCurva, Math.sqrt(vFwd[i - 1] * vFwd[i - 1] + 2.0 * aMax * ds));
        }

        vFwd[n - 1] = 0.0;
        for (int i = n - 2; i >= 0; i--) {
            double ds = distAcum[i + 1] - distAcum[i];
            vFwd[i] = Math.min(vFwd[i], Math.sqrt(vFwd[i + 1] * vFwd[i + 1] + 2.0 * aMax * ds));
        }

        double t = 0.0;
        for (int i = 0; i < n; i++) {
            double[] pt = trayectoriaPosM.get(i);
            double tau = (totalDist > 1e-6) ? (distAcum[i] / totalDist) : 0.0;
            // Polinomio quíntico C^2: velocidad y aceleración angular iniciales nulas (cero sacudida al arrancar)
            double sRot = tau * tau * tau * (10.0 - 15.0 * tau + 6.0 * tau * tau);
            double targetTheta = wrapAngle(thetaStart + sRot * deltaTheta);

            if (i > 0) {
                double ds = distAcum[i] - distAcum[i - 1];
                double vMed = Math.max(0.12, 0.5 * (vFwd[i - 1] + vFwd[i]));
                t += ds / vMed;
            }
            resultado.add(new EstadoTrayectoria(pt[0], pt[1], targetTheta, vFwd[i], t));
        }
        return resultado;
    }

    private boolean verificarPosicionRobotLibre(double x, double y, double theta, double extraMargin) {
        if (m_collisionsEnabledEntry != null && !m_collisionsEnabledEntry.getBoolean(true)) return true;
        double c = Math.cos(theta), s = Math.sin(theta);
        double extX = (m_spec.halfL + extraMargin) * Math.abs(c) + (m_spec.halfW + extraMargin) * Math.abs(s);
        double extY = (m_spec.halfL + extraMargin) * Math.abs(s) + (m_spec.halfW + extraMargin) * Math.abs(c);

        if (x - extX < 0.06 || x + extX > CAMPO_LARGO_M - 0.06 || y - extY < 0.06 || y + extY > CAMPO_ANCHO_M - 0.06) {
            return false;
        }
        for (double[] box : m_cajasObstaculosReales) {
            double[] res = MultiClassSwerveSE2Plant.resolveObbAabbSatManifold(
                x, y, theta, m_spec.halfL + extraMargin, m_spec.halfW + extraMargin, box
            );
            if (res[0] > 0.5) return false;
        }
        return true;
    }

    private List<double[]> planificarRutaOrtogonalM(double[] inicio, double[] fin, List<double[]> cajas) {
        double[] limites = {0.06, CAMPO_LARGO_M - 0.06, 0.06, CAMPO_ANCHO_M - 0.06};
        if (dist(inicio, fin) < 1e-7) return Collections.singletonList(new double[]{inicio[0], inicio[1]});

        TreeSet<Double> xs = new TreeSet<>();
        TreeSet<Double> ys = new TreeSet<>();
        xs.add(round7(inicio[0])); xs.add(round7(fin[0]));
        ys.add(round7(inicio[1])); ys.add(round7(fin[1]));

        for (double[] c : cajas) {
            double[] candX = {c[0] - SEPARACION_BORDE_M, c[1] + SEPARACION_BORDE_M};
            for (double x : candX) if (limites[0] <= x && x <= limites[1]) xs.add(round7(x));
            double[] candY = {c[2] - SEPARACION_BORDE_M, c[3] + SEPARACION_BORDE_M};
            for (double y : candY) if (limites[2] <= y && y <= limites[3]) ys.add(round7(y));
        }

        List<double[]> nodos = new ArrayList<>();
        Map<String, Integer> mapaIndice = new HashMap<>();
        Map<Double, List<Integer>> filas = new HashMap<>();
        Map<Double, List<Integer>> columnas = new HashMap<>();

        for (double y : ys) {
            for (double x : xs) {
                double[] p = {x, y};
                boolean enObstaculo = false;
                for (double[] c : cajas) {
                    if (puntoEnCaja(p, c, 1e-9)) {
                        enObstaculo = true;
                        break;
                    }
                }
                if (puntoEnCampo(p, limites) && !enObstaculo) {
                    int idx = nodos.size();
                    nodos.add(p);
                    mapaIndice.put(x + "," + y, idx);
                    filas.computeIfAbsent(y, k -> new ArrayList<>()).add(idx);
                    columnas.computeIfAbsent(x, k -> new ArrayList<>()).add(idx);
                }
            }
        }

        double[] iniRound = {round7(inicio[0]), round7(inicio[1])};
        double[] finRound = {round7(fin[0]), round7(fin[1])};

        int idxIni = mapaIndice.computeIfAbsent(iniRound[0] + "," + iniRound[1], k -> {
            int id = nodos.size();
            nodos.add(iniRound);
            filas.computeIfAbsent(iniRound[1], key -> new ArrayList<>()).add(id);
            columnas.computeIfAbsent(iniRound[0], key -> new ArrayList<>()).add(id);
            return id;
        });

        int idxFin = mapaIndice.computeIfAbsent(finRound[0] + "," + finRound[1], k -> {
            int id = nodos.size();
            nodos.add(finRound);
            filas.computeIfAbsent(finRound[1], key -> new ArrayList<>()).add(id);
            columnas.computeIfAbsent(finRound[0], key -> new ArrayList<>()).add(id);
            return id;
        });

        int totalNodos = nodos.size();
        List<List<double[]>> ady = new ArrayList<>(totalNodos);
        for (int i = 0; i < totalNodos; i++) ady.add(new ArrayList<>());

        for (List<Integer> lista : filas.values()) {
            lista.sort((a, b) -> Double.compare(nodos.get(a)[0], nodos.get(b)[0]));
            for (int i = 0; i < lista.size() - 1; i++) {
                int u = lista.get(i), v = lista.get(i + 1);
                double[] pu = nodos.get(u), pv = nodos.get(v);
                if (segmentoLibre(pu, pv, cajas)) {
                    double peso = Math.abs(pv[0] - pu[0]);
                    ady.get(u).add(new double[]{v, peso});
                    ady.get(v).add(new double[]{u, peso});
                }
            }
        }

        for (List<Integer> lista : columnas.values()) {
            lista.sort((a, b) -> Double.compare(nodos.get(a)[1], nodos.get(b)[1]));
            for (int i = 0; i < lista.size() - 1; i++) {
                int u = lista.get(i), v = lista.get(i + 1);
                double[] pu = nodos.get(u), pv = nodos.get(v);
                if (segmentoLibre(pu, pv, cajas)) {
                    double peso = Math.abs(pv[1] - pu[1]);
                    ady.get(u).add(new double[]{v, peso});
                    ady.get(v).add(new double[]{u, peso});
                }
            }
        }

        double[] distancias = new double[totalNodos];
        int[] anterior = new int[totalNodos];
        Arrays.fill(distancias, Double.POSITIVE_INFINITY);
        Arrays.fill(anterior, -1);

        distancias[idxIni] = 0.0;
        PriorityQueue<double[]> heap = new PriorityQueue<>((a, b) -> Double.compare(a[0], b[0]));
        heap.add(new double[]{0.0, idxIni});

        while (!heap.isEmpty()) {
            double[] top = heap.poll();
            double costeActual = top[0];
            int actual = (int) top[1];
            if (costeActual > distancias[actual] + 1e-10) continue;
            if (actual == idxFin) break;
            for (double[] arista : ady.get(actual)) {
                int vecino = (int) arista[0];
                double peso = arista[1];
                double nuevo = costeActual + peso;
                if (nuevo + 1e-10 < distancias[vecino]) {
                    distancias[vecino] = nuevo;
                    anterior[vecino] = actual;
                    heap.add(new double[]{nuevo, vecino});
                }
            }
        }

        if (!Double.isFinite(distancias[idxFin])) return Collections.emptyList();

        List<double[]> camino = new ArrayList<>();
        int curr = idxFin;
        while (curr != -1) {
            camino.add(nodos.get(curr));
            if (curr == idxIni) break;
            curr = anterior[curr];
        }
        Collections.reverse(camino);
        return simplificarCaminoColineal(camino);
    }

    private List<double[]> crearCajasObstaculosInfladas(double margenSeguridadM) {
        double radio = Math.hypot(m_spec.halfW, m_spec.halfL) + margenSeguridadM;
        List<double[]> resultado = new ArrayList<>(m_cajasObstaculosReales.size());
        for (double[] c : m_cajasObstaculosReales) {
            resultado.add(new double[]{c[0] - radio, c[1] + radio, c[2] - radio, c[3] + radio});
        }
        return resultado;
    }

    private boolean puntoEnCaja(double[] p, double[] caja, double tol) {
        return (caja[0] - tol <= p[0] && p[0] <= caja[1] + tol && caja[2] - tol <= p[1] && p[1] <= caja[3] + tol);
    }

    private boolean segmentoIntersectaCaja(double[] a, double[] b, double[] caja) {
        double x0 = a[0], y0 = a[1];
        double dx = b[0] - x0, dy = b[1] - y0;
        double xmin = caja[0], xmax = caja[1], ymin = caja[2], ymax = caja[3];

        double[] pVals = {-dx, dx, -dy, dy};
        double[] qVals = {x0 - xmin, xmax - x0, y0 - ymin, ymax - y0};
        double t0 = 0.0, t1 = 1.0;

        for (int i = 0; i < 4; i++) {
            double pv = pVals[i], qv = qVals[i];
            if (Math.abs(pv) < 1e-12) {
                if (qv < 0.0) return false;
            } else {
                double r = qv / pv;
                if (pv < 0.0) {
                    if (r > t1) return false;
                    t0 = Math.max(t0, r);
                } else {
                    if (r < t0) return false;
                    t1 = Math.min(t1, r);
                }
            }
        }
        return t0 <= t1 + 1e-10;
    }

    private boolean segmentoLibre(double[] a, double[] b, List<double[]> cajas) {
        for (double[] c : cajas) if (segmentoIntersectaCaja(a, b, c)) return false;
        return true;
    }

    private boolean puntoEnCampo(double[] p, double[] limites) {
        return (limites[0] - 1e-8 <= p[0] && p[0] <= limites[1] + 1e-8
            && limites[2] - 1e-8 <= p[1] && p[1] <= limites[3] + 1e-8);
    }

    /**
     * Elimina puntos duplicados y fusiona vértices intermedios colineales
     * para obtener exclusivamente los vértices de esquina en forma de L.
     */
    private List<double[]> simplificarCaminoColineal(List<double[]> camino) {
        if (camino == null || camino.isEmpty()) return new ArrayList<>();
        List<double[]> sinDup = new ArrayList<>();
        for (double[] p : camino) {
            if (sinDup.isEmpty() || dist(p, sinDup.get(sinDup.size() - 1)) > 1e-6) {
                sinDup.add(new double[]{p[0], p[1]});
            }
        }
        if (sinDup.size() < 3) return sinDup;

        List<double[]> lVertices = new ArrayList<>();
        lVertices.add(sinDup.get(0));
        for (int i = 1; i < sinDup.size() - 1; i++) {
            double[] prev = lVertices.get(lVertices.size() - 1);
            double[] curr = sinDup.get(i);
            double[] next = sinDup.get(i + 1);
            double cross = Math.abs((curr[0] - prev[0]) * (next[1] - curr[1]) - (curr[1] - prev[1]) * (next[0] - curr[0]));
            if (cross > 1e-4) {
                lVertices.add(curr);
            }
        }
        lVertices.add(sinDup.get(sinDup.size() - 1));
        return lVertices;
    }

    private List<double[]> eliminarMicroEscalones(List<double[]> camino, List<double[]> cajas) {
        if (camino.size() < 4) return camino;
        List<double[]> resultado = new ArrayList<>();
        resultado.add(camino.get(0));
        int i = 1, n = camino.size();

        while (i < n) {
            double[] a = resultado.get(resultado.size() - 1);
            double[] b = camino.get(i);
            if (i + 1 < n) {
                double[] c = camino.get(i + 1);
                if (dist(b, c) < UMBRAL_MICRO_ESCALON_M && segmentoLibre(a, c, cajas)) {
                    resultado.add(c);
                    i += 2;
                    continue;
                }
            }
            resultado.add(b);
            i++;
        }
        return resultado;
    }

    /**
     * Simplificación geométrica Ramer-Douglas-Peucker para resumir el camino real
     * ejecutado por el robot en sus vértices principales en forma de L.
     */
    private static List<double[]> simplificarPolilineaRDP(List<double[]> pts, double epsilon) {
        if (pts == null || pts.size() <= 2) return pts == null ? Collections.emptyList() : new ArrayList<>(pts);
        double maxDist = 0.0;
        int index = 0;
        int end = pts.size() - 1;
        for (int i = 1; i < end; i++) {
            double d = distanciaPuntoSegmento(pts.get(i), pts.get(0), pts.get(end));
            if (d > maxDist) {
                index = i;
                maxDist = d;
            }
        }
        if (maxDist > epsilon) {
            List<double[]> left = simplificarPolilineaRDP(pts.subList(0, index + 1), epsilon);
            List<double[]> right = simplificarPolilineaRDP(pts.subList(index, end + 1), epsilon);
            List<double[]> res = new ArrayList<>(left.subList(0, left.size() - 1));
            res.addAll(right);
            return res;
        } else {
            return Arrays.asList(pts.get(0), pts.get(end));
        }
    }

    private static double distanciaPuntoSegmento(double[] p, double[] a, double[] b) {
        double dx = b[0] - a[0], dy = b[1] - a[1];
        double l2 = dx * dx + dy * dy;
        if (l2 < 1e-12) return Math.hypot(p[0] - a[0], p[1] - a[1]);
        double t = Math.max(0.0, Math.min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2));
        return Math.hypot(p[0] - (a[0] + t * dx), p[1] - (a[1] + t * dy));
    }

    private static double calcularLongitudPolilinea(List<double[]> pts) {
        double len = 0.0;
        for (int i = 1; i < pts.size(); i++) {
            len += Math.hypot(pts.get(i)[0] - pts.get(i - 1)[0], pts.get(i)[1] - pts.get(i - 1)[1]);
        }
        return len;
    }

    // =========================================================================
    // 11. PUBLICACIÓN VISUAL, CARGA DE MAPAS Y UTILIDADES
    // =========================================================================
    private void publicarTrayectoriaVisual(List<EstadoTrayectoria> rutaGlobal, List<double[]> lVertices) {
        Pose2d[] posesArray = new Pose2d[rutaGlobal.size()];
        List<Pose2d> posesList = new ArrayList<>(rutaGlobal.size());
        for (int i = 0; i < rutaGlobal.size(); i++) {
            EstadoTrayectoria st = rutaGlobal.get(i);
            Pose2d p = new Pose2d(st.x, st.y, new Rotation2d(st.theta));
            posesArray[i] = p;
            posesList.add(p);
        }
        m_trajectoryPublisher.set(posesArray);
        m_field2d.getObject("PlannedPath").setPoses(posesList);

        Pose2d[] lArray = new Pose2d[lVertices.size()];
        List<Pose2d> lList = new ArrayList<>(lVertices.size());
        for (int i = 0; i < lVertices.size(); i++) {
            double[] v = lVertices.get(i);
            double ang = (i < lVertices.size() - 1)
                ? Math.atan2(lVertices.get(i + 1)[1] - v[1], lVertices.get(i + 1)[0] - v[0])
                : 0.0;
            Pose2d p = new Pose2d(v[0], v[1], new Rotation2d(ang));
            lArray[i] = p;
            lList.add(p);
        }
        m_plannedLPathPublisher.set(lArray);
        m_field2d.getObject("PlannedLPath").setPoses(lList);
    }

    private void cargarObstaculosDesdeArchivo(String campoSeleccionado) {
        m_obstaculos.clear();
        m_cajasObstaculosReales.clear();

        File archivo = null;
        for (String cand : OBSTACLES_CANDIDATE_PATHS) {
            File f = new File(cand);
            if (f.exists() && f.isFile()) {
                archivo = f;
                break;
            }
        }
        if (archivo == null) {
            System.out.println("[MAPA INFO] No se halló map_obstacles_calibrated.txt; operando con perímetro de campo.");
            return;
        }

        StringBuilder contenido = new StringBuilder();
        try (BufferedReader reader = new BufferedReader(new FileReader(archivo))) {
            String linea;
            while ((linea = reader.readLine()) != null) contenido.append(linea).append('\n');
        } catch (IOException e) {
            return;
        }

        String numero = "([-+]?(?:\\d+(?:\\.\\d*)?|\\.\\d+)(?:[eE][+]?\\d+)?)";
        Pattern bloquePattern = Pattern.compile("/\\*\\s*OBSTACLE\\s*\\*/(.*?)/\\*\\s*END_OBSTACLE\\s*\\*/", Pattern.DOTALL | Pattern.CASE_INSENSITIVE);
        Pattern segmentoPattern = Pattern.compile("\\[\\s*" + numero + "\\s*,\\s*" + numero +
            "\\s*,\\s*\\[\\s*" + numero + "\\s*,\\s*" + numero + "\\s*\\]\\s*,\\s*\\[\\s*" + numero + "\\s*,\\s*" +
            numero + "\\s*\\]\\s*,\\s*\"([^\"]*)\"\\s*\\]", Pattern.DOTALL | Pattern.CASE_INSENSITIVE);

        Matcher bloqueMatcher = bloquePattern.matcher(contenido.toString());
        while (bloqueMatcher.find()) {
            String cuerpoBloque = bloqueMatcher.group(1);
            Matcher segmentoMatcher = segmentoPattern.matcher(cuerpoBloque);

            double cajaMinX = Double.POSITIVE_INFINITY, cajaMaxX = Double.NEGATIVE_INFINITY;
            double cajaMinY = Double.POSITIVE_INFINITY, cajaMaxY = Double.NEGATIVE_INFINITY;
            boolean grupoValido = false;

            while (segmentoMatcher.find()) {
                double m = Double.parseDouble(segmentoMatcher.group(1));
                double b = Double.parseDouble(segmentoMatcher.group(2));
                double minX = Double.parseDouble(segmentoMatcher.group(3));
                double maxX = Double.parseDouble(segmentoMatcher.group(4));
                double minY = Double.parseDouble(segmentoMatcher.group(5));
                double maxY = Double.parseDouble(segmentoMatcher.group(6));
                String campoObstaculo = segmentoMatcher.group(7).trim();

                if (!coincideCampo(campoSeleccionado, campoObstaculo)) continue;

                double[][] segmento = construirSegmentoObstaculo(m, b, minX, maxX, minY, maxY);
                if (segmento != null) {
                    m_obstaculos.add(segmento);
                    double x0 = Math.min(minX, maxX), x1 = Math.max(minX, maxX);
                    double y0 = Math.min(minY, maxY), y1 = Math.max(minY, maxY);
                    cajaMinX = Math.min(cajaMinX, x0);
                    cajaMaxX = Math.max(cajaMaxX, x1);
                    cajaMinY = Math.min(cajaMinY, y0);
                    cajaMaxY = Math.max(cajaMaxY, y1);
                    grupoValido = true;
                }
            }
            if (grupoValido) {
                m_cajasObstaculosReales.add(new double[]{cajaMinX, cajaMaxX, cajaMinY, cajaMaxY});
            }
        }
        System.out.println("[MAPA OK] Cargado desde " + archivo.getPath() + " | Obstáculos: " + m_cajasObstaculosReales.size());
    }

    private boolean coincideCampo(String seleccionado, String enArchivo) {
        if (seleccionado == null || enArchivo == null) return true;
        String s1 = seleccionado.trim().toLowerCase(Locale.ROOT);
        String s2 = enArchivo.trim().toLowerCase(Locale.ROOT);
        return s1.contains(s2) || s2.contains(s1);
    }

    private double[][] construirSegmentoObstaculo(double m, double b, double minX, double maxX, double minY, double maxY) {
        double x1 = minX, y1 = minY, x2 = maxX, y2 = maxY;
        if (Math.abs(maxX - minX) <= VERTICAL_BORDER_X_RANGE_TOLERANCE) {
            x1 = minX; y1 = minY; x2 = minX; y2 = maxY;
        } else if (Math.abs(m) <= COLLISION_EPSILON) {
            x1 = minX; y1 = b; x2 = maxX; y2 = b;
        } else {
            x1 = minX; y1 = m * x1 + b; x2 = maxX; y2 = m * x2 + b;
        }
        return new double[][]{{x1, y1}, {x2, y2}};
    }

    private void cargarCampo() {
        String campoSeleccionado = m_fieldEntry != null ? m_fieldEntry.getString(DEFAULT_FIELD) : DEFAULT_FIELD;
        try {
            m_fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
            m_loadedFieldName = campoSeleccionado;
        } catch (Exception e) {
            m_loadedFieldName = DEFAULT_FIELD;
        }
    }

    private void comprobarCambioDeMapaYClase() {
        if (m_fieldEntry != null) {
            String campoSeleccionado = m_fieldEntry.getString(DEFAULT_FIELD);
            if (m_loadedFieldName == null || !campoSeleccionado.equals(m_loadedFieldName)) {
                cargarCampo();
                cargarObstaculosDesdeArchivo(m_loadedFieldName);
            }
        }
        if (m_robotClassEntry != null) {
            int reqIdx = Math.max(0, Math.min(2, (int) m_robotClassEntry.getInteger(m_selectedClassIdx)));
            if (reqIdx != m_selectedClassIdx) {
                m_selectedClassIdx = reqIdx;
                m_spec = ROBOT_CLASSES[m_selectedClassIdx];
                m_plant = new MultiClassSwerveSE2Plant(m_spec, m_spec.mNom, m_spec.izNom, new double[]{1, 1, 1, 1}, DT_CONTROL, PHYS_SUBSTEPS);
                m_controller = new ProposedO1SwerveController(m_spec, DT_CONTROL, true);
                posicionarEnReposoSeguro(m_trueWorldPose);
                System.out.println("[CLASS SWITCH] Plataforma cambiada a " + m_spec.name + " (Zero-Retuning DARE)");
            }
        }
    }

    private double dist(double[] a, double[] b) {
        return Math.hypot(b[0] - a[0], b[1] - a[1]);
    }

    private double round7(double val) {
        return Math.round(val * 1e7) / 1e7;
    }

    private double normalizarAngulo(double angulo) {
        return wrapAngle(angulo);
    }

    public static double wrapAngle(double a) {
        double res = (a + Math.PI) % (2.0 * Math.PI);
        if (res < 0.0) res += 2.0 * Math.PI;
        return res - Math.PI;
    }
}

