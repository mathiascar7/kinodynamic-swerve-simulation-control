package frc.robot;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;


import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
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
 * Robot Java - Control Swerve Local Nativo O(1)
 * + Interpolación Continua de Rotación Heading theta(s)
 * + Bucle de Pruebas Automáticas con Generación Estocástica Aleatoria (Tecla C)
 * + Rechazo de Perturbaciones e Impactos Externa sin Detención Falsa
 */
public class Robot extends TimedRobot {


    // =========================================================================
    // 1. ESTRUCTURAS DE DATOS
    // =========================================================================
    public static class EstadoTrayectoria {
        public final double x, y, theta, v, t;
        public final boolean esCorredor;
        public final double kCurv;


        public EstadoTrayectoria(double x, double y, double theta, double v, double t, boolean esCorredor, double kCurv) {
            this.x = x; this.y = y; this.theta = theta;
            this.v = v; this.t = t;
            this.esCorredor = esCorredor;
            this.kCurv = kCurv;
        }


        public EstadoTrayectoria(double x, double y, double theta, double v, double t) {
            this(x, y, theta, v, t, false, 0.0);
        }
    }


    // =========================================================================
    // 2. CONSTANTES Y PARÁMETROS MECATRÓNICOS
    // =========================================================================
    private static final double CAMPO_LARGO_M = 16.54;
    private static final double CAMPO_ANCHO_M = 8.07;
    private static final String DEFAULT_FIELD = "2026 - Rebuilt Welded";
    private static final String DEFAULT_OBSTACLES_PATH = "C:\\Downloads\\map_obstacles_calibrated.txt";


    private static final double SEPARACION_BORDE_M = 0.04;
    private static final double RADIO_CURVA_MAX_M = 0.55;
    private static final double MARGEN_SEGURIDAD_DEFAULT_M = 0.20;
    private static final double MARGEN_ESTRICTO_COLISION_M = 0.015;
    private static final double UMBRAL_MICRO_ESCALON_M = 0.18;
    private static final double COLLISION_EPSILON = 1e-9;
    private static final double VERTICAL_BORDER_X_RANGE_TOLERANCE = 0.0000011;


    private static final int BUTTON_Z = 1;
    private static final int BUTTON_X = 2;
    private static final int BUTTON_C = 3;


    private double m_robotWidth = 0.850;
    private double m_robotLength = 0.850;
    private double m_robotMass = 74.088;
    private double m_robotMoi = 6.883;
    private double m_wheelRadius = 0.048;
    private double m_driveGearing = 5.143;
    private double m_trueMaxSpeed = 4.200;
    private String m_obstaclesFilePath = DEFAULT_OBSTACLES_PATH;


    private Translation2d m_frontLeft = new Translation2d(m_robotLength / 2.0, m_robotWidth / 2.0);
    private Translation2d m_frontRight = new Translation2d(m_robotLength / 2.0, -m_robotWidth / 2.0);
    private Translation2d m_backRight = new Translation2d(-m_robotLength / 2.0, -m_robotWidth / 2.0);
    private Translation2d m_backLeft = new Translation2d(-m_robotLength / 2.0, m_robotWidth / 2.0);


    // Estado Físico y Odometría
    private Joystick m_joystick;
    private Pose2d m_trueWorldPose = new Pose2d(2.5, 2.0, new Rotation2d(0.0));
    private Pose2d m_poseInicioGlobalManiobra = new Pose2d(2.5, 2.0, new Rotation2d(0.0));
    private Pose2d m_localEstimatedPose = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
    private Pose2d m_rawWheelOdometryPose = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
    private Rotation2d m_forwardHeading = new Rotation2d(0.0);
    private double m_ultimoTimestamp = 0.0;
    private double m_tiempoUltimoReinicioPrueba = 0.0;


    // Dinámica de Perturbación Externa (Empujones/Impactos)
    private double m_tiempoRestanteImpacto = 0.0;
    private double m_duracionTotalImpacto = 0.28;
    private Translation2d m_fuerzaExternaMundoN = new Translation2d();
    private double m_torqueExternoMundoNm = 0.0;
    private Translation2d m_velDeslizamientoRealMundo = new Translation2d();
    private double m_omegaDeslizamientoReal = 0.0;


    private int m_impactosDisparadosCount = 0;          
    private double m_ultimoProgresoImpactoPct = -100.0;


    // Marcadores Visuales en AdvantageScope
    private final List<Pose2d> m_impactMarkerPoses = new ArrayList<>();
    private StructArrayPublisher<Pose2d> m_impactMarkersPublisher;


    private double m_vxLastLocal = 0.0, m_vyLastLocal = 0.0;
    private double[] m_angulosModulos = new double[]{0, 0, 0, 0};
    private double[] m_velocidadesModulos = new double[]{0, 0, 0, 0};
    private double[] m_corrientesModulos = new double[]{0, 0, 0, 0};


    private final List<double[][]> m_obstaculos = new ArrayList<>();
    private final List<double[]> m_cajasObstaculosReales = new ArrayList<>();


    private final LocalInertiaGhostFollower m_follower = new LocalInertiaGhostFollower();
    private final ControladorCinodinamicoO1 m_controladorO1 = new ControladorCinodinamicoO1();


    private Pose2d m_puntoInicioGlobal = null;
    private Pose2d m_puntoFinGlobal = null;
    private List<EstadoTrayectoria> m_trayectoriaLocalActiva = new ArrayList<>();
    private boolean m_siguiendoRuta = false;
    private boolean m_modoPruebasAutomaticas = false;
    private String m_ultimoErrorPlanificador = "Presiona [C] para iniciar/detener bucle de pruebas aleatorias.";


    private ShuffleboardTab m_tab;
    private GenericEntry m_fieldEntry;
    private GenericEntry m_collisionsEnabledEntry;
    private GenericEntry m_safetyMarginEntry;
    private GenericEntry m_plannerStatusEntry;
    private GenericEntry m_observerTelemetryEntry;
    private Field2d m_field2d;


    private StructPublisher<Pose2d> m_trueWorldPosePublisher;
    private StructPublisher<Pose2d> m_localEstimatedPosePublisher;
    private StructPublisher<Pose2d> m_rawOdometryPosePublisher;
    private StructPublisher<Pose2d> m_ghostPosePublisher;
    private StructPublisher<Pose2d> m_stoppingPointPublisher;
    private StructPublisher<Pose2d> m_startPosePublisher;
    private StructPublisher<Pose2d> m_endPosePublisher;
    private StructArrayPublisher<Pose2d> m_trajectoryPublisher;
    private DoublePublisher m_estimatedImpactForcePublisher;


    private AprilTagFieldLayout m_fieldLayout;
    private String m_loadedFieldName = null;


    // =========================================================================
    // 3. CICLO DE VIDA WPILIB
    // =========================================================================
    @Override
    public void robotInit() {
        m_joystick = new Joystick(0);
        m_ultimoTimestamp = Timer.getFPGATimestamp();
        actualizarDimensionesHitbox();


        m_tab = Shuffleboard.getTab("Navigation");
        m_fieldEntry = m_tab.add("Field Layout", DEFAULT_FIELD).withPosition(0, 0).withSize(3, 1).getEntry();
        m_collisionsEnabledEntry = m_tab.add("Collisions Enabled", true).withWidget(BuiltInWidgets.kToggleButton).withPosition(3, 0).withSize(2, 1).getEntry();
        m_safetyMarginEntry = m_tab.add("Safety Margin (m)", MARGEN_SEGURIDAD_DEFAULT_M).withPosition(5, 0).withSize(2, 1).getEntry();
        m_plannerStatusEntry = m_tab.add("Planner Status", m_ultimoErrorPlanificador).withPosition(0, 6).withSize(5, 1).getEntry();
        m_observerTelemetryEntry = m_tab.add("O(1) Kinodynamic Control", "Shock Filter: Idle | F_ext: 0N").withPosition(5, 6).withSize(4, 1).getEntry();


        m_field2d = new Field2d();
        m_tab.add("Field2d", m_field2d).withPosition(0, 1).withSize(8, 5);


        NetworkTableInstance nt = NetworkTableInstance.getDefault();
        m_trueWorldPosePublisher = nt.getStructTopic("SmartDashboard/RobotPose", Pose2d.struct).publish();
        m_localEstimatedPosePublisher = nt.getStructTopic("SmartDashboard/PathPlanner/LocalEstimatedPose", Pose2d.struct).publish();
        m_rawOdometryPosePublisher = nt.getStructTopic("SmartDashboard/PathPlanner/RawWheelOdometryPose", Pose2d.struct).publish();
        m_ghostPosePublisher = nt.getStructTopic("SmartDashboard/PathPlanner/GhostPoseGlobal", Pose2d.struct).publish();
        m_stoppingPointPublisher = nt.getStructTopic("SmartDashboard/PathPlanner/StoppingPointGlobal", Pose2d.struct).publish();
        m_startPosePublisher = nt.getStructTopic("SmartDashboard/PathPlanner/StartPose", Pose2d.struct).publish();
        m_endPosePublisher = nt.getStructTopic("SmartDashboard/PathPlanner/EndPose", Pose2d.struct).publish();
        m_trajectoryPublisher = nt.getStructArrayTopic("SmartDashboard/PathPlanner/Trajectory", Pose2d.struct).publish();
        m_estimatedImpactForcePublisher = nt.getDoubleTopic("SmartDashboard/PathPlanner/EstimatedImpactForce_N").publish();


        m_impactMarkersPublisher = nt.getStructArrayTopic("SmartDashboard/PathPlanner/ImpactMarkers", Pose2d.struct).publish();


        cargarCampo();
        cargarObstaculosDesdeArchivo(m_loadedFieldName);
    }


    @Override
    public void robotPeriodic() {
        comprobarCambioDeMapa();


        Pose2d ghostGlobal = m_follower.convertirLocalAGlobal(m_follower.obtenerGhostPoseLocal(), m_poseInicioGlobalManiobra);
        Pose2d stopGlobal = m_follower.convertirLocalAGlobal(m_follower.obtenerStoppingPointLocal(), m_poseInicioGlobalManiobra);


        if (Double.isFinite(m_trueWorldPose.getX()) && Double.isFinite(m_trueWorldPose.getY())) {
            m_trueWorldPosePublisher.set(m_trueWorldPose);
            m_field2d.setRobotPose(m_trueWorldPose);
        }


        m_localEstimatedPosePublisher.set(m_localEstimatedPose);
        m_rawOdometryPosePublisher.set(m_rawWheelOdometryPose);
        m_ghostPosePublisher.set(ghostGlobal);
        m_stoppingPointPublisher.set(stopGlobal);


        m_field2d.getObject("GhostPose").setPose(ghostGlobal);
        m_field2d.getObject("StoppingPoint").setPose(stopGlobal);


        if (m_field2d != null && m_impactMarkerPoses != null) {
            m_field2d.getObject("ImpactMarkers").setPoses(new ArrayList<>(m_impactMarkerPoses));
        }


        m_plannerStatusEntry.setString(m_ultimoErrorPlanificador);
        m_observerTelemetryEntry.setString(m_controladorO1.obtenerResumenTelemetria());
        m_estimatedImpactForcePublisher.set(m_controladorO1.obtenerUltimaFuerzaMag());


        if (m_joystick.getRawButtonPressed(BUTTON_Z)) {
            m_forwardHeading = m_trueWorldPose.getRotation();
            m_localEstimatedPose = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
            m_rawWheelOdometryPose = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
            m_controladorO1.reiniciar();
        }


        if (m_joystick.getRawButtonPressed(BUTTON_X)) {
            iniciarEmpujonFisicoInterpolado();
        }


        // TOGGLE TECLA C: BUCLE DE PRUEBAS ALEATORIAS
        if (m_joystick.getRawButtonPressed(BUTTON_C)) {
            m_modoPruebasAutomaticas = !m_modoPruebasAutomaticas;
            if (m_modoPruebasAutomaticas) {
                System.out.println("[BENCHMARK] Bucle de pruebas estocásticas aleatorias ACTIVADO.");
                m_ultimoErrorPlanificador = "BUCLE ALEATORIO ACTIVADO: Generando nueva ruta estocástica...";
                ejecutarSiguientePruebaAleatoria();
            } else {
                m_siguiendoRuta = false;
                m_trayectoriaLocalActiva.clear();
                System.out.println("[BENCHMARK] Bucle de pruebas DETENIDO.");
                m_ultimoErrorPlanificador = "Bucle de pruebas detenido por el usuario.";
            }
        }
    }


    @Override public void teleopPeriodic() { procesarCicloFisicoYControl(); }
    @Override public void simulationPeriodic() { procesarCicloFisicoYControl(); }


    // =========================================================================
    // 4. BUCLE DE PRUEBAS ESTOCÁSTICAS ALEATORIAS
    // =========================================================================
    private void ejecutarSiguientePruebaAleatoria() {
        m_siguiendoRuta = false;
        m_trayectoriaLocalActiva.clear();
        m_tiempoUltimoReinicioPrueba = Timer.getFPGATimestamp();


        Random rng = new Random();
        List<EstadoTrayectoria> rutaGlobal = null;
        Pose2d startPose = null;
        Pose2d endPose = null;


        // Generación de rutas aleatorias con rotación de destino arbitraria
        for (int intento = 0; intento < 35; intento++) {
            double x1 = 1.8 + rng.nextDouble() * 2.2;
            double y1 = 1.5 + rng.nextDouble() * 5.0;
            double theta1 = normalizarAngulo((rng.nextDouble() - 0.5) * Math.PI);


            double x2 = 11.5 + rng.nextDouble() * 3.0;
            double y2 = 1.5 + rng.nextDouble() * 5.0;
            // Rotación objetivo aleatoria para verificar la orientación final
            double theta2 = normalizarAngulo((rng.nextDouble() - 0.5) * 2.0 * Math.PI);


            double xMid = (x1 + x2) / 2.0;
            double yMid = (y1 + y2) / 2.0 + (rng.nextDouble() - 0.5) * 3.5;


            Pose2d pStart = new Pose2d(x1, y1, new Rotation2d(theta1));
            Pose2d pMid   = new Pose2d(xMid, yMid, new Rotation2d(theta1));
            Pose2d pEnd   = new Pose2d(x2, y2, new Rotation2d(theta2));


            if (verificarPosicionRobotLibre(pStart.getX(), pStart.getY(), theta1) &&
                verificarPosicionRobotLibre(pEnd.getX(), pEnd.getY(), theta2)) {


                List<EstadoTrayectoria> candidata = obtenerTrayectoriaCompletaM(
                    Arrays.asList(pStart, pMid, pEnd),
                    m_safetyMarginEntry.getDouble(MARGEN_SEGURIDAD_DEFAULT_M), 0.0, null
                );


                if (candidata != null && candidata.size() >= 10) {
                    rutaGlobal = candidata;
                    startPose = pStart;
                    endPose = pEnd;
                    break;
                }
            }
        }


        if (rutaGlobal == null || rutaGlobal.isEmpty()) {
            System.err.println("[PATH ERROR] No se pudo encontrar una ruta aleatoria libre en este intento. Reintentando...");
            ejecutarSiguientePruebaAleatoria();
            return;
        }


        m_puntoInicioGlobal = startPose;
        m_puntoFinGlobal = endPose;
        m_poseInicioGlobalManiobra = startPose;
        m_trueWorldPose = startPose;
        m_localEstimatedPose = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
        m_rawWheelOdometryPose = new Pose2d(0.0, 0.0, new Rotation2d(0.0));


        m_trayectoriaLocalActiva = m_follower.transformarRutaAMarcoLocal(rutaGlobal, m_poseInicioGlobalManiobra);
        publicarTrayectoriaVisual(rutaGlobal);


        m_startPosePublisher.set(startPose);
        m_endPosePublisher.set(endPose);


        m_impactosDisparadosCount = 0;
        m_ultimoProgresoImpactoPct = -100.0;
        m_tiempoRestanteImpacto = 0.0;


        m_impactMarkerPoses.clear();
        if (m_impactMarkersPublisher != null) m_impactMarkersPublisher.set(new Pose2d[0]);
        if (m_field2d != null) m_field2d.getObject("ImpactMarkers").setPoses(Collections.emptyList());


        m_follower.reiniciar();
        m_controladorO1.reiniciar();
        m_siguiendoRuta = true;
        m_ultimoErrorPlanificador = String.format(Locale.US, "Ruta Aleatoria Activa. Destino Heading: %.1f deg", endPose.getRotation().getDegrees());
    }


    private void procesarCicloFisicoYControl() {
        double ahora = Timer.getFPGATimestamp();
        double dtCalculated = ahora - m_ultimoTimestamp;
        m_ultimoTimestamp = ahora;


        if (dtCalculated <= 1e-4 || dtCalculated > 0.05) dtCalculated = 0.02;
        final double dt = dtCalculated;


        double rawForward = m_joystick.getRawAxis(1);
        double rawStrafe = m_joystick.getRawAxis(0);
        double rawTurn = -m_joystick.getRawAxis(4);


        if (Math.abs(rawForward) > 0.08 || Math.abs(rawStrafe) > 0.08 || Math.abs(rawTurn) > 0.08) {
            if (m_siguiendoRuta) {
                m_siguiendoRuta = false;
                m_modoPruebasAutomaticas = false;
                m_ultimoErrorPlanificador = "Interrupción manual por Joystick. Bucle cancelado.";
            }
        }


        double vxCmdLocal = 0.0;
        double vyCmdLocal = 0.0;
        double omegaCmd = 0.0;


        if (m_siguiendoRuta && !m_trayectoriaLocalActiva.isEmpty()) {
            int totalPuntos = m_trayectoriaLocalActiva.size();
            int idxActual = Math.min((int) (m_follower.obtenerTiempoGhost() / 0.05), totalPuntos - 1);
            double progresoActual = ((double) idxActual / Math.max(1, totalPuntos - 1)) * 100.0;


            // Inyección Automática de Choques
            double pctTargetSiguiente = obtenerPctSiguienteChoqueTriada(m_impactosDisparadosCount);
            boolean puedeDisparar = (m_impactosDisparadosCount < 3)
                && (progresoActual >= pctTargetSiguiente)
                && (progresoActual - m_ultimoProgresoImpactoPct >= 4.0)
                && (m_tiempoRestanteImpacto <= 0.0);


            if (puedeDisparar) {
                m_impactosDisparadosCount++;
                m_ultimoProgresoImpactoPct = progresoActual;
                double fuerzaPicoN = m_robotMass * 18.0;
                double anguloImpacto = (m_impactosDisparadosCount - 1) * (2.0 * Math.PI / 3.0);


                m_fuerzaExternaMundoN = new Translation2d(fuerzaPicoN * Math.cos(anguloImpacto), fuerzaPicoN * Math.sin(anguloImpacto));
                m_torqueExternoMundoNm = m_robotMoi * 10.0;
                m_duracionTotalImpacto = 0.28;
                m_tiempoRestanteImpacto = m_duracionTotalImpacto;


                m_impactMarkerPoses.add(m_trueWorldPose);
                if (m_impactMarkersPublisher != null) {
                    m_impactMarkersPublisher.set(m_impactMarkerPoses.toArray(new Pose2d[0]));
                }
                if (m_field2d != null) {
                    m_field2d.getObject("ImpactMarkers").setPoses(new ArrayList<>(m_impactMarkerPoses));
                }


                System.out.println(String.format(Locale.US,
                    "[WORLD PHYSICS] Choque #%d/3 inyectado a %.1f%% en (%.2f, %.2f) m.",
                    m_impactosDisparadosCount, progresoActual, m_trueWorldPose.getX(), m_trueWorldPose.getY()));
            }


            if (m_follower.haLlegadoAlDestino(m_localEstimatedPose, m_trayectoriaLocalActiva)) {
                System.out.println("[BENCHMARK OK] Prueba finalizada exitosamente.");
                if (m_modoPruebasAutomaticas) {
                    ejecutarSiguientePruebaAleatoria();
                } else {
                    m_siguiendoRuta = false;
                    m_ultimoErrorPlanificador = "¡Destino alcanzado exitosamente!";
                }
            } else {
                double[] cmdBase = m_follower.calcularComandoLocal(m_localEstimatedPose, m_trayectoriaLocalActiva, dt);
               
                double[] cmdCorregido = m_controladorO1.ejecutarPasoControlO1(
                    cmdBase, m_corrientesModulos, m_velocidadesModulos,
                    m_vxLastLocal, m_vyLastLocal, dt, ahora
                );


                vxCmdLocal = cmdCorregido[0];
                vyCmdLocal = cmdCorregido[1];
                omegaCmd   = cmdCorregido[2];
            }
        } else {
            double vManualX = Math.max(-1.0, Math.min(1.0, rawForward)) * (m_trueMaxSpeed * 0.65);
            double vManualY = Math.max(-1.0, Math.min(1.0, rawStrafe)) * (m_trueMaxSpeed * 0.65);
            omegaCmd = Math.max(-1.0, Math.min(1.0, rawTurn)) * 3.5;


            Translation2d velField = new Translation2d(vManualX, vManualY).rotateBy(m_localEstimatedPose.getRotation().minus(m_forwardHeading));
            vxCmdLocal = velField.getX();
            vyCmdLocal = velField.getY();
        }


        m_vxLastLocal = vxCmdLocal;
        m_vyLastLocal = vyCmdLocal;


        double thetaInicioManiobra = m_poseInicioGlobalManiobra.getRotation().getRadians();
        double vxCmdField = vxCmdLocal * Math.cos(thetaInicioManiobra) - vyCmdLocal * Math.sin(thetaInicioManiobra);
        double vyCmdField = vxCmdLocal * Math.sin(thetaInicioManiobra) + vyCmdLocal * Math.cos(thetaInicioManiobra);


        ejecutarFisicaDelMundoYSensores(vxCmdField, vyCmdField, omegaCmd, dt, ahora);
    }


    private void ejecutarFisicaDelMundoYSensores(double vxCmdField, double vyCmdField, double omegaCmd, double dt, double ahora) {
        double thetaReal = m_trueWorldPose.getRotation().getRadians();
        double cosT = Math.cos(thetaReal);
        double sinT = Math.sin(thetaReal);


        double fxExtField = 0.0, fyExtField = 0.0, torqueExt = 0.0;
        boolean enImpactoExterno = (m_tiempoRestanteImpacto > 0.0);


        if (enImpactoExterno) {
            double progreso = 1.0 - (m_tiempoRestanteImpacto / m_duracionTotalImpacto);
            double env = Math.sin(Math.PI * Math.max(0.0, Math.min(1.0, progreso)));
            fxExtField = m_fuerzaExternaMundoN.getX() * env;
            fyExtField = m_fuerzaExternaMundoN.getY() * env;
            torqueExt = m_torqueExternoMundoNm * env;
            m_tiempoRestanteImpacto = Math.max(0.0, m_tiempoRestanteImpacto - dt);
        }


        double lambdaCarpeta = Math.exp(-dt / 0.08);
        double axSlipReal = fxExtField / m_robotMass;
        double aySlipReal = fyExtField / m_robotMass;
        double alphaSlipReal = torqueExt / m_robotMoi;


        double vxSlipNuevo = lambdaCarpeta * m_velDeslizamientoRealMundo.getX() + axSlipReal * dt;
        double vySlipNuevo = lambdaCarpeta * m_velDeslizamientoRealMundo.getY() + aySlipReal * dt;
        m_omegaDeslizamientoReal = lambdaCarpeta * m_omegaDeslizamientoReal + alphaSlipReal * dt;


        double dxRuedasField = vxCmdField * dt;
        double dyRuedasField = vyCmdField * dt;
        double dxSlipMundo = 0.5 * (m_velDeslizamientoRealMundo.getX() + vxSlipNuevo) * dt;
        double dySlipMundo = 0.5 * (m_velDeslizamientoRealMundo.getY() + vySlipNuevo) * dt;


        m_velDeslizamientoRealMundo = new Translation2d(vxSlipNuevo, vySlipNuevo);
        double omegaRealMundo = omegaCmd + m_omegaDeslizamientoReal;
        double dThetaReal = omegaRealMundo * dt;


        double nextX = Math.max(0.1, Math.min(CAMPO_LARGO_M - 0.1, m_trueWorldPose.getX() + dxRuedasField + dxSlipMundo));
        double nextY = Math.max(0.1, Math.min(CAMPO_ANCHO_M - 0.1, m_trueWorldPose.getY() + dyRuedasField + dySlipMundo));


        Pose2d candidataMundo = new Pose2d(nextX, nextY, new Rotation2d(normalizarAngulo(thetaReal + dThetaReal)));


        boolean tiempoInmune = (ahora - m_tiempoUltimoReinicioPrueba) < 0.35;


        // Si hay un empujón/impacto externo aplicado, NO detener la prueba como colisión de pared
        if (!m_collisionsEnabledEntry.getBoolean(true) || tiempoInmune || enImpactoExterno || !chocaPoseCompleta(candidataMundo)) {
            m_trueWorldPose = candidataMundo;
        } else {
            m_velDeslizamientoRealMundo = new Translation2d();
            dxRuedasField = 0.0; dyRuedasField = 0.0;
            if (m_siguiendoRuta) {
                System.out.println("[COLLISION] Impacto detectado contra obstáculo de campo.");
                if (m_modoPruebasAutomaticas) {
                    ejecutarSiguientePruebaAleatoria();
                } else {
                    m_siguiendoRuta = false;
                    m_ultimoErrorPlanificador = "COLISIÓN DETECTADA - Detención de seguridad.";
                }
            }
        }


        // Modelo de corrientes de motores
        double fxExtRobot = fxExtField * cosT + fyExtField * sinT;
        double fyExtRobot = -fxExtField * sinT + fyExtField * cosT;
        double vxCmdRobot = vxCmdField * cosT + vyCmdField * sinT;
        double vyCmdRobot = -vxCmdField * sinT + vyCmdField * cosT;


        Translation2d[] posM = {m_frontLeft, m_frontRight, m_backRight, m_backLeft};
        double kt = 7.09 / 483.0, factorF21 = m_wheelRadius / (kt * m_driveGearing * 0.88);


        for (int i = 0; i < 4; i++) {
            double rx = posM[i].getX(), ry = posM[i].getY();
            double vxMod = vxCmdRobot - omegaCmd * ry;
            double vyMod = vyCmdRobot + omegaCmd * rx;
            double vMag = Math.hypot(vxMod, vyMod);
            m_angulosModulos[i] = (vMag > 1e-4) ? Math.atan2(vyMod, vxMod) : 0.0;
            m_velocidadesModulos[i] = vMag;


            double iNom = 2.0 * Math.signum(vMag);
            double fProy = fxExtRobot * Math.cos(m_angulosModulos[i]) + fyExtRobot * Math.sin(m_angulosModulos[i]);
            m_corrientesModulos[i] = Math.abs(iNom - 0.25 * fProy * factorF21);
        }


        double cosInit = Math.cos(m_poseInicioGlobalManiobra.getRotation().getRadians());
        double sinInit = Math.sin(m_poseInicioGlobalManiobra.getRotation().getRadians());


        double dxLocal = dxRuedasField * cosInit + dyRuedasField * sinInit;
        double dyLocal = -dxRuedasField * sinInit + dyRuedasField * cosInit;
        double dThetaLocal = normalizarAngulo(m_trueWorldPose.getRotation().getRadians() - m_poseInicioGlobalManiobra.getRotation().getRadians());


        m_rawWheelOdometryPose = new Pose2d(
            m_rawWheelOdometryPose.getX() + dxRuedasField,
            m_rawWheelOdometryPose.getY() + dyRuedasField,
            new Rotation2d(dThetaLocal)
        );
        m_localEstimatedPose = new Pose2d(
            m_localEstimatedPose.getX() + dxLocal,
            m_localEstimatedPose.getY() + dyLocal,
            new Rotation2d(dThetaLocal)
        );
    }


    // =========================================================================
    // 5. CONTROLADOR CINODINÁMICO O(1) CON RECHAZO DE IMPACTOS
    // =========================================================================
    public static class ControladorCinodinamicoO1 {
        private double m_mass = 74.088;
        private double m_hCG = 0.25;
        private double m_L = 0.70;
        private double m_W = 0.70;
        private double m_wheelRadius = 0.048;
        private double m_gearRatio = 5.143;


        private double[] m_prevTorques = new double[4];
        private double m_ultimaFuerzaMag = 0.0;
        private boolean m_shockDetectado = false;


        public void reiniciar() {
            Arrays.fill(m_prevTorques, 0.0);
            m_ultimaFuerzaMag = 0.0;
            m_shockDetectado = false;
        }


        public double[] ejecutarPasoControlO1(double[] cmdBase, double[] corrientes, double[] velocidades,
                                               double axLocal, double ayLocal, double dt, double tCurrent) {
            double[] torques = new double[4];
            double[] dTorquesDt = new double[4];
            double sumTorques = 0.0;


            for (int i = 0; i < 4; i++) {
                torques[i] = corrientes[i] * 0.018 * m_gearRatio;
                dTorquesDt[i] = (torques[i] - m_prevTorques[i]) / Math.max(0.01, dt);
                m_prevTorques[i] = torques[i];
                sumTorques += torques[i];
            }


            List<Double> tZeros = new ArrayList<>();
            int activeCount = 0;


            for (int i = 0; i < 4; i++) {
                if (Math.abs(dTorquesDt[i]) > 120.0) {
                    activeCount++;
                    double tCalc = tCurrent - (Math.abs(torques[i]) / Math.abs(dTorquesDt[i]));
                    tZeros.add(tCalc);
                } else {
                    tZeros.add(tCurrent);
                }
            }


            double meanT0 = tZeros.stream().mapToDouble(a -> a).average().orElse(tCurrent);
            double varT0 = 0.0;
            for (double t0 : tZeros) varT0 += Math.pow(t0 - meanT0, 2);
            double sigmaT0 = Math.sqrt(varT0 / 4.0);


            m_shockDetectado = (sigmaT0 <= 0.0008) && (activeCount >= 3);


            double nBase = (m_mass * 9.81) / 4.0;
            double dx = (m_mass * axLocal * m_hCG) / (2.0 * m_L);
            double dy = (m_mass * ayLocal * m_hCG) / (2.0 * m_W);


            double[] nForces = new double[]{
                Math.max(10.0, nBase - dx + dy),
                Math.max(10.0, nBase - dx - dy),
                Math.max(10.0, nBase + dx + dy),
                Math.max(10.0, nBase + dx - dy)
            };


            double totalNormal = Arrays.stream(nForces).sum();
            double fExtEst = (sumTorques / m_wheelRadius);
            m_ultimaFuerzaMag = m_shockDetectado ? fExtEst : 0.0;


            // Compensación activa por fuerza normal para absorber el choque sin detener la ruta
            double factorCancelacion = m_shockDetectado ? 0.20 : 0.0;
            double vxCmdCorregido = cmdBase[0] - Math.max(-0.8, Math.min(0.8, (fExtEst / totalNormal) * factorCancelacion));
            double vyCmdCorregido = cmdBase[1];
            double omegaCmdCorregido = cmdBase[2];


            return new double[]{vxCmdCorregido, vyCmdCorregido, omegaCmdCorregido};
        }


        public double obtenerUltimaFuerzaMag() { return m_ultimaFuerzaMag; }
        public String obtenerResumenTelemetry() {
            return String.format(Locale.US, "Shock Filter: %s | F_ext: %.0fN",
                m_shockDetectado ? "SHOCK ABSORBED" : "No Slip (Pass)", m_ultimaFuerzaMag);
        }
        public String obtenerResumenTelemetria() { return obtenerResumenTelemetry(); }
    }


    // =========================================================================
    // 6. FUNCIONES AUXILIARES DE PRUEBAS
    // =========================================================================
    private void iniciarEmpujonFisicoInterpolado() {
        double heading = m_trueWorldPose.getRotation().getRadians() + Math.PI / 2.0;
        double fuerzaPicoN = m_robotMass * 22.0;
        m_fuerzaExternaMundoN = new Translation2d(fuerzaPicoN * Math.cos(heading), fuerzaPicoN * Math.sin(heading));
        m_torqueExternoMundoNm = m_robotMoi * 14.0;
        m_duracionTotalImpacto = 0.28;
        m_tiempoRestanteImpacto = m_duracionTotalImpacto;


        m_impactMarkerPoses.add(m_trueWorldPose);
        if (m_impactMarkersPublisher != null) {
            m_impactMarkersPublisher.set(m_impactMarkerPoses.toArray(new Pose2d[0]));
        }
        if (m_field2d != null) {
            m_field2d.getObject("ImpactMarkers").setPoses(new ArrayList<>(m_impactMarkerPoses));
        }
    }


    private double obtenerPctSiguienteChoqueTriada(int choqueIndex) {
        switch (choqueIndex) {
            case 0: return 15.0;
            case 1: return 50.0;
            case 2: return 80.0;
            default: return 99.0;
        }
    }


    private void publicarTrayectoriaVisual(List<EstadoTrayectoria> rutaGlobal) {
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
    }


    // =========================================================================
    // 7. SEGUIDOR CON INERCIA Y SEGUIMIENTO DE HEADING
    // =========================================================================
    public static class LocalInertiaGhostFollower {
        private int m_ghostIndex = 0;
        private double m_ghostTime = 0.0;
        private double m_aMax = 9.0;
        private double m_alphaMax = 12.0;


        private Pose2d m_ghostPoseLocal = new Pose2d();
        private Pose2d m_stoppingPointLocal = new Pose2d();
        private Translation2d m_velCmdPreviaLocal = new Translation2d();
        private double m_omegaCmdPrevia = 0.0;


        public void reiniciar() {
            m_ghostIndex = 0;
            m_ghostTime = 0.0;
            m_velCmdPreviaLocal = new Translation2d();
            m_omegaCmdPrevia = 0.0;
            m_ghostPoseLocal = new Pose2d();
            m_stoppingPointLocal = new Pose2d();
        }


        public double obtenerTiempoGhost() { return m_ghostTime; }


        public List<EstadoTrayectoria> transformarRutaAMarcoLocal(List<EstadoTrayectoria> rutaGlobal, Pose2d poseInicioGlobal) {
            List<EstadoTrayectoria> rutaLocal = new ArrayList<>(rutaGlobal.size());
            double x0 = poseInicioGlobal.getX();
            double y0 = poseInicioGlobal.getY();
            double theta0 = poseInicioGlobal.getRotation().getRadians();
            double cosT = Math.cos(theta0);
            double sinT = Math.sin(theta0);


            for (EstadoTrayectoria pt : rutaGlobal) {
                double dx = pt.x - x0;
                double dy = pt.y - y0;
                double xLocal = dx * cosT + dy * sinT;
                double yLocal = -dx * sinT + dy * cosT;
                double thetaLocal = resolverAnguloCorto(theta0, pt.theta);
                rutaLocal.add(new EstadoTrayectoria(xLocal, yLocal, thetaLocal, pt.v, pt.t, pt.esCorredor, pt.kCurv));
            }
            return rutaLocal;
        }


        public double[] calcularComandoLocal(Pose2d poseOdometriaLocal, List<EstadoTrayectoria> rutaLocal, double dt) {
            if (rutaLocal == null || rutaLocal.isEmpty() || dt <= 1e-5) return new double[]{0.0, 0.0, 0.0};
            int N = rutaLocal.size();


            double vx = m_velCmdPreviaLocal.getX();
            double vy = m_velCmdPreviaLocal.getY();
            double stopXLocal = poseOdometriaLocal.getX() + (vx * Math.abs(vx)) / (2.0 * m_aMax);
            double stopYLocal = poseOdometriaLocal.getY() + (vy * Math.abs(vy)) / (2.0 * m_aMax);
            double stopThetaLocal = poseOdometriaLocal.getRotation().getRadians() + (m_omegaCmdPrevia * Math.abs(m_omegaCmdPrevia)) / (2.0 * m_alphaMax);


            m_stoppingPointLocal = new Pose2d(stopXLocal, stopYLocal, new Rotation2d(stopThetaLocal));


            EstadoTrayectoria stGhostLocal = rutaLocal.get(m_ghostIndex);
            double eX_Inertia = stGhostLocal.x - stopXLocal;
            double eY_Inertia = stGhostLocal.y - stopYLocal;
            double errorNorm = Math.hypot(eX_Inertia, eY_Inertia);


            double factorClock = Math.max(0.0, 1.0 - 1.5 * errorNorm);
            m_ghostTime += dt * factorClock;


            while (m_ghostIndex < N - 1 && rutaLocal.get(m_ghostIndex + 1).t <= m_ghostTime) {
                m_ghostIndex++;
            }


            stGhostLocal = rutaLocal.get(m_ghostIndex);
            m_ghostPoseLocal = new Pose2d(stGhostLocal.x, stGhostLocal.y, new Rotation2d(stGhostLocal.theta));


            double psiTangenteLocal = stGhostLocal.theta;
            if (m_ghostIndex < N - 1) {
                double dx = rutaLocal.get(m_ghostIndex + 1).x - stGhostLocal.x;
                double dy = rutaLocal.get(m_ghostIndex + 1).y - stGhostLocal.y;
                if (Math.hypot(dx, dy) > 1e-5) psiTangenteLocal = Math.atan2(dy, dx);
            }


            double factorAtenuacionFF = 1.0 / (1.0 + 2.5 * errorNorm);
            double vxFF = stGhostLocal.v * Math.cos(psiTangenteLocal) * factorAtenuacionFF;
            double vyFF = stGhostLocal.v * Math.sin(psiTangenteLocal) * factorAtenuacionFF;


            double vxCmdLocal = vxFF + 2.5 * eX_Inertia;
            double vyCmdLocal = vyFF + 2.5 * eY_Inertia;


            // Control de Orientación (Heading): Seguimiento estricto del perfil theta(t)
            double errorThetaLocal = resolverAnguloCorto(stopThetaLocal, stGhostLocal.theta);
            double omegaCmd = 4.5 * errorThetaLocal;


            m_velCmdPreviaLocal = new Translation2d(vxCmdLocal, vyCmdLocal);
            m_omegaCmdPrevia = omegaCmd;


            return new double[]{vxCmdLocal, vyCmdLocal, omegaCmd};
        }


        public boolean haLlegadoAlDestino(Pose2d poseOdometriaLocal, List<EstadoTrayectoria> rutaLocal) {
            if (rutaLocal == null || rutaLocal.isEmpty()) return true;
            if (m_ghostIndex < rutaLocal.size() - 1) return false;
            EstadoTrayectoria finLocal = rutaLocal.get(rutaLocal.size() - 1);
            double distErr = Math.hypot(finLocal.x - poseOdometriaLocal.getX(), finLocal.y - poseOdometriaLocal.getY());
            double rotErr = Math.abs(resolverAnguloCorto(poseOdometriaLocal.getRotation().getRadians(), finLocal.theta));
            return distErr <= 0.08 && rotErr <= 0.08;
        }


        public Pose2d convertirLocalAGlobal(Pose2d poseLocal, Pose2d poseOrigenGlobal) {
            double x0 = poseOrigenGlobal.getX();
            double y0 = poseOrigenGlobal.getY();
            double theta0 = poseOrigenGlobal.getRotation().getRadians();
            double cosT = Math.cos(theta0);
            double sinT = Math.sin(theta0);


            double xGlobal = x0 + poseLocal.getX() * cosT - poseLocal.getY() * sinT;
            double yGlobal = y0 + poseLocal.getX() * sinT + poseLocal.getY() * cosT;
            double thetaGlobal = theta0 + poseLocal.getRotation().getRadians();
            return new Pose2d(xGlobal, yGlobal, new Rotation2d(thetaGlobal));
        }


        public Pose2d obtenerGhostPoseLocal() { return m_ghostPoseLocal; }
        public Pose2d obtenerStoppingPointLocal() { return m_stoppingPointLocal; }


        private double resolverAnguloCorto(double a0, double a1) {
            double diff = (a1 - a0 + Math.PI) % (2.0 * Math.PI);
            if (diff < 0) diff += 2.0 * Math.PI;
            return diff - Math.PI;
        }
    }


    // =========================================================================
    // 8. MOTOR GEOMÉTRICO E INTERPOLACIÓN CONTINUA DE ROTACIÓN theta(s)
    // =========================================================================
    private List<EstadoTrayectoria> obtenerTrayectoriaCompletaM(List<Pose2d> waypoints, double margenSeguridadM, double velocidadInicialMps, double[][] obstaculoExtra) {
        if (waypoints == null || waypoints.size() < 2) return Collections.emptyList();
        List<double[]> cajasInfladas = crearCajasObstaculosInfladas(true, margenSeguridadM, obstaculoExtra);


        List<double[]> nodosTotales = new ArrayList<>();
        for (int i = 0; i < waypoints.size() - 1; i++) {
            double[] ini = {waypoints.get(i).getX(), waypoints.get(i).getY()};
            double[] fin = {waypoints.get(i + 1).getX(), waypoints.get(i + 1).getY()};
            List<double[]> tramo = planificarRutaOrtogonalM(ini, fin, cajasInfladas);
            if (tramo.isEmpty()) return Collections.emptyList();
            if (nodosTotales.isEmpty()) nodosTotales.addAll(tramo);
            else for (int k = 1; k < tramo.size(); k++) nodosTotales.add(tramo.get(k));
        }


        nodosTotales = simplificarCamino(nodosTotales);
        List<double[]> nodosFiltrados = eliminarMicroEscalones(nodosTotales, cajasInfladas);
        List<double[]> posicionesSuaves = suavizarCaminoOrtogonalM(nodosFiltrados, cajasInfladas);
        List<EstadoTrayectoria> perfilada = perfilamientoFisicoON(posicionesSuaves, waypoints, cajasInfladas, velocidadInicialMps);


        for (EstadoTrayectoria st : perfilada) {
            if (!verificarPosicionRobotLibre(st.x, st.y, st.theta)) {
                return Collections.emptyList();
            }
        }
        return perfilada;
    }


    private List<double[]> suavizarCaminoOrtogonalM(List<double[]> camino, List<double[]> cajas) {
        if (camino.size() < 3) return camino;
        List<double[]> salida = new ArrayList<>();
        salida.add(camino.get(0));


        for (int i = 1; i < camino.size() - 1; i++) {
            double[] a = camino.get(i - 1), b = camino.get(i), c = camino.get(i + 1);
            double lenIn = dist(a, b), lenOut = dist(b, c);
            if (lenIn < 1e-5 || lenOut < 1e-5) continue;


            double[] uIn = {(b[0] - a[0]) / lenIn, (b[1] - a[1]) / lenIn};
            double[] uOut = {(c[0] - b[0]) / lenOut, (c[1] - b[1]) / lenOut};


            double dIn = Math.min(RADIO_CURVA_MAX_M, lenIn * 0.40);
            double dOut = Math.min(RADIO_CURVA_MAX_M, lenOut * 0.40);


            double[] entrada = {b[0] - uIn[0] * dIn, b[1] - uIn[1] * dIn};
            double[] salidaCurva = {b[0] + uOut[0] * dOut, b[1] + uOut[1] * dOut};


            List<double[]> curva = puntosBezierCuadratica(entrada, b, salidaCurva);
            salida.addAll(curva);
        }


        salida.add(camino.get(camino.size() - 1));
        return salida;
    }


    private List<double[]> puntosBezierCuadratica(double[] p0, double[] p1, double[] p2) {
        List<double[]> pts = new ArrayList<>();
        int n = 10;
        for (int i = 0; i <= n; i++) {
            double t = (double) i / n;
            double u = 1.0 - t;
            double x = u * u * p0[0] + 2 * u * t * p1[0] + t * t * p2[0];
            double y = u * u * p0[1] + 2 * u * t * p1[1] + t * t * p2[1];
            pts.add(new double[]{x, y});
        }
        return pts;
    }


    private List<EstadoTrayectoria> perfilamientoFisicoON(List<double[]> trayectoriaPosM, List<Pose2d> waypoints, List<double[]> cajas, double vIni) {
        List<EstadoTrayectoria> resultado = new ArrayList<>();
        int N = trayectoriaPosM.size();
        double[] distAcum = new double[N];
        distAcum[0] = 0.0;
        for (int i = 1; i < N; i++) {
            distAcum[i] = distAcum[i - 1] + dist(trayectoriaPosM.get(i - 1), trayectoriaPosM.get(i));
        }


        double totalDist = distAcum[N - 1];
        double thetaStart = waypoints.get(0).getRotation().getRadians();
        double thetaEnd = waypoints.get(waypoints.size() - 1).getRotation().getRadians();


        // Calculador de diferencia más corta de ángulos
        double deltaTheta = (thetaEnd - thetaStart + Math.PI) % (2.0 * Math.PI);
        if (deltaTheta < 0) deltaTheta += 2.0 * Math.PI;
        deltaTheta -= Math.PI;


        double vMax = 2.8;
        double aMax = 4.0;
        double[] vFwd = new double[N];
        vFwd[0] = Math.max(0.1, vIni);


        for (int i = 1; i < N; i++) {
            double ds = distAcum[i] - distAcum[i - 1];
            double kCurv = 0.0;
            if (i > 0 && i < N - 1) {
                double[] p1 = trayectoriaPosM.get(i - 1), p2 = trayectoriaPosM.get(i), p3 = trayectoriaPosM.get(i + 1);
                double d1 = dist(p1, p2), d2 = dist(p2, p3);
                if (d1 > 1e-4 && d2 > 1e-4) {
                    double cross = (p2[0] - p1[0]) * (p3[1] - p2[1]) - (p2[1] - p1[1]) * (p3[0] - p2[0]);
                    kCurv = Math.abs(2.0 * cross / (d1 * d2 * dist(p1, p3) + 1e-6));
                }
            }
            double vLimCurva = (kCurv > 0.1) ? Math.min(vMax, Math.sqrt((0.95 * 9.81) / kCurv)) : vMax;
            vFwd[i] = Math.min(vLimCurva, Math.sqrt(vFwd[i - 1] * vFwd[i - 1] + 2.0 * aMax * ds));
        }


        vFwd[N - 1] = 0.0;
        for (int i = N - 2; i >= 0; i--) {
            double ds = distAcum[i + 1] - distAcum[i];
            vFwd[i] = Math.min(vFwd[i], Math.sqrt(vFwd[i + 1] * vFwd[i + 1] + 2.0 * aMax * ds));
        }


        double t = 0.0;
        for (int i = 0; i < N; i++) {
            double[] pt = trayectoriaPosM.get(i);
           
            // Interpolación de rotación theta(s) proporcional al avance longitudinal s
            double sFrac = (totalDist > 1e-6) ? (distAcum[i] / totalDist) : 0.0;
            double targetTheta = normalizarAngulo(thetaStart + sFrac * deltaTheta);


            if (i > 0) {
                double ds = distAcum[i] - distAcum[i - 1];
                double vMed = Math.max(0.1, (vFwd[i - 1] + vFwd[i]) / 2.0);
                t += ds / vMed;
            }
            resultado.add(new EstadoTrayectoria(pt[0], pt[1], targetTheta, vFwd[i], t));
        }
        return resultado;
    }


    private boolean verificarPosicionRobotLibre(double x, double y, double theta) {
        Pose2d pose = new Pose2d(x, y, new Rotation2d(theta));
        if (m_collisionsEnabledEntry != null && !m_collisionsEnabledEntry.getBoolean(true)) return true;


        List<double[]> cajasConBuffer = expandirCajas(m_cajasObstaculosReales, MARGEN_ESTRICTO_COLISION_M);
        Translation2d[] esquinas = getCorners(pose);
        double[][] pts = new double[4][2];
        for (int i = 0; i < 4; i++) {
            pts[i][0] = esquinas[i].getX(); pts[i][1] = esquinas[i].getY();
        }


        for (double[] c : cajasConBuffer) {
            if (puntoEnCaja(pts[0], c, 1e-9) || puntoEnCaja(pts[1], c, 1e-9) ||
                puntoEnCaja(pts[2], c, 1e-9) || puntoEnCaja(pts[3], c, 1e-9)) return false;
        }
        for (int j = 0; j < 4; j++) {
            if (!segmentoLibre(pts[j], pts[(j + 1) % 4], cajasConBuffer)) return false;
        }


        return !chocaPoseCompleta(pose);
    }


    private boolean chocaPoseCompleta(Pose2d pose) {
        if (pose == null || m_obstaculos.isEmpty()) return false;
        Translation2d[] esquinas = getCorners(pose);
        double[][] robotPoints = new double[4][2];
        for (int i = 0; i < 4; i++) {
            robotPoints[i][0] = esquinas[i].getX();
            robotPoints[i][1] = esquinas[i].getY();
        }


        int[][] lados = {{0, 1}, {1, 2}, {2, 3}, {3, 0}};
        int[][] diagonales = {{0, 2}, {1, 3}};


        for (double[][] obstaculo : m_obstaculos) {
            if (obstaculo == null || obstaculo.length < 2) continue;
            double[] p1 = obstaculo[0], p2 = obstaculo[1];
            for (int[] lado : lados) {
                if (segmentosIntersectan(robotPoints[lado[0]], robotPoints[lado[1]], p1, p2)) return true;
            }
            for (int[] diagonal : diagonales) {
                if (segmentosIntersectan(robotPoints[diagonal[0]], robotPoints[diagonal[1]], p1, p2)) return true;
            }
        }
        return false;
    }


    private Translation2d[] getCorners(Pose2d pose) {
        Translation2d[] relativos = {m_frontLeft, m_frontRight, m_backRight, m_backLeft};
        Translation2d[] absolutos = new Translation2d[relativos.length];
        for (int i = 0; i < relativos.length; i++) {
            absolutos[i] = relativos[i].rotateBy(pose.getRotation()).plus(pose.getTranslation());
        }
        return absolutos;
    }


    private boolean segmentosIntersectan(double[] a, double[] b, double[] c, double[] d) {
        int o1 = orientacionCCW(a, b, c), o2 = orientacionCCW(a, b, d);
        int o3 = orientacionCCW(c, d, a), o4 = orientacionCCW(c, d, b);
        if (o1 != o2 && o3 != o4) return true;
        if (o1 == 0 && enSegmento(a, b, c)) return true;
        if (o2 == 0 && enSegmento(a, b, d)) return true;
        if (o3 == 0 && enSegmento(c, d, a)) return true;
        return o4 == 0 && enSegmento(c, d, b);
    }


    private int orientacionCCW(double[] a, double[] b, double[] c) {
        double val = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
        if (Math.abs(val) <= COLLISION_EPSILON) return 0;
        return val > 0.0 ? 1 : -1;
    }


    private boolean enSegmento(double[] a, double[] b, double[] p) {
        return p[0] >= Math.min(a[0], b[0]) - COLLISION_EPSILON && p[0] <= Math.max(a[0], b[0]) + COLLISION_EPSILON &&
               p[1] >= Math.min(a[1], b[1]) - COLLISION_EPSILON && p[1] <= Math.max(a[1], b[1]) + COLLISION_EPSILON;
    }


    private List<double[]> planificarRutaOrtogonalM(double[] inicio, double[] fin, List<double[]> cajas) {
        double[] limites = {0.01, CAMPO_LARGO_M - 0.01, 0.01, CAMPO_ANCHO_M - 0.01};
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
                for (double[] c : cajas) if (puntoEnCaja(p, c, 1e-9)) { enObstaculo = true; break; }
                if (puntoEnCampo(p, limites) && !enObstaculo) {
                    int idx = nodos.size(); nodos.add(p);
                    mapaIndice.put(x + "," + y, idx);
                    filas.computeIfAbsent(y, k -> new ArrayList<>()).add(idx);
                    columnas.computeIfAbsent(x, k -> new ArrayList<>()).add(idx);
                }
            }
        }


        double[] iniRound = {round7(inicio[0]), round7(inicio[1])};
        double[] finRound = {round7(fin[0]), round7(fin[1])};


        int idxIni = mapaIndice.computeIfAbsent(iniRound[0] + "," + iniRound[1], k -> {
            int id = nodos.size(); nodos.add(iniRound);
            filas.computeIfAbsent(iniRound[1], key -> new ArrayList<>()).add(id);
            columnas.computeIfAbsent(iniRound[0], key -> new ArrayList<>()).add(id);
            return id;
        });


        int idxFin = mapaIndice.computeIfAbsent(finRound[0] + "," + finRound[1], k -> {
            int id = nodos.size(); nodos.add(finRound);
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
                    ady.get(u).add(new double[]{v, peso}); ady.get(v).add(new double[]{u, peso});
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
                    ady.get(u).add(new double[]{v, peso}); ady.get(v).add(new double[]{u, peso});
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
            double costeActual = top[0]; int actual = (int) top[1];
            if (costeActual > distancias[actual] + 1e-10) continue;
            if (actual == idxFin) break;
            for (double[] arista : ady.get(actual)) {
                int vecino = (int) arista[0]; double peso = arista[1];
                double nuevo = costeActual + peso;
                if (nuevo + 1e-10 < distancias[vecino]) {
                    distancias[vecino] = nuevo; anterior[vecino] = actual;
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
        return simplificarCamino(camino);
    }


    private List<double[]> crearCajasObstaculosInfladas(boolean inflar, double margenSeguridadM, double[][] obstaculoExtra) {
        double radio = inflar ? (Math.hypot(m_robotWidth, m_robotLength) / 2.0 + margenSeguridadM) : 0.001;
        List<double[]> resultado = new ArrayList<>(m_cajasObstaculosReales.size() + 1);
        for (double[] c : m_cajasObstaculosReales) {
            resultado.add(new double[]{c[0] - radio, c[1] + radio, c[2] - radio, c[3] + radio});
        }
        return resultado;
    }


    private List<double[]> expandirCajas(List<double[]> cajas, double margen) {
        List<double[]> exp = new ArrayList<>(cajas.size());
        for (double[] c : cajas) exp.add(new double[]{c[0] - margen, c[1] + margen, c[2] - margen, c[3] + margen});
        return exp;
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
        return (limites[0] - 1e-8 <= p[0] && p[0] <= limites[1] + 1e-8 && limites[2] - 1e-8 <= p[1] && p[1] <= limites[3] + 1e-8);
    }


    private double dist(double[] a, double[] b) { return Math.hypot(b[0] - a[0], b[1] - a[1]); }
    private double round7(double val) { return Math.round(val * 1e7) / 1e7; }


    private List<double[]> simplificarCamino(List<double[]> camino) {
        if (camino == null || camino.isEmpty()) return new ArrayList<>();
        List<double[]> limpio = new ArrayList<>();
        for (double[] p : camino) {
            if (limpio.isEmpty() || dist(p, limpio.get(limpio.size() - 1)) > 1e-7) limpio.add(new double[]{p[0], p[1]});
        }
        return limpio;
    }


    private List<double[]> eliminarMicroEscalones(List<double[]> camino, List<double[]> cajas) {
        if (camino.size() < 4) return camino;
        List<double[]> resultado = new ArrayList<>();
        resultado.add(camino.get(0));
        int i = 1, n = camino.size();


        while (i < n) {
            double[] A = resultado.get(resultado.size() - 1);
            double[] B = camino.get(i);
            if (i + 1 < n) {
                double[] C = camino.get(i + 1);
                if (dist(B, C) < UMBRAL_MICRO_ESCALON_M && segmentoLibre(A, C, cajas)) {
                    resultado.add(C); i += 2; continue;
                }
            }
            resultado.add(B); i++;
        }
        return simplificarCamino(resultado);
    }


    // =========================================================================
    // 9. CARGA DE MAPAS Y CONFIGURACIÓN
    // =========================================================================
    private void actualizarDimensionesHitbox() {
        m_frontLeft = new Translation2d(m_robotLength / 2.0, m_robotWidth / 2.0);
        m_frontRight = new Translation2d(m_robotLength / 2.0, -m_robotWidth / 2.0);
        m_backRight = new Translation2d(-m_robotLength / 2.0, -m_robotWidth / 2.0);
        m_backLeft = new Translation2d(-m_robotLength / 2.0, m_robotWidth / 2.0);
    }


    private void cargarObstaculosDesdeArchivo(String campoSeleccionado) {
        m_obstaculos.clear();
        m_cajasObstaculosReales.clear();


        File archivo = new File(m_obstaclesFilePath);
        if (!archivo.exists() || !archivo.isFile()) archivo = new File(DEFAULT_OBSTACLES_PATH);
        if (!archivo.exists() || !archivo.isFile()) return;


        StringBuilder contenido = new StringBuilder();
        try (BufferedReader reader = new BufferedReader(new FileReader(archivo))) {
            String linea;
            while ((linea = reader.readLine()) != null) contenido.append(linea).append('\n');
        } catch (IOException e) { return; }


        String numero = "([-+]?(?:\\d+(?:\\.\\d*)?|\\.\\d+)(?:[eE][+]?\\d+)?)";
        Pattern bloquePattern = Pattern.compile("/\\*\\s*OBSTACLE\\s*\\*/(.*?)/\\*\\s*END_OBSTACLE\\s*\\*/", Pattern.DOTALL | Pattern.CASE_INSENSITIVE);
        Pattern segmentoPattern = Pattern.compile("\\[\\s*" + numero + "\\s*, \\s*" + numero +
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
                    cajaMinX = Math.min(cajaMinX, x0); cajaMaxX = Math.max(cajaMaxX, x1);
                    cajaMinY = Math.min(cajaMinY, y0); cajaMaxY = Math.max(cajaMaxY, y1);
                    grupoValido = true;
                }
            }
            if (grupoValido) {
                m_cajasObstaculosReales.add(new double[]{cajaMinX, cajaMaxX, cajaMinY, cajaMaxY});
            }
        }
        System.out.println("[MAPA OK] Obstáculos cargados: " + m_cajasObstaculosReales.size());
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


    private void comprobarCambioDeMapa() {
        if (m_fieldEntry == null) return;
        String campoSeleccionado = m_fieldEntry.getString(DEFAULT_FIELD);
        if (m_loadedFieldName == null || !campoSeleccionado.equals(m_loadedFieldName)) {
            cargarCampo();
            cargarObstaculosDesdeArchivo(m_loadedFieldName);
        }
    }


    private double normalizarAngulo(double angulo) {
        while (angulo > Math.PI) angulo -= 2.0 * Math.PI;
        while (angulo < -Math.PI) angulo += 2.0 * Math.PI;
        return angulo;
    }
}

