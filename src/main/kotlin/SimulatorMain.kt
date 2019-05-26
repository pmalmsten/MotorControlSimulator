import java.time.LocalDateTime
import java.time.format.DateTimeFormatter
import java.util.concurrent.TimeUnit

val SIMULATION_DURATION_NANOS = TimeUnit.SECONDS.toNanos(6)
val SIMULATION_TICK_NANOS = TimeUnit.MILLISECONDS.toNanos(2)

enum class CSVColumn(val fieldName: String) {
    SIMULATION_TIMESTAMP_SECONDS("Simulation Timestamp (s)"),

    POSITION_RAD("Position (Rad)"),
    VELOCITY_RAD_PER_S("Velocity (Rad/S)"),
    ACCEL_RAD_PER_S2("Acceleration (Rad/S^2)"),
    NET_TORQUE_NM("Net Torque (NM)"),
    JERK_RAD_PER_S3("Jerk (Rad/S^3)"),
    MOTOR_DUTY_CYCLE("Motor Duty Cycle (%)"),
    SETPOINT_POS_RAD("Setpoint Position (Rad)"),
    PID_EVENT_TRIGGERED("PID Event Triggered (Bool)")
}

val CSV_COLUMNS = CSVColumn.values().map { it.fieldName }

fun main() {
    val currentTimeFormatted = DateTimeFormatter.ISO_DATE_TIME.format(LocalDateTime.now())

    CSVWriter("simulation_$currentTimeFormatted.csv", CSV_COLUMNS).use {
        runSimulation(it)
    }
}

const val kP = 0.0002

fun runSimulation(csv: CSVWriter) {
    val initialPose = RotationalPointMassPose(0.3, 0.01, AngularPose())
    val motor = DCPermanentMagnetMotor(0.3, 2000.0)

    var motorMassPose = MotorAndRotationalPointMassPose(motor, initialPose)

    var setpointPositionRad = initialPose.pose.positionRad
    var dutyCycle = 0.0

    val pidTimer = PeriodicEvent(0.020) {
        val error = setpointPositionRad - motorMassPose.pose.positionRad
        val outputDutyCycle = (error * kP).coerceIn(-1.0, 1.0)

        PIDUpdate(outputDutyCycle)
    }
    pidTimer.handleSimulationTick(0)

    for (currentTimeNanos in SIMULATION_TICK_NANOS..SIMULATION_DURATION_NANOS step SIMULATION_TICK_NANOS) {
        val elapsedTimeS = SIMULATION_TICK_NANOS / TimeUnit.SECONDS.toNanos(1).toDouble()
        val currentTimeS = currentTimeNanos / TimeUnit.SECONDS.toNanos(1).toDouble()

        setpointPositionRad = simulatedSetpointPositionRadAtTime(currentTimeS)

        val pidUpdate = pidTimer.handleSimulationTick(currentTimeNanos)
        if (pidUpdate != null) {
            dutyCycle = pidUpdate.dutyCycle
        }

        val updatePoseResult = motorMassPose.updatePose(dutyCycle, elapsedTimeS)
        motorMassPose = updatePoseResult.pose

        csv.writeRecord(mapOf(
            CSVColumn.SIMULATION_TIMESTAMP_SECONDS.fieldName to
                    currentTimeS,

            CSVColumn.POSITION_RAD.fieldName to motorMassPose.pose.positionRad,
            CSVColumn.VELOCITY_RAD_PER_S.fieldName to motorMassPose.pose.velocityRadPerS,
            CSVColumn.NET_TORQUE_NM.fieldName to updatePoseResult.computedTorqueNM,
            CSVColumn.ACCEL_RAD_PER_S2.fieldName to motorMassPose.pose.accelerationRadPerS2,
            CSVColumn.JERK_RAD_PER_S3.fieldName to motorMassPose.pose.jerkRadPerS3,
            CSVColumn.MOTOR_DUTY_CYCLE.fieldName to dutyCycle,
            CSVColumn.SETPOINT_POS_RAD.fieldName to setpointPositionRad,
            CSVColumn.PID_EVENT_TRIGGERED.fieldName to if (pidUpdate != null) 1.0 else 0.0
        ))
    }
}

fun simulatedSetpointPositionRadAtTime(currentTimeS: Double): Double {
    return if (currentTimeS >= 1)
        3000.0
    else
        0.0
}

data class PIDUpdate(val dutyCycle: Double)