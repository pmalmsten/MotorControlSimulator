import kotlin.math.pow

class AngularPose(val positionDegrees: Double,
                  val velocityDegPerS: Double,
                  val accelerationDegPerS2: Double,
                  val jerkDegPerS3: Double) {
    constructor() : this(0.0,0.0,0.0,0.0)
}

class DCPermanentMagnetMotor(val maxTorqueNM: Double,
                             val maxVelDegPerS: Double) {

    fun computeTorque(currentVelRPM: Double,
                      currentDirectionalDutyCyclePercent: Double): Double {
        val percentMaxTorque = 1 - (currentVelRPM / maxVelDegPerS)

        return percentMaxTorque * currentDirectionalDutyCyclePercent
    }
}

class RotationalPointMassPose(
    val massKg: Double,
    val radiusM: Double,
    val pose: AngularPose) {

    private val pointMassAngularInertia = massKg*(radiusM.pow(2))

    fun updatePose(appliedTorqueNM: Double,
                   elapsedTimeS: Double): RotationalPointMassPose {

        val angularDeltaPositionDeg = pose.velocityDegPerS * elapsedTimeS
        val angularDeltaVDegPerS = pose.accelerationDegPerS2 * elapsedTimeS

        val currentAngularAccelDegPerS2 = appliedTorqueNM / pointMassAngularInertia

        val newPose = AngularPose(
            positionDegrees = pose.positionDegrees + angularDeltaPositionDeg,
            velocityDegPerS = pose.velocityDegPerS + angularDeltaVDegPerS,
            accelerationDegPerS2 = currentAngularAccelDegPerS2,
            jerkDegPerS3 = (currentAngularAccelDegPerS2 - pose.accelerationDegPerS2) / elapsedTimeS
        )

        return RotationalPointMassPose(massKg, radiusM, newPose)
    }
}

class DCPermanentMagnetMotorDirectToBalancedMass(val motor: DCPermanentMagnetMotor,
                                                 val massPose: RotationalPointMassPose) {

}