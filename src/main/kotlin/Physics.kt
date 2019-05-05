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

    fun updatePose(currentTorqueNMAtVelocityDegPerS: (currentVelocityDegPerS: Double) -> Double,
                   elapsedTimeS: Double): RotationalPointMassPose {

        val currentVelocityDegPerS = pose.velocityDegPerS + deltaVelocityDegPerS(elapsedTimeS)

        val currentAngularAccelDegPerS2 =
            currentTorqueNMAtVelocityDegPerS(currentVelocityDegPerS) / pointMassAngularInertia

        val newPose = AngularPose(
            positionDegrees = pose.positionDegrees + deltaPositionDeg(elapsedTimeS),
            velocityDegPerS = currentVelocityDegPerS,
            accelerationDegPerS2 = currentAngularAccelDegPerS2,
            jerkDegPerS3 = (currentAngularAccelDegPerS2 - pose.accelerationDegPerS2) / elapsedTimeS
        )

        return RotationalPointMassPose(massKg, radiusM, newPose)
    }

    private fun deltaPositionDeg(elapsedTimeS: Double): Double {
        return pose.velocityDegPerS * elapsedTimeS
    }

    private fun deltaVelocityDegPerS(elapsedTimeS: Double): Double {
        return pose.accelerationDegPerS2 * elapsedTimeS
    }
}

class DCPermanentMagnetMotorDirectToBalancedMass(val motor: DCPermanentMagnetMotor,
                                                 val massPose: RotationalPointMassPose) {

}