import java.util.concurrent.TimeUnit

class PeriodicEvent<T>(
    periodSeconds: Double,
    private val event: () -> T
) {

    private val periodNanos = TimeUnit.SECONDS.toNanos(1) * periodSeconds

    var lastRunTimeNanos: Long? = null

    fun handleSimulationTick(currentTimeNanos: Long): T? {
        val lastRunTimeNanosCopy = lastRunTimeNanos

        return if (lastRunTimeNanosCopy == null || currentTimeNanos - lastRunTimeNanosCopy >= periodNanos) {
            val result = event()
            lastRunTimeNanos = currentTimeNanos

            result
        } else null
    }
}