package gay.zharel.fastlane

import com.acmerobotics.dashboard.config.Config
import dev.nextftc.units.Volts
import dev.nextftc.units.unittypes.VoltageUnit

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import dev.nextftc.control.geometry.Vector2d
import dev.nextftc.units.Measure
import org.firstinspires.ftc.teamcode.VoltageConstants.voltageCeil
import org.firstinspires.ftc.teamcode.VoltageConstants.voltageFloor
import gay.zharel.fateweaver.telemetry.Fate
import gay.zharel.fateweaver.telemetry.log
import org.firstinspires.ftc.teamcode.VoltageConstants

@Config
/**
 * Global object for synchronized and IO-efficient voltage reads.
 */
object VoltageCache {

    private lateinit var voltageSensor: VoltageSensor
    private var cachedVoltage: Double = 0.0
    private val timeElapsed: ElapsedTime = ElapsedTime()

    val currentVoltage: Double
        get() {
            if (VoltageConstants.voltageReadSeconds > 0 && timeElapsed.seconds() > VoltageConstants.voltageCeil) {
                val trueVoltage = voltageSensor.voltage
                Fate.log("Voltage/TrueVoltage", trueVoltage);
                cachedVoltage = Range.clip(trueVoltage, voltageFloor, voltageCeil)
                Fate.log("Voltage/Voltage", cachedVoltage);
                timeElapsed.reset()
            }
            return cachedVoltage
        }

    /**
     * Initializes the VoltageCache with the voltage sensor for the OpMode.
     */
    @JvmStatic
    fun init(hardwareMap: HardwareMap) {
        this.voltageSensor = hardwareMap.voltageSensor.iterator().next()
        cachedVoltage = voltageSensor.voltage
        timeElapsed.reset()
    }

}

// to robot battery
@JvmField
val AppliedThrottle = VoltageUnit(Volts,
    {throttle: Double -> throttle * VoltageCache.currentVoltage},
    {volt: Double -> volt / VoltageCache.currentVoltage},
    "applied-throttle", "thr")

// normalized, do not clamp!
@JvmField
val Throttle = VoltageUnit(Volts, VoltageConstants.nominalVoltage,
    "throttle", "thrN")

data class PoseVoltage2d(
    @JvmField val linearVolt: Vector2d<VoltageUnit>,
    @JvmField val angVolt: Measure<VoltageUnit>,
)