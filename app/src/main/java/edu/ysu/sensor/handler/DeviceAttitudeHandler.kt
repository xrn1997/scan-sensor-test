package edu.ysu.sensor.handler

import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.util.Log
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Job
import kotlinx.coroutines.launch
import kotlin.math.atan2


/**
 * 航向检测类，用来检测行人走动的方向
 * @author xrn1997
 * @date 2021/6/2
 */
class DeviceAttitudeHandler(
    var sensorManager: SensorManager
) : SensorEventListener {

    private var magneticField: Sensor? = null
    private var accelerometer: Sensor? = null

    private val accelerometerReading = FloatArray(3)
    private val magnetometerReading = FloatArray(3)

    private var timestamp: Float = 0f

    val rotationMatrix = FloatArray(9)
    val orientationAngles = FloatArray(3)
    
    private val job = Job()
    override fun onSensorChanged(event: SensorEvent) {
        val scope= CoroutineScope(job)
        if (timestamp != 0f) {
            val dT = (event.timestamp - timestamp) * NS2S
            scope.launch {
                Log.e("数据", dT.toString() )
            }
        }
        timestamp = event.timestamp.toFloat()

        if (event.sensor.type == Sensor.TYPE_ACCELEROMETER) {
            System.arraycopy(event.values, 0, accelerometerReading, 0, accelerometerReading.size)
        } else if (event.sensor.type == Sensor.TYPE_MAGNETIC_FIELD) {
            System.arraycopy(event.values, 0, magnetometerReading, 0, magnetometerReading.size)
        }
        updateOrientationAngles()
    }

    fun start() {
        // 为加速度传感器注册监听器
        sensorManager.registerListener(this, accelerometer, SensorManager.SENSOR_DELAY_NORMAL)
        // 为磁场传感器注册监听器
        sensorManager.registerListener(this, magneticField, SensorManager.SENSOR_DELAY_NORMAL)
    }

    fun stop() {
        job.cancel()
        sensorManager.unregisterListener(this)
    }

    init {
        accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        magneticField = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD)
    }

    private fun updateOrientationAngles() {
        SensorManager.getRotationMatrix(
            rotationMatrix,
            null,
            accelerometerReading,
            magnetometerReading
        )
        SensorManager.getOrientation(rotationMatrix, orientationAngles)
        Log.e(
            "对比",
            "${atan2(rotationMatrix[3], rotationMatrix[4]) / Math.PI * 180}    " +
                    "    ${orientationAngles[0] / Math.PI * 180}   " +
                    "MX${rotationMatrix[3]},MY${rotationMatrix[4]}" +
                    " AX${rotationMatrix[6]}  AY${rotationMatrix[7]}  AZ${rotationMatrix[8]} " +
                    "${atan2(rotationMatrix[6], rotationMatrix[8]) / Math.PI * 180}" + "" +
                    "${orientationAngles[2] / Math.PI * 180} "
        )
    }

    companion object {
        private const val NS2S = 1.0f / 1000000000.0f
    }

    override fun onAccuracyChanged(sensor: Sensor, accuracy: Int) {}
}