package edu.ysu.sensor.handler

import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager


/**
 * 航向检测类，用来检测行人走动的方向
 * @author xrn1997
 * @date 2021/6/2
 */
class DeviceAttitudeHandler(
    private var sensorManager:SensorManager
) : SensorEventListener {

    var sensor: Sensor? = null

   val orientationValues = ArrayList<Float>(3)

    override fun onSensorChanged(event: SensorEvent) {
        orientationValues[0]=event.values[0]
        orientationValues[1]=event.values[1]
        orientationValues[2]=event.values[2]
    }
    fun start() {
        sensorManager.registerListener(this, sensor, SensorManager.SENSOR_DELAY_NORMAL)
    }

    fun stop() {
        sensorManager.unregisterListener(this)
    }

    init {
        sensor=sensorManager.getDefaultSensor(SENSOR_TYPE)
    }

    companion object {
        private const val SENSOR_TYPE = Sensor.TYPE_ORIENTATION
    }

    override fun onAccuracyChanged(sensor: Sensor, accuracy: Int) {}
}