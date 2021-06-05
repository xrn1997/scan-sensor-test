package edu.ysu.sensor.handler

import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.util.Log


/**
 * 步态检测类，用来检测是否走了一步。
 * @author xrn1997
 * @date 2021/6/1
 */
class StepDetectionHandler(
    private var sensorManager: SensorManager
) : SensorEventListener {

    private var mStepDetectionListener: StepDetectionListener? = null

    var sensor: Sensor? = null

    var step = 0


    override fun onSensorChanged(event: SensorEvent) {
        if (event.sensor.type == Sensor.TYPE_LINEAR_ACCELERATION) {
            val y = event.values[1]
            if (y > 1 && mStepDetectionListener != null) {
                onNewStepDetected()
            }
        }
    }


    private fun onNewStepDetected() {
        val distanceStep = 0.8F
        step++
        mStepDetectionListener?.newStep(distanceStep)
    }

    fun setStepListener(listener: StepDetectionListener) {
        mStepDetectionListener = listener
    }

    fun start() {
        sensorManager.registerListener(this, sensor, SensorManager.SENSOR_DELAY_NORMAL)
    }

    fun stop() {
        sensorManager.unregisterListener(this)
    }

    interface StepDetectionListener {
        fun newStep(stepSize: Float)
    }

    init {
        sensor = sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION)
    }

    override fun onAccuracyChanged(sensor: Sensor, accuracy: Int) {}
}