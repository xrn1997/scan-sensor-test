package edu.ysu.sensor.handler

import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import androidx.lifecycle.Lifecycle
import androidx.lifecycle.LifecycleObserver
import androidx.lifecycle.OnLifecycleEvent
import kotlin.math.sqrt


/**
 * 步态检测类，用来检测是否走了一步。开发中。
 * @author xrn1997
 * @date 2021/6/1
 */
@Suppress("unused")
class StepDetectionHandler(
    private var sensorManager: SensorManager
) : SensorEventListener, LifecycleObserver {

    private var mStepDetectionListener: StepDetectionListener? = null

    var sensor: Sensor? = null

    var step = 0

    /**
     * 存放三轴数据
     */
    private val oriValues = FloatArray(3)

    /**
     * value数组大小
     */
    private val valueNum = 4

    /**
     * 是否上升的标志位
     */
    private var isDirectionUp = false

    /**
     * 用于存放计算阈值的波峰波谷差值
     */
    private val tempValue = FloatArray(valueNum)

    /**
     * 持续上升次数
     */
    private var continueUpCount = 0

    /**
     * 上一点的持续上升的次数，为了记录波峰的上升次数
     */
    private var continueUpFormerCount = 0

    /**
     * 上一点的状态，上升还是下降
     */
    private var lastStatus = false

    /**
     * 波峰值
     */
    private var peakOfWave = 0f

    /**
     * 波谷值
     */
    private var valleyOfWave = 0f

    /**
     * 此次波峰的时间
     */
    private var timeOfThisPeak: Long = 0

    /**
     * 上次波峰的时间
     */
    private var timeOfLastPeak: Long = 0

    /**
     * 当前的时间
     */
    private var timeOfNow: Long = 0

    /**
     * 当前传感器的值
     */
    private var gravityNew = 0f

    /**
     * 上次传感器的值
     */
    private var gravityOld = 0f

    /**
     * 动态阈值需要动态的数据，这个值用于这些动态数据的阈值
     */
    private val initialValue = 1.3f

    /**
     * 初始阈值
     */
    private var threadValue = 2.0f



    override fun onSensorChanged(event: SensorEvent) {
        if (event.sensor.type == Sensor.TYPE_GRAVITY) {
            System.arraycopy(event.values, 0, oriValues, 0,oriValues.size)
            val gravityNew =
                sqrt(oriValues[0] * oriValues[0] + oriValues[1] * oriValues[1] + oriValues[2] * oriValues[2])
//            if ( > 1 && mStepDetectionListener != null) {
//                onNewStepDetected()
//            }
            detectorNewStep(gravityNew)
        }
    }

    private fun detectorNewStep(values: Float) {
        if (gravityOld == 0f) {
            gravityOld = values
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

    @OnLifecycleEvent(Lifecycle.Event.ON_START)
    fun start() {
        sensorManager.registerListener(this, sensor, SensorManager.SENSOR_DELAY_NORMAL)
    }

    @OnLifecycleEvent(Lifecycle.Event.ON_STOP)
    fun stop() {
        sensorManager.unregisterListener(this)
    }

    interface StepDetectionListener {
        fun newStep(stepSize: Float)
    }

    init {
        sensor = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
    }

    override fun onAccuracyChanged(sensor: Sensor, accuracy: Int) {}

    /**
     * 1.通过波峰波谷的差值计算阈值
     * 2.记录4个值，存入tempValue数组中
     * 3.再将数组传入函数averageValue中计算阈值
     */
    @Suppress("unused")
    fun peakValleyThread(value: Float) {

    }


    /**
     * 梯度化阈值
     * 1.计算数组的均值
     * 2.通过均值将阈值梯度化在一个范围里
     */
    @Suppress("unused")
    fun averageValue(values: FloatArray, n: Int): Float {
        var average = 0f
        for (i in values) {
            average += i
        }
        average /= values.size
        average = if (average > 8) {
            4.3f
        } else if (average > 4 && average < 7) {
            2.3f
        } else if (average > 3 && average < 4) {
            2.0f
        } else {
            1.3f
        }
        return average
    }

}