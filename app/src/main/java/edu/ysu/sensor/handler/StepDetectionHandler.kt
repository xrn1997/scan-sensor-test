package edu.ysu.sensor.handler

import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.util.Log
import androidx.lifecycle.DefaultLifecycleObserver
import androidx.lifecycle.LifecycleObserver
import androidx.lifecycle.LifecycleOwner
import kotlin.math.sqrt


/**
 * 步态检测类，用来检测是否走了一步。开发中。
 * @author xrn1997
 * @date 2021/6/1
 */
@Suppress("unused")
class StepDetectionHandler(
    private var sensorManager: SensorManager
) : DefaultLifecycleObserver, SensorEventListener, LifecycleObserver {

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
    private var tempCount = 0

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

    //初始范围
    private var minValue = 11f
    private var maxValue = 19.6f

    //波峰波谷时间差
    private var timeInterval = 250

    override fun onSensorChanged(event: SensorEvent) {
        if (event.sensor.type == Sensor.TYPE_ACCELEROMETER) {
            System.arraycopy(event.values, 0, oriValues, 0, oriValues.size)
            val gravityNew =
                sqrt(oriValues[0] * oriValues[0] + oriValues[1] * oriValues[1] + oriValues[2] * oriValues[2])
//            if ( > 1 && mStepDetectionListener != null) {
//                onNewStepDetected()
//            }
            detectorNewStep(gravityNew)
        }
    }

    /**
     *
     * 检测步子，并开始计步
     * 1.传入sensor中的数据
     * 2.如果检测到了波峰，并且符合时间差以及阈值的条件，则判定为1步
     * 3.符合时间差条件，波峰波谷差值大于initialValue，则将该差值纳入阈值的计算中
     *
     * @param values Float
     */
    private fun detectorNewStep(values: Float) {
        if (gravityOld == 0f) {
            gravityOld = values
        } else {
            val detectResult = detectorPeak(values, gravityOld)
            if (detectResult) {
                timeOfLastPeak = timeOfThisPeak
                timeOfNow = System.currentTimeMillis()
                if (timeOfNow - timeOfLastPeak >= timeInterval && peakOfWave - valleyOfWave >= threadValue) {
                    timeOfThisPeak = timeOfNow
                    //更新界面数据
                    onNewStepDetected()
                }
                if (timeOfNow - timeOfLastPeak >= 200 && (peakOfWave - valleyOfWave >= initialValue)) {
                    timeOfThisPeak = timeOfNow
                    threadValue = peakValleyThread(peakOfWave - valleyOfWave)
                }
            }
        }
        gravityOld = values
    }


    private fun onNewStepDetected() {
        val distanceStep = 0.8F
        step++
        mStepDetectionListener?.newStep(distanceStep)
    }

    fun setStepListener(listener: StepDetectionListener) {
        mStepDetectionListener = listener
    }

    override fun onStart(owner: LifecycleOwner) {
        super.onStart(owner)
        sensorManager.registerListener(this, sensor, SensorManager.SENSOR_DELAY_NORMAL)
    }

    override fun onStop(owner: LifecycleOwner) {
        super.onStop(owner)
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
    private fun peakValleyThread(value: Float): Float {
        var tempThread = threadValue
        if (tempCount < valueNum) {
            tempValue[tempCount] = value
            tempCount++
        } else {
            tempThread = averageValue(tempValue)
            for (i in 1 until valueNum) {
                tempValue[i - 1] = tempValue[i]
            }
            tempValue[valueNum - 1] = value
        }
        return tempThread

    }

    /**
     *
     * 检测波峰
     * 以下四个条件判断为波峰：
     * 1.目前点为下降的趋势：isDirectionUp为false
     * 2.之前的点为上升的趋势：lastStatus为true
     * 3.到波峰为止，持续上升大于等于2次
     * 4.波峰值大于1.2g,小于2g
     * 记录波谷值
     * 1.观察波形图，可以发现在出现步子的地方，波谷的下一个就是波峰，有比较明显的特征以及差值
     * 2.所以要记录每次的波谷值，为了和下次的波峰做对比
     *
     * @param newValue Float
     * @param oldValue Float
     * @return Boolean
     */
    private fun detectorPeak(newValue: Float, oldValue: Float): Boolean {
        lastStatus = isDirectionUp
        if (newValue >= oldValue) {
            isDirectionUp = true
            continueUpCount++
        } else {
            continueUpFormerCount = continueUpCount
            continueUpCount = 0
            isDirectionUp = false
        }

//    Log.v(TAG, "oldValue:" + oldValue);
        return if (!isDirectionUp && lastStatus && continueUpFormerCount >= 2 && oldValue >= minValue && oldValue < maxValue) {
            peakOfWave = oldValue
            true
        } else if (!lastStatus && isDirectionUp) {
            valleyOfWave = oldValue
            false
        } else {
            false
        }
    }

    /**
     * 梯度化阈值
     * 1.计算数组的均值
     * 2.通过均值将阈值梯度化在一个范围里
     */
    @Suppress("unused")
    fun averageValue(values: FloatArray): Float {
        var average = 0f
        for (i in values) {
            average += i
        }
        average /= values.size
        average = if (average >= 8) {
            4.3f
        } else if (average >= 7) {
            3.3f
        } else if (average >= 4) {
            2.3f
        } else if (average >= 3) {
            2.0f
        } else {
            1.3f
        }
        return average
    }

}