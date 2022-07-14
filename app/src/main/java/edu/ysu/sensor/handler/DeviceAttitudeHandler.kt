package edu.ysu.sensor.handler

import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.util.Log
import androidx.lifecycle.DefaultLifecycleObserver
import androidx.lifecycle.LifecycleObserver
import androidx.lifecycle.LifecycleOwner
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Job
import kotlinx.coroutines.launch


/**
 * 航向检测类，用来检测行人走动的方向
 * @author xrn1997
 * @date 2021/6/2
 */
class DeviceAttitudeHandler(
    private var sensorManager: SensorManager
) : DefaultLifecycleObserver, SensorEventListener, LifecycleObserver {

    private var magneticField: Sensor? = null
    private var accelerometer: Sensor? = null
    private var rotationVector: Sensor? = null

    private val accelerometerReading = FloatArray(3)
    private val magnetometerReading = FloatArray(3)
    private val rotationVectorReading = FloatArray(3)

    /**
     * 重力计和磁力计计算出的旋转矩阵
     */
    private val rotationMatrix = FloatArray(9)

    /**
     * 旋转矢量（重力计、磁力计、陀螺仪）计算出的旋转矩阵
     */
    private val rotationMatrixFromVector = FloatArray(9)

    /**
     * 根据加速度计和磁力计得出的方向
     */
    val orientationAngles = FloatArray(3)

    /**
     *  根据旋转矢量得出的角度
     */
    private val orientationAnglesFromVector = FloatArray(3)

    private val job = Job()

    override fun onSensorChanged(event: SensorEvent) {
        val scope = CoroutineScope(job)

        when (event.sensor.type) {
            Sensor.TYPE_ACCELEROMETER -> {
                System.arraycopy(
                    event.values,
                    0,
                    accelerometerReading,
                    0,
                    accelerometerReading.size
                )

            }
            Sensor.TYPE_MAGNETIC_FIELD -> {
                System.arraycopy(event.values, 0, magnetometerReading, 0, magnetometerReading.size)
            }
            Sensor.TYPE_ROTATION_VECTOR -> {
                System.arraycopy(
                    event.values,
                    0,
                    rotationVectorReading,
                    0,
                    rotationVectorReading.size
                )
            }
        }
        scope.launch {
            updateOrientationAngles()
        }
    }

    override fun onStart(owner: LifecycleOwner) {
        super.onStart(owner)
        // 为加速度传感器注册监听器
        sensorManager.registerListener(this, accelerometer, SensorManager.SENSOR_DELAY_NORMAL)
        // 为磁场传感器注册监听器
        sensorManager.registerListener(this, magneticField, SensorManager.SENSOR_DELAY_NORMAL)
        //旋转矢量传感器注册监听器
        sensorManager.registerListener(this, rotationVector, SensorManager.SENSOR_DELAY_NORMAL)
    }

    override fun onStop(owner: LifecycleOwner) {
        super.onStop(owner)
        job.cancel()
        sensorManager.unregisterListener(this)
    }

    init {
        accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        magneticField = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD)
        rotationVector = sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)
    }

    private fun updateOrientationAngles() {
        SensorManager.getRotationMatrix(
            rotationMatrix,
            null,
            accelerometerReading,
            magnetometerReading
        )
        SensorManager.getOrientation(rotationMatrix, orientationAngles)

//        Log.d(
//            "对比",
//            "getRotationMatrix计算的旋转矩阵：" +
//                    " ${rotationMatrix[0]}   ${rotationMatrix[1]}    ${rotationMatrix[2]} \n" +
//                    "${rotationMatrix[3]}   ${rotationMatrix[4]}    ${rotationMatrix[5]} \n" +
//                    "${rotationMatrix[6]}   ${rotationMatrix[7]}    ${rotationMatrix[8]} \n"
//        )
//        Log.d(
//            "对比", "重力计读数：" +
//                    "${gravityReading[0]}   ${gravityReading[1]}  ${gravityReading[2]} " +
//                    "加速度计使用低通滤波过滤出来的重力：" +
//                    "${gravityCalculating[0]}   ${gravityCalculating[1]}  ${gravityCalculating[2]}"
//        )
        SensorManager.getRotationMatrixFromVector(
            rotationMatrixFromVector,
            rotationVectorReading
        )
//        Log.d("对比", "通过陀螺仪旋转矢量获得的旋转矩阵：" +
//                " ${rotationMatrixFromVector[0]}   ${rotationMatrixFromVector[1]}    ${rotationMatrixFromVector[2]} \n" +
//                "${rotationMatrixFromVector[3]}   ${rotationMatrixFromVector[4]}    ${rotationMatrixFromVector[5]} \n" +
//                "${rotationMatrixFromVector[6]}   ${rotationMatrixFromVector[7]}    ${rotationMatrixFromVector[8]} \n"
//        )
        SensorManager.getOrientation(rotationMatrixFromVector, orientationAnglesFromVector)
//        Log.d(
//            "对比", "通过getRotationMatrix计算的旋转矩阵欧拉角：" +
//                    "${orientationAngles[0] * 180 / Math.PI}   ${orientationAngles[1] * 180 / Math.PI}  ${orientationAngles[2] * 180 / Math.PI}  \n" +
//                    "通过getRotationMatrixFromVector计算的旋转矩阵欧拉角：" +
//                    "${orientationAnglesFromVector[0] * 180 / Math.PI}   ${orientationAnglesFromVector[1] * 180 / Math.PI}  ${orientationAnglesFromVector[2] * 180 / Math.PI} \n"
//        )
    }


    override fun onAccuracyChanged(sensor: Sensor, accuracy: Int) {}
}