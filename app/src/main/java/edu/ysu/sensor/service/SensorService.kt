package edu.ysu.sensor.service

import android.annotation.SuppressLint
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.PendingIntent
import android.app.Service
import android.content.Context
import android.content.Intent
import android.graphics.BitmapFactory
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Build
import android.os.IBinder
import androidx.core.app.NotificationCompat
import edu.ysu.sensor.MainActivity
import edu.ysu.sensor.R
import org.greenrobot.eventbus.EventBus

/**
 * @author xrn1997
 * @date 2021/6/1
 */
class SensorService : Service(), SensorEventListener {
    private lateinit var mSensorManager: SensorManager
    private var pressure: Sensor? = null
    private var accelerometer: Sensor? = null
    private var gyroscope: Sensor? = null
    private var magneticField: Sensor? = null
    private var gravity: Sensor? = null
    private var linearAcceleration: Sensor? = null
    private var light: Sensor? = null
    private var rotationVector: Sensor? = null
    private var relativeHumidity: Sensor? = null
    private var proximity: Sensor? = null
    private var ambientTemperature: Sensor? = null


    override fun onCreate() {
        super.onCreate()
        mSensorManager = getSystemService(Context.SENSOR_SERVICE) as SensorManager
        initSensor()
        initNotification()
    }

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        registerSensorListener()
        return super.onStartCommand(intent, flags, startId)
    }

    /**
     * 初始化通知
     */
    @SuppressLint("UnspecifiedImmutableFlag")
    private fun initNotification() {
        val notificationManager =
            getSystemService(NOTIFICATION_SERVICE) as NotificationManager
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            val channel = NotificationChannel("传感器服务", "通知", NotificationManager.IMPORTANCE_DEFAULT)
            notificationManager.createNotificationChannel(channel)
        }
        val intent = Intent(this, MainActivity::class.java)
        val pi = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            PendingIntent.getActivity(this, 0, intent, PendingIntent.FLAG_IMMUTABLE)
        } else {
            PendingIntent.getActivity(this, 0, intent, PendingIntent.FLAG_UPDATE_CURRENT)
        }
        val notification = NotificationCompat.Builder(this, "传感器服务")
            .setContentTitle("正在扫描传感器")
            .setContentText("获得传感器数据")
            .setSmallIcon(R.mipmap.ic_launcher)
            .setLargeIcon(BitmapFactory.decodeResource(resources, R.mipmap.ic_launcher))
            .setContentIntent(pi)
            .build()
        startForeground(2, notification)
    }

    override fun onDestroy() {
        super.onDestroy()
        mSensorManager.unregisterListener(this)
    }

    override fun onBind(intent: Intent): IBinder? {
        return null
    }

    override fun onSensorChanged(event: SensorEvent) {
        EventBus.getDefault().post(event)
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {
    }

    /**
     * 注册传感器Listener
     * 加速度传感器 = 重力传感器+线性加速度传感器。
     */
    private fun registerSensorListener() {
        //为气压传感器注册监听器
        mSensorManager.registerListener(this, pressure, SENSOR_DELAY_TYPE)
        // 为加速度传感器注册监听器
        mSensorManager.registerListener(this, accelerometer, SENSOR_DELAY_TYPE)
        // 为陀螺仪传感器注册监听器
        mSensorManager.registerListener(this, gyroscope, SENSOR_DELAY_TYPE)
        // 为磁场传感器注册监听器
        mSensorManager.registerListener(this, magneticField, SENSOR_DELAY_TYPE)
        // 为重力传感器注册监听器
        mSensorManager.registerListener(this, gravity, SENSOR_DELAY_TYPE)
        // 为线性加速度传感器注册监听器
        mSensorManager.registerListener(this, linearAcceleration, SENSOR_DELAY_TYPE)
        //为温度传感器注册监听器
        mSensorManager.registerListener(this, ambientTemperature, SENSOR_DELAY_TYPE)
        //为湿度传感器注册监听器
        mSensorManager.registerListener(this, relativeHumidity, SENSOR_DELAY_TYPE)
        //为旋转矢量传感器注册监听器
        mSensorManager.registerListener(this, rotationVector, SENSOR_DELAY_TYPE)
        //为近距离传感器注册监听器
        mSensorManager.registerListener(this, proximity, SENSOR_DELAY_TYPE)
        //为光线传感器注册监听器
        mSensorManager.registerListener(this, light, SENSOR_DELAY_TYPE)
    }

    private fun initSensor() {
        pressure = mSensorManager.getDefaultSensor(Sensor.TYPE_PRESSURE)
        accelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        gyroscope = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
        /**
         * 校准过的磁力计
         */
        magneticField = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD)
        /**
         * 未校准过的磁力计
         */
//        magneticField = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED)
        gravity = mSensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY)
        linearAcceleration = mSensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION)
        light = mSensorManager.getDefaultSensor(Sensor.TYPE_LIGHT)
        rotationVector = mSensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)
        relativeHumidity = mSensorManager.getDefaultSensor(Sensor.TYPE_RELATIVE_HUMIDITY)
        proximity = mSensorManager.getDefaultSensor(Sensor.TYPE_PROXIMITY)
        ambientTemperature = mSensorManager.getDefaultSensor(Sensor.TYPE_AMBIENT_TEMPERATURE)
    }

    companion object {
        /**
         * 延迟100ms,即10Hz
         */
        private const val SENSOR_DELAY_100 = 100000

        /**
         * 修改采样周期。
         * SensorManager.SENSOR_DELAY_FASTEST = 0 没有延迟，能多快就多快
         * SensorManager.SENSOR_DELAY_GAME = 1 延迟20 ms，即50Hz
         * SensorManager.SENSOR_DELAY_UI  = 2延迟66.667 ms，即15Hz
         * SensorManager.SENSOR_DELAY_NORMAL = 3 延迟200 ms，即5Hz
         * @see SensorManager.SENSOR_DELAY_UI 延迟100ms,即10Hz
         */
        private const val SENSOR_DELAY_TYPE = SensorManager.SENSOR_DELAY_NORMAL
        // private const val SENSOR_DELAY_TYPE = SENSOR_DELAY_100
    }
}