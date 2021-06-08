package edu.ysu.sensor.service

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
       // registerSensorListener(SensorManager.SENSOR_DELAY_GAME)
        registerSensorListener(SensorManager.SENSOR_DELAY_UI)
        return super.onStartCommand(intent, flags, startId)
    }
    /**
     * 初始化通知
     */
    private fun initNotification() {
        val notificationManager =
            getSystemService(NOTIFICATION_SERVICE) as NotificationManager
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            val channel = NotificationChannel("传感器服务", "通知", NotificationManager.IMPORTANCE_DEFAULT)
            notificationManager.createNotificationChannel(channel)
        }
        val intent = Intent(this, MainActivity::class.java)
        val pi = PendingIntent.getActivity(this, 0, intent, 0)
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
     * @param sensorDelayType 数据更新模式
     */
    private fun registerSensorListener(sensorDelayType:Int) {
        //为气压传感器注册监听器
        mSensorManager.registerListener(this, pressure, sensorDelayType)
        // 为加速度传感器注册监听器
        mSensorManager.registerListener(this, accelerometer, sensorDelayType)
        // 为陀螺仪传感器注册监听器
        mSensorManager.registerListener(this, gyroscope, sensorDelayType)
        // 为磁场传感器注册监听器
        mSensorManager.registerListener(this, magneticField, sensorDelayType)
        // 为重力传感器注册监听器
        mSensorManager.registerListener(this, gravity, sensorDelayType)
        // 为线性加速度传感器注册监听器
        mSensorManager.registerListener(this, linearAcceleration, sensorDelayType)
        //为温度传感器注册监听器
        mSensorManager.registerListener(this, ambientTemperature, sensorDelayType)
        //为湿度传感器注册监听器
        mSensorManager.registerListener(this, relativeHumidity, sensorDelayType)
        //为旋转矢量传感器注册监听器
        mSensorManager.registerListener(this, rotationVector, sensorDelayType)
        //为近距离传感器注册监听器
        mSensorManager.registerListener(this, proximity, sensorDelayType)
        //为光线传感器注册监听器
        mSensorManager.registerListener(this, light, sensorDelayType)
    }

    private fun initSensor() {
        pressure = mSensorManager.getDefaultSensor(Sensor.TYPE_PRESSURE)
        accelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        gyroscope = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
        magneticField = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD)
        gravity = mSensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY)
        linearAcceleration = mSensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION)
        light = mSensorManager.getDefaultSensor(Sensor.TYPE_LIGHT)
        rotationVector = mSensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)
        relativeHumidity = mSensorManager.getDefaultSensor(Sensor.TYPE_RELATIVE_HUMIDITY)
        proximity = mSensorManager.getDefaultSensor(Sensor.TYPE_PROXIMITY)
        ambientTemperature = mSensorManager.getDefaultSensor(Sensor.TYPE_AMBIENT_TEMPERATURE)
    }
}