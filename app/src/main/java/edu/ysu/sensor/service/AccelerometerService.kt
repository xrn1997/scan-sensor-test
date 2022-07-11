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
import android.os.Environment
import android.os.IBinder
import androidx.core.app.NotificationCompat
import edu.ysu.sensor.MainActivity
import edu.ysu.sensor.MyApplication
import edu.ysu.sensor.R
import java.io.*

/**
 * @author xrn1997
 * @date 2021/6/19
 */
class AccelerometerService : Service(), SensorEventListener {
    private lateinit var mSensorManager: SensorManager
    private var accelerometer: Sensor? = null

    private lateinit var writer: BufferedWriter
    private lateinit var file: File


    override fun onCreate() {
        super.onCreate()
        mSensorManager = getSystemService(Context.SENSOR_SERVICE) as SensorManager
        initSensor()
        initNotification()
        initFile()
    }

    /**
     * 初始化文件
     */
    private fun initFile() {
        file = File(
            MyApplication.context.getExternalFilesDir(Environment.DIRECTORY_DOCUMENTS),
            "sensor" + System.currentTimeMillis() + ".csv"
        )
        if (!file.exists()) {
            file.createNewFile()
        }
        try {
            writer = BufferedWriter(OutputStreamWriter(FileOutputStream(file, true)))
        } catch (e: Exception) {
            e.printStackTrace()
        }

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

    private fun initSensor() {
        accelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
    }

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        // 为加速度传感器注册监听器
        mSensorManager.registerListener(this, accelerometer, SENSOR_DELAY_TYPE)
        return super.onStartCommand(intent, flags, startId)
    }

    override fun onSensorChanged(event: SensorEvent) {
        if (event.sensor.type == Sensor.TYPE_ACCELEROMETER) {
            try {
                writer.write("${event.timestamp},${event.values[0]},${event.values[1]},${event.values[2]}\n")
            } catch (e: IOException) {
                e.printStackTrace()
            }
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        mSensorManager.unregisterListener(this)
        writer.close()
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {}

    override fun onBind(intent: Intent?): IBinder? {
        return null
    }

    companion object {
        private const val SENSOR_DELAY_TYPE = 100000
    }
}