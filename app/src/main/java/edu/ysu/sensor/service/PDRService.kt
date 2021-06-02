package edu.ysu.sensor.service

import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.PendingIntent
import android.app.Service
import android.content.Context
import android.content.Intent
import android.graphics.BitmapFactory
import android.hardware.SensorManager
import android.os.Build
import android.os.IBinder
import androidx.core.app.NotificationCompat
import edu.ysu.sensor.MainActivity
import edu.ysu.sensor.R
import edu.ysu.sensor.event.NewStepEvent
import edu.ysu.sensor.handler.DeviceAttitudeHandler
import edu.ysu.sensor.handler.StepDetectionHandler
import edu.ysu.sensor.util.PDRUtil
import org.greenrobot.eventbus.EventBus

/**
 * @author xrn1997
 * @date 2021/6/2
 */
class PDRService : Service() {

    private lateinit var mSensorManager: SensorManager

    private lateinit var deviceAttitudeHandler: DeviceAttitudeHandler
    private lateinit var stepDetectionHandler: StepDetectionHandler

    override fun onCreate() {
        super.onCreate()
        mSensorManager = getSystemService(Context.SENSOR_SERVICE) as SensorManager
        deviceAttitudeHandler = DeviceAttitudeHandler(mSensorManager)
        stepDetectionHandler = StepDetectionHandler(mSensorManager)
        initListener()
        initNotification()
    }


    private fun initListener() {
        stepDetectionHandler.setStepListener(object : StepDetectionHandler.StepDetectionListener {
            override fun newStep(stepSize: Float) {
                val newLocation =
                    PDRUtil.computeNextStep(stepSize, deviceAttitudeHandler.orientationValues[0])
                EventBus.getDefault().post(NewStepEvent(stepDetectionHandler.step.toString(), newLocation))
            }

        })
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
        startForeground(3, notification)
    }

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        stepDetectionHandler.start()
        deviceAttitudeHandler.start()
        return super.onStartCommand(intent, flags, startId)
    }

    override fun onDestroy() {
        super.onDestroy()
        stepDetectionHandler.stop()
        deviceAttitudeHandler.stop()
    }

    override fun onBind(intent: Intent): IBinder? {
        return null
    }
}