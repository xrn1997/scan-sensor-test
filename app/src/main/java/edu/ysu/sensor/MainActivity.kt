package edu.ysu.sensor

import android.content.Intent
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.os.Build
import androidx.appcompat.app.AppCompatActivity
import android.os.Bundle
import android.util.Log
import android.view.View
import edu.ysu.sensor.databinding.ActivityMainBinding
import edu.ysu.sensor.entity.Location
import edu.ysu.sensor.event.NewStepEvent
import edu.ysu.sensor.service.PDRService
import edu.ysu.sensor.service.SensorService
import org.greenrobot.eventbus.EventBus
import org.greenrobot.eventbus.Subscribe
import org.greenrobot.eventbus.ThreadMode
import kotlin.math.pow
import kotlin.math.sqrt

class MainActivity : AppCompatActivity() {
    private lateinit var binding: ActivityMainBinding

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityMainBinding.inflate(layoutInflater)
        val rootView: View = binding.root
        setContentView(rootView)
        EventBus.getDefault().register(this)
        initListener()
    }


    private fun initListener() {
        binding.button.setOnClickListener {
            val intent = Intent(this, SensorService::class.java)
            if (binding.button.text == resources.getText(R.string.start_scan_sensor_data)) {
                binding.button.text = resources.getText(R.string.stop_scan_sensor_data)
                if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
                    startForegroundService(intent);
                } else {
                    startService(intent);
                }
            } else if (binding.button.text == resources.getText(R.string.stop_scan_sensor_data)) {
                binding.button.text = resources.getText(R.string.start_scan_sensor_data)
                stopService(intent)
            }
        }
        binding.button2.setOnClickListener {
            val intent = Intent(this, PDRService::class.java)
            if (binding.button2.text == resources.getText(R.string.start_pdr)) {
                binding.button2.text = resources.getText(R.string.stop_pdr)
                startService(intent)
            } else if (binding.button2.text == resources.getText(R.string.stop_pdr)) {
                binding.button2.text = resources.getText(R.string.start_pdr)
                stopService(intent)
            }
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        EventBus.getDefault().unregister(this)
    }

    /**
     * 更新UI
     */
    @Subscribe(threadMode = ThreadMode.MAIN)
    fun onSensorChanged(event: SensorEvent) {
        val values = event.values
        when (event.sensor.type) {
            Sensor.TYPE_ACCELEROMETER ->
                binding.textView1.text = ("X${values[0]},Y${values[1]},Z${values[2]}")
            Sensor.TYPE_GYROSCOPE ->
                binding.textView2.text = ("X${values[0]},Y${values[1]},Z${values[2]}")
            Sensor.TYPE_MAGNETIC_FIELD ->
                binding.textView3.text = ("X${values[0]},Y${values[1]},Z${values[2]}")
            Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED ->{
                binding.textView03.text=(resources.getText(R.string.magneticField).toString() +"未经校准")
                binding.textView3.text = ("X${values[0]},Y${values[1]},Z${values[2]}")
            }

            Sensor.TYPE_GRAVITY -> {
                Log.e(
                    "重力向量", "X${values[0]}  Y${values[1]}  Z${values[2]}" +
                            " ${
                                sqrt(
                                    values[0].toDouble().pow(2.0) + values[1].toDouble()
                                        .pow(2.0) + values[2].toDouble().pow(2.0)
                                )
                            }"
                )
                binding.textView4.text = ("X${values[0]},Y${values[1]},Z${values[2]}")
            }
            Sensor.TYPE_LINEAR_ACCELERATION ->
                binding.textView5.text = ("X${values[0]},Y${values[1]},Z${values[2]}")
            Sensor.TYPE_PRESSURE ->
                binding.textView6.text = ("${values[0]}")
            Sensor.TYPE_LIGHT ->
                binding.textView7.text = ("${values[0]}")
            Sensor.TYPE_PROXIMITY ->
                binding.textView8.text = ("${values[0]}")
            Sensor.TYPE_ROTATION_VECTOR ->
                binding.textView9.text = ("X${values[0]},Y${values[1]},Z${values[2]}")
            Sensor.TYPE_RELATIVE_HUMIDITY ->
                binding.textView10.text = ("${values[0]}")
            Sensor.TYPE_AMBIENT_TEMPERATURE ->
                binding.textView11.text = ("${values[0]}")
        }
    }

    @Subscribe(threadMode = ThreadMode.MAIN)
    fun pdrStepEvent(newStepEvent: NewStepEvent) {
        val newLocation: Location = newStepEvent.data
        binding.textView12.text = (
                "最新位置:(${newLocation.x}，${newLocation.y}) \n" +
                        "步数: ${newStepEvent.msg} \n")
    }
}