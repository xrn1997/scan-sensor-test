package edu.ysu.sensor

import android.annotation.SuppressLint
import android.content.Intent
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.os.Bundle
import android.util.Log
import android.view.View
import androidx.appcompat.app.AppCompatActivity
import edu.ysu.sensor.databinding.ActivityMainBinding
import edu.ysu.sensor.entity.Location
import edu.ysu.sensor.event.NewStepEvent
import edu.ysu.sensor.service.AccelerometerService
import edu.ysu.sensor.service.PDRService
import edu.ysu.sensor.service.SensorService
import org.greenrobot.eventbus.EventBus
import org.greenrobot.eventbus.Subscribe
import org.greenrobot.eventbus.ThreadMode

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
        binding.button0.setOnClickListener {
            val intent = Intent(this, SensorService::class.java)
            if (binding.button0.text == resources.getText(R.string.start_scan_sensor_data)) {
                binding.button0.text = resources.getText(R.string.stop_scan_sensor_data)
                startService(intent)
            } else if (binding.button0.text == resources.getText(R.string.stop_scan_sensor_data)) {
                binding.button0.text = resources.getText(R.string.start_scan_sensor_data)
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
        binding.button3.setOnClickListener {
            val intent = Intent(this, AccelerometerService::class.java)
            if (binding.button3.text == resources.getText(R.string.start_collect)) {
                binding.button3.text = resources.getText(R.string.stop_collect)
                startService(intent)
            } else if (binding.button3.text == resources.getText(R.string.stop_collect)) {
                binding.button3.text = resources.getText(R.string.start_collect)
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
    @SuppressLint("SetTextI18n")
    @Suppress("unused")
    fun onSensorChanged(event: SensorEvent) {
        val values = event.values

        when (event.sensor.type) {
            Sensor.TYPE_ACCELEROMETER ->
                binding.textView1.text = ("X${values[0]},Y${values[1]},Z${values[2]}")
            Sensor.TYPE_GYROSCOPE ->
                binding.textView2.text = ("X${values[0]},Y${values[1]},Z${values[2]}")
            Sensor.TYPE_MAGNETIC_FIELD ->
                binding.textView3.text = ("X${values[0]},Y${values[1]},Z${values[2]}")
            Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED -> {
                binding.textView03.text =
                    (resources.getText(R.string.magneticField).toString() + "未经校准")
                binding.textView3.text = ("X${values[0]},Y${values[1]},Z${values[2]}")
            }
            Sensor.TYPE_GRAVITY -> {
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

    @SuppressLint("SetTextI18n")
    @Subscribe(threadMode = ThreadMode.MAIN)
    @Suppress("unused")
    fun pdrStepEvent(newStepEvent: NewStepEvent) {
        val newLocation: Location = newStepEvent.data
        Log.e(
            "测试", "最新位置:(${newLocation.x}，${newLocation.y}) \n" +
                    "步数: ${newStepEvent.msg} \n"
        )
        binding.textView12.text = (
                "最新位置:(${newLocation.x}，${newLocation.y}) \n" +
                        "步数: ${newStepEvent.msg} \n")
    }
}