package edu.ysu.sensor

import android.content.Intent
import android.hardware.Sensor
import android.hardware.SensorEvent
import androidx.appcompat.app.AppCompatActivity
import android.os.Bundle
import android.view.View
import edu.ysu.sensor.databinding.ActivityMainBinding
import edu.ysu.sensor.entity.Location
import edu.ysu.sensor.event.NewStepEvent
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
        binding.button.setOnClickListener {
            val intent = Intent(this, SensorService::class.java)
            if (binding.button.text == resources.getText(R.string.start_scan_sensor_data)) {
                binding.button.text = resources.getText(R.string.stop_scan_sensor_data)
                startService(intent)
            } else if (binding.button.text == resources.getText(R.string.stop_scan_sensor_data)) {
                binding.button.text = resources.getText(R.string.start_scan_sensor_data)
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
            Sensor.TYPE_GRAVITY ->
                binding.textView4.text = ("X${values[0]},Y${values[1]},Z${values[2]}")
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