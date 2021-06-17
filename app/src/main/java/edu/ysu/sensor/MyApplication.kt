package edu.ysu.sensor

import android.annotation.SuppressLint
import android.app.Application
import android.content.Context

/**
 * @author xrn1997
 * @date 2021/6/17
 */
class MyApplication: Application() {
    companion object{
        @SuppressLint("StaticFieldLeak")
        @JvmStatic
        lateinit var context: Context
    }
    override fun onCreate() {
        super.onCreate()
        context =applicationContext
    }
}