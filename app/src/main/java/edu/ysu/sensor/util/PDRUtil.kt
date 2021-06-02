package edu.ysu.sensor.util

import edu.ysu.sensor.entity.Location
import kotlin.math.cos
import kotlin.math.sin


/**
 * 计算当前的坐标位置类(PDR算法)
 * @author xrn1997
 * @date 2021/6/2
 */
object PDRUtil {
    /**
     * 当前坐标
     */
    private var mCurrentLocation: Location

    /**
     * 重新更正位置坐标
     * @param location 相对坐标，左下角为（0.0，0.0）
     */
    fun reInitLocation(location: Location) {
        mCurrentLocation = location
    }

    /**
     * 根据步长和方向角计算下一步的坐标位置（PDR算法）
     * @param stepSize 步长
     * @param bearing 方向角
     * @return 返回下一步的坐标
     */
    fun computeNextStep(stepSize: Float, bearing: Float): Location {
        val newLocation = mCurrentLocation.copy()
        val newX = mCurrentLocation.x - stepSize * sin(bearing * Math.PI / 180)
        val newY = mCurrentLocation.y - stepSize * cos(bearing * Math.PI / 180)
        newLocation.x = newX
        newLocation.y = newY
        mCurrentLocation = newLocation
        return newLocation
    }

    init {
        mCurrentLocation = Location(0.0, 0.0)
    }
}