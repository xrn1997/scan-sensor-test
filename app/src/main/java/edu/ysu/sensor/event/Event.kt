package edu.ysu.sensor.event

import edu.ysu.sensor.entity.Location

/**
 * 迈步
 * @author xrn1997
 * @date 2021/6/2
 */
class NewStepEvent(msg: String, data: Location) : BaseEvent<Location>(msg, data)
